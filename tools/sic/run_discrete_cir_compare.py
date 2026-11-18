#!/usr/bin/env python3
"""Sequential, provenance-bound comparison of full-packet SIC backends.

Old backend: ``--reconstruction full`` (path_fractional reference).
New backend: ``--reconstruction discrete_cir`` (direct discrete FIR estimate).

This driver only orchestrates the existing C++ benchmark and the independent
offline verifier; it does not import or duplicate the C++ implementation.
Formal timing and functional runs take an exclusive ``/tmp/uwb-sic-benchmark.lock``
and never overlap.  A timing run is refused unless a passed functional summary
for the same binary/commit/library is supplied with ``--quality-summary``.
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
VERIFIER = ROOT / "tools/sic/verify_discrete_cir_compare.py"
QA = ROOT / "tools/sic/qa_discrete_cir_compare.py"
LOCK = Path("/tmp/uwb-sic-benchmark.lock")
REPRESENTATIVE = ["sync_base", "payload_base", "payload_len16", "payload_len127",
                  "payload_snr8", "payload_isr0", "payload_isr20",
                  "payload_cfo_m50k", "payload_cfo_p50k", "payload_four_frac"]
NEGATIVE_CELLS = ["no_interference", "no_overlap"]
TIMING_CELLS = ["sync_base", "payload_base"]
DEFAULT_CELLS = {"functional": REPRESENTATIVE,
                 "negative": NEGATIVE_CELLS,
                 "timing": TIMING_CELLS}
REJECT_REASONS = {"queue_full", "source_late", "cancelled", "not_submitted"}
SCHEMA = "sic-discrete-cir-experiment/1"
FIXED_SOURCES = (
    "gr-uwb/apps/benchmark_fullpacket_compare.cc",
    "gr-uwb/apps/fullpacket_rx_pipeline.h",
    "gr-uwb/include/gnuradio/uwb/uwb_fullpacket_sic_core.h",
    "gr-uwb/lib/uwb_fullpacket_sic_core.cc",
    "gr-uwb/include/gnuradio/uwb/uwb_discrete_cir_sic.h",
    "gr-uwb/lib/uwb_preamble_sic_core.cc",
    "gr-uwb/include/gnuradio/uwb/uwb_hrp_mod_core.h",
    "gr-uwb/include/gnuradio/uwb/uwb_demod_core.h",
)
DISCOVERED_SOURCES = ("gr-uwb/lib/*discrete*.cc",
                      "gr-uwb/include/gnuradio/uwb/*discrete*.h")
DISCLAIMER = ("finite-sample functional gate on the declared source windows; "
              "not a proof of a general decode rate. MATLAB, native, hardware "
              "and real-time throughput are not certified by this experiment.")
TIMING_GATE = {"mean_ratio_max": 0.90,
               "rule": "new mean <= 0.90 * old mean and new P95 <= old P95"}


# --------------------------------------------------------------------------
# generic helpers
# --------------------------------------------------------------------------
def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True,
                                    allow_nan=False) + "\n")


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def git_output(*args):
    return subprocess.run(["git", "-C", str(ROOT), *args], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          check=True).stdout.strip()


def clean_env():
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    env["OPENBLAS_NUM_THREADS"] = "1"
    env["OMP_NUM_THREADS"] = "1"
    return env


def run_logged(command, log, env):
    # List arguments preserve exact bytes; nothing is shell-interpolated.
    log = Path(log)
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open("w") as stream:
        completed = subprocess.run([str(x) for x in command], cwd=ROOT, env=env,
                                   stdout=stream, stderr=subprocess.STDOUT)
    return completed.returncode


def source_index(value):
    if type(value) is int:
        return value
    text = str(value)
    if text.isdigit():
        return int(text)
    name = Path(text).name
    if name.startswith("w") and name[1:].isdigit():
        return int(name[1:])
    raise ValueError(f"invalid source_window: {value}")


def row_rejected(row):
    return row.get("rejected") is True or row.get("reason") in REJECT_REASONS


def load_rows(run):
    path = Path(run) / "windows.jsonl"
    if not path.is_file():
        raise RuntimeError(f"missing terminal rows: {path}")
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def read_json(path):
    return json.loads(Path(path).read_text())


# --------------------------------------------------------------------------
# binary / provenance binding
# --------------------------------------------------------------------------
def select_binary(build, name):
    if name:
        if "/" in name:
            raise ValueError("--binary takes a file name inside <build>/apps")
        path = build / "apps" / name
        if not path.is_file():
            raise RuntimeError(f"selected binary missing: {path}")
        return path
    for candidate in ("benchmark_fullpacket_compare_fast_rx",
                      "benchmark_fullpacket_compare"):
        path = build / "apps" / candidate
        if path.is_file():
            return path
    raise RuntimeError(f"no full-packet benchmark under {build}/apps")


def binary_contains(binary, needle):
    needle = needle.encode() if isinstance(needle, str) else needle
    with Path(binary).open("rb") as stream:
        previous = b""
        while True:
            block = stream.read(1 << 20)
            if not block:
                return False
            if needle in previous + block:
                return True
            previous = block[-len(needle):]


def library_binding(binary, build, env):
    completed = subprocess.run(["ldd", str(binary)], env=env, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               check=True)
    libraries = []
    for line in completed.stdout.splitlines():
        if "libgnuradio-uwb" not in line or "=>" not in line:
            continue
        candidate = line.split("=>", 1)[1].strip().split()[0]
        if candidate == "not":
            raise RuntimeError("runner uwb library cannot be resolved")
        path = Path(candidate).resolve()
        if not path.is_relative_to(build.resolve()):
            raise RuntimeError(
                f"runner resolves uwb library outside the selected build: {path}")
        libraries.append({"path": str(path), "sha256": sha256(path)})
    if not libraries:
        raise RuntimeError("ldd did not identify the runner's uwb library")
    return completed.stdout, libraries


def source_inventory():
    paths, missing = [], []
    for relative in FIXED_SOURCES:
        path = ROOT / relative
        if path.is_file():
            paths.append(path)
        else:
            missing.append(relative)
    for pattern in DISCOVERED_SOURCES:
        paths.extend(sorted(ROOT.glob(pattern)))
    paths.extend([VERIFIER, QA, Path(__file__).resolve()])
    hashes = {}
    for path in sorted(set(path.resolve() for path in paths if path.is_file())):
        try:
            relative = str(path.relative_to(ROOT))
        except ValueError:
            relative = str(path)
        hashes[relative] = sha256(path)
    required = {"gr-uwb/apps/benchmark_fullpacket_compare.cc",
                "gr-uwb/apps/fullpacket_rx_pipeline.h",
                "gr-uwb/include/gnuradio/uwb/uwb_discrete_cir_sic.h",
                "gr-uwb/include/gnuradio/uwb/uwb_fullpacket_sic_core.h",
                "tools/sic/verify_discrete_cir_compare.py"}
    for relative in required:
        if relative not in hashes:
            raise RuntimeError(f"required source missing: {relative}")
    return hashes, missing


def cpu_model():
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "unknown"


def provenance(binary, build, env, args):
    flags_file = build / "apps/CMakeFiles" / (binary.name + ".dir") / "flags.make"
    if not flags_file.is_file():
        raise RuntimeError(f"selected runner compile flags missing: {flags_file}")
    ldd_text, libraries = library_binding(binary, build, env)
    status = git_output("status", "--porcelain=v1", "--untracked-files=normal")
    sources, missing = source_inventory()
    return {
        "schema": "sic-discrete-cir-provenance/1",
        "created_utc": utc_now(),
        "repo": str(ROOT),
        "git_head": git_output("rev-parse", "HEAD"),
        "git_branch": git_output("rev-parse", "--abbrev-ref", "HEAD"),
        "git_dirty": bool(status),
        "git_status": status.splitlines(),
        "build_dir": str(build),
        "binary": {"path": str(binary), "name": binary.name,
                   "sha256": sha256(binary)},
        "binary_markers": {
            "discrete_cir": binary_contains(binary, "discrete_cir"),
        },
        "compile_flags": {"path": str(flags_file), "sha256": sha256(flags_file),
                          "content": flags_file.read_text()},
        "libraries": libraries,
        "ldd": ldd_text,
        "sources": sources,
        "sources_missing": missing,
        "machine": {"system": platform.platform(), "cpu": cpu_model(),
                    "cpu_count": os.cpu_count(),
                    "affinity": (sorted(os.sched_getaffinity(0))
                                 if hasattr(os, "sched_getaffinity") else None)},
        "environment": {
            "OPENBLAS_NUM_THREADS": env.get("OPENBLAS_NUM_THREADS"),
            "OMP_NUM_THREADS": env.get("OMP_NUM_THREADS"),
            "LD_LIBRARY_PATH_removed": "LD_LIBRARY_PATH" not in env,
        },
        "lock_path": str(LOCK),
        "lock_path_sha256": hashlib.sha256(str(LOCK).encode()).hexdigest(),
        "python": {"executable": sys.executable,
                   "version": platform.python_version()},
        "requested": {"kind": args.kind, "cells": args.cells,
                      "observations": args.observations,
                      "rounds": args.rounds, "warmup": args.warmup,
                      "old_run": str(args.old_run) if args.old_run else None,
                      "binary_override": args.binary},
        "time_kind": ("times_us are steady-clock service microseconds inside "
                      "the C++ benchmark; timing rows are repeated exposures "
                      "of a finite 16-window pool"),
    }


def provenance_summary(path, prov):
    return {
        "path": str(path),
        "git_head": prov["git_head"],
        "git_dirty": prov["git_dirty"],
        "binary_sha256": prov["binary"]["sha256"],
        "libraries": [dict(item) for item in prov["libraries"]],
        "sources": dict(prov.get("sources") or {}),
        "lock_path": prov["lock_path"],
    }


@contextlib.contextmanager
def benchmark_lock(required=True):
    if not required:
        yield
        return
    with LOCK.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(
                f"another benchmark owns {LOCK}; formal runs must be serial") from exc
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


# --------------------------------------------------------------------------
# case handling
# --------------------------------------------------------------------------
def case_directory(inputs, cell, selected_count):
    if (inputs / "public.json").is_file():
        if selected_count != 1:
            raise ValueError("direct --inputs case requires exactly one --cell")
        declared = read_json(inputs / "public.json").get("cell")
        if declared != cell:
            raise ValueError(f"input cell {declared!r} does not match {cell!r}")
        return inputs
    return inputs / cell


def check_input(case):
    case = Path(case)
    public_path = case / "public.json"
    if not public_path.is_file():
        raise ValueError(f"case has no public.json: {case}")
    public = read_json(public_path)
    for key in ("n", "observed_end", "target_lo", "target_hi"):
        if key not in public:
            raise ValueError(f"case public.json missing {key}: {case}")
    windows = sorted(p for p in (case / "windows").iterdir() if p.is_dir())
    if not windows:
        raise ValueError(f"no input windows in {case}")
    hashes = {}
    for window in windows:
        for name in ("raw.cf32", "clean.cf32", "sidecar.json"):
            if not (window / name).is_file():
                raise ValueError(f"incomplete source window {window}: missing {name}")
        if (window / "raw.cf32").stat().st_size != int(public["n"]) * 8:
            raise ValueError(f"raw size mismatch in {window}")
        hashes[window.name] = sha256(window / "raw.cf32")
    templates = {}
    for name in ("comm_sync.cf32", "comm_preamble.cf32", "sensing_sync.cf32"):
        path = case / name
        if not path.is_file():
            raise ValueError(f"missing public template {name} in {case}")
        templates[name] = sha256(path)
    expected = public.get("template_hashes", {})
    for name, digest in templates.items():
        if expected.get(name) != digest:
            raise ValueError(f"public template hash mismatch in {case}: {name}")
    return public, windows, hashes, templates


def prepare_cases(inputs, cells, source_count):
    plans = []
    for cell in cells:
        case = case_directory(inputs, cell, len(cells))
        if not (case / "public.json").is_file():
            raise RuntimeError(f"missing case {case}; supply existing fixtures "
                               "or generate them on archive/gnuradio-adapter")
        public, windows, hashes, templates = check_input(case)
        if public.get("cell") != cell:
            raise ValueError(f"input cell {public.get('cell')!r} does not "
                             f"match selected {cell!r}")
        if len(windows) != source_count:
            print(f"reuse {cell}: {len(windows)} existing source windows "
                  f"(requested generation size {source_count})", flush=True)
        plans.append({"cell": cell, "case": case.resolve(), "public": public,
                      "windows": windows, "hashes": hashes,
                      "templates": templates})
    return plans


def annotate_input_hashes(run, windows, hashes):
    log = Path(run) / "windows.jsonl"
    original = Path(run) / "windows.runner.jsonl"
    if original.exists():
        raise RuntimeError(f"{run} is already annotated; refusing to rewrite")
    if not log.is_file():
        raise RuntimeError(f"cannot annotate missing {log}")
    log.rename(original)
    with original.open() as source, log.open("w") as target:
        for line in source:
            row = json.loads(line)
            index = source_index(row.get("source_window"))
            if not 0 <= index < len(windows):
                raise ValueError(f"runner source_window outside pool: {index}")
            row["input_hash"] = hashes[windows[index].name]
            row["input_hash_added_offline"] = True
            target.write(json.dumps(row, separators=(",", ":"),
                                    allow_nan=False) + "\n")


def validate_run(run, expected_reconstruction, public, windows, mode="full",
                 allow_repeats=False):
    rows = load_rows(run)
    summary_path = Path(run) / "runner_summary.json"
    summary = read_json(summary_path) if summary_path.is_file() else None
    errors = []
    if summary is None:
        errors.append("runner_summary.json missing")
    else:
        if summary.get("reconstruction") != expected_reconstruction:
            errors.append("runner summary reconstruction mismatch")
        submitted = summary.get("submitted")
        completed = summary.get("completed")
        rejected = summary.get("rejected")
        if not all(type(v) is int and v >= 0 for v in (submitted, completed, rejected)):
            errors.append("runner summary count types")
        elif submitted != len(rows):
            errors.append("runner summary submitted/row mismatch")
        elif submitted != completed + rejected:
            errors.append("runner summary terminal accounting")
        if rejected:
            errors.append(f"runner rejected {rejected} observations")
    if not rows:
        errors.append("no terminal rows")
    if any(row.get("mode") != mode for row in rows):
        errors.append(f"unexpected terminal mode (wanted {mode})")
    sources = []
    for position, row in enumerate(rows):
        if row.get("n") != public["n"] or row.get("observed_end") != public["observed_end"]:
            errors.append(f"row {position}: context mismatch")
        if row.get("reconstruction") != expected_reconstruction:
            errors.append(f"row {position}: reconstruction mismatch")
        if row_rejected(row):
            errors.append(f"row {position}: rejected")
        try:
            sources.append(source_index(row.get("source_window")))
        except (ValueError, TypeError):
            errors.append(f"row {position}: source_window")
        if expected_reconstruction == "discrete_cir":
            if not isinstance(row.get("discrete_cir"), dict):
                errors.append(f"row {position}: discrete_cir object missing")
        elif isinstance(row.get("discrete_cir"), dict):
            errors.append(f"row {position}: old path unexpectedly carries discrete_cir")
    if allow_repeats:
        # Timing repeats a finite pool; a short smoke need not cover every
        # source, but every recorded source must exist in the pool.
        if not sources or not set(sources) <= set(range(len(windows))):
            errors.append("source window coverage mismatch")
    else:
        if len(sources) != len(set(sources)):
            errors.append("repeated source windows in a functional run")
        if sorted(sources) != list(range(len(windows))):
            errors.append("source window coverage mismatch")
    if errors:
        raise RuntimeError(f"invalid run {run}: " + "; ".join(errors))
    return {"rows": rows, "summary": summary}


def run_variant(binary, case, run_dir, reconstruction, env, extra, record):
    run_dir = Path(run_dir)
    if run_dir.exists():
        raise RuntimeError(f"run directory exists; refusing to overwrite: {run_dir}")
    command = [binary, "--case", case, "--out", run_dir, "--mode", "full",
               "--reconstruction", reconstruction, *extra]
    entry = dict(record)
    entry.update(argv=[str(x) for x in command], cwd=str(ROOT),
                 started_utc=utc_now(),
                 requested_reconstruction=reconstruction,
                 service_observations_are_unique_packets="--repeat" not in extra)
    run_dir.mkdir(parents=True)
    write_json(run_dir / "command.json", entry)
    rc = run_logged(command, run_dir / "runner.log", env)
    entry.update(returncode=rc, finished_utc=utc_now())
    write_json(run_dir / "command.json", entry)
    if rc:
        raise RuntimeError(f"runner failed ({rc}); see {run_dir}/runner.log")
    return rc


def resolve_old_run(old_run, cell, selected_count):
    root = Path(old_run).resolve()
    if (root / "windows.jsonl").is_file():
        if selected_count != 1:
            raise ValueError("a direct --old-run needs exactly one --cell")
        return root
    for candidate in (root / cell / "paired", root / cell / "old", root / cell):
        if (candidate / "windows.jsonl").is_file():
            return candidate
    raise RuntimeError(
        f"--old-run has no full-mode run for {cell}; tried "
        f"{root/cell/'paired'}, {root/cell/'old'}, {root/cell}")


def check_reused_old(run, case_info):
    run = Path(run)
    rows = [row for row in load_rows(run) if row.get("mode") == "full"]
    if len(rows) != len(case_info["windows"]):
        raise RuntimeError(
            f"reused old run has {len(rows)} full rows for "
            f"{len(case_info['windows'])} source windows")
    sources = [source_index(row["source_window"]) for row in rows]
    if sorted(sources) != list(range(len(case_info["windows"]))):
        raise RuntimeError("reused old run does not cover the case source windows")
    for row in rows:
        if row.get("reconstruction") != "full":
            raise RuntimeError(
                "reused old run is not --reconstruction full; archived "
                "payload_cached runs cannot serve as the old baseline")
        if row_rejected(row):
            raise RuntimeError("reused old run contains rejected rows")
        source = source_index(row["source_window"])
        expected = case_info["hashes"][f"w{source:06d}"]
        if row.get("input_hash") != expected:
            raise RuntimeError(
                "reused old run lacks a matching offline input_hash; rerun the "
                "old path so the same source windows are provable")
        output = run / f"w{int(row['window_id']):06d}_full.cf32"
        if not output.is_file():
            raise RuntimeError(f"reused old run missing dumped IQ: {output}")
        if output.stat().st_size != int(case_info["public"]["n"]) * 8:
            raise RuntimeError(f"reused old output size mismatch: {output}")
    summary_path = run / "runner_summary.json"
    summary = read_json(summary_path) if summary_path.is_file() else None
    if summary and summary.get("reconstruction") != "full":
        raise RuntimeError("reused old run summary is not the full backend")
    return {"run": str(run), "reused": True, "rows": len(rows),
            "runner_summary": summary,
            "windows_sha256": sha256(run / "windows.jsonl")}


# --------------------------------------------------------------------------
# functional comparison
# --------------------------------------------------------------------------
def comparison_payload(path, cell_key):
    if not Path(path).is_file():
        return None
    result = read_json(path)
    cells = result.get("cells") or []
    cell = next((item for item in cells if item.get("cell") == cell_key), None)
    if cell is None and len(cells) == 1:
        cell = cells[0]
    if cell is None:
        return {"path": str(path), "passed": False,
                "failures": ["comparison cell missing"]}
    return {
        "path": str(path),
        "passed": cell.get("passed") is True,
        "verification_level": result.get("verification_level"),
        "disclaimer": result.get("disclaimer"),
        "structural": cell.get("structural"),
        "counts": cell.get("counts"),
        "metrics": cell.get("metrics"),
        "discrete_model": cell.get("discrete_model"),
        "gates": cell.get("gates"),
        "exact_differences": cell.get("exact_differences"),
        "windows_exceeding_tolerance": cell.get("windows_exceeding_tolerance"),
        "successes": cell.get("successes"),
        "failures": cell.get("failures"),
    }


def run_functional(args, binary, env, out, prov):
    plans = prepare_cases(args.inputs, args.cells, args.source_windows)
    results = []
    for plan in plans:
        cell = plan["cell"]
        case_out = out / cell
        case_out.mkdir(parents=True)
        if args.old_run:
            old_dir = resolve_old_run(args.old_run, cell, len(args.cells))
            old_info = check_reused_old(old_dir, plan)
            write_json(case_out / "old_reused.json", old_info)
        else:
            old_dir = case_out / "old"
            run_variant(binary, plan["case"], old_dir, "full", env, ["--dump-iq"],
                        dict(cell=cell, role="old", case=str(plan["case"]),
                             provenance=str(out / "provenance.json")))
            old_info = validate_run(old_dir, "full", plan["public"],
                                    plan["windows"])
            annotate_input_hashes(old_dir, plan["windows"], plan["hashes"])
            old_info = {"run": str(old_dir), "reused": False,
                        "rows": len(old_info["rows"]),
                        "runner_summary": old_info["summary"]}
        new_dir = case_out / "new"
        run_variant(binary, plan["case"], new_dir, "discrete_cir", env,
                    ["--dump-iq"],
                    dict(cell=cell, role="new", case=str(plan["case"]),
                         provenance=str(out / "provenance.json")))
        new_info = validate_run(new_dir, "discrete_cir", plan["public"],
                                plan["windows"])
        annotate_input_hashes(new_dir, plan["windows"], plan["hashes"])
        new_info = {"run": str(new_dir), "reused": False,
                    "rows": len(new_info["rows"]),
                    "runner_summary": new_info["summary"]}
        comparison_path = case_out / "comparison.json"
        verify_command = [sys.executable, VERIFIER, "--case", plan["case"],
                          "--old-run", old_dir, "--new-run", new_dir,
                          "--out", comparison_path, "--require-provenance"]
        verify_rc = run_logged(verify_command, case_out / "verify.log", env)
        comparison = comparison_payload(comparison_path, cell)
        if comparison is None:
            comparison = {"path": str(comparison_path), "passed": False,
                          "failures": ["verifier produced no comparison.json"]}
        item = {
            "cell": cell,
            "case": str(plan["case"]),
            "public": {key: plan["public"][key] for key in
                       ("n", "observed_end", "target_lo", "target_hi",
                        "sensing_seed") if key in plan["public"]},
            "source_windows": len(plan["windows"]),
            "template_hashes": plan["templates"],
            "source_hashes": plan["hashes"],
            "old": old_info,
            "new": new_info,
            "verifier_rc": verify_rc,
            "comparison": comparison,
            "status": "passed" if comparison.get("passed") else "failed",
            "passed": comparison.get("passed") is True,
        }
        results.append(item)
        write_experiment(out, dict(
            schema=SCHEMA, kind="functional", complete=False, passed=None,
            provenance=provenance_summary(out / "provenance.json", prov),
            results=results, disclaimer=DISCLAIMER))
        print(f"{cell}: verifier={verify_rc} "
              f"passed={item['passed']}", flush=True)
    passed = bool(results) and all(item["passed"] for item in results)
    failures = [f"{item['cell']}: " + ", ".join(
        (item.get("comparison") or {}).get("failures") or ["unknown"])
        for item in results if not item["passed"]]
    summary = dict(
        schema=SCHEMA, kind="functional", complete=True, passed=passed,
        created_utc=utc_now(),
        provenance=provenance_summary(out / "provenance.json", prov),
        results=results, failures=failures, disclaimer=DISCLAIMER,
        gate_definition={
            "structural": "record/schema/IQ invariance checks",
            "comm": "new comm FCS and exact counts >= old",
            "sensing": "new sensing FCS and exact counts >= old",
            "target_suppression_db": "new >= old - 1.0 dB",
            "cir_recovery_db": "new >= old - 1.0 dB",
            "weak_target_abs_error_db": "new <= old + 1.0 dB",
        },
        quality_gate_note="a timing run requires this file with passed=true",
    )
    write_experiment(out, summary)
    write_functional_report(out, summary)
    return summary


def write_experiment(out, summary):
    write_json(Path(out) / "experiment_summary.json", summary)


def write_functional_report(out, summary):
    source_counts = sorted({item.get("source_windows")
                            for item in summary["results"]})
    lines = ["# discrete_cir versus full-packet: functional comparison", "",
             f"- Created: {summary.get('created_utc')}",
             f"- Git HEAD: `{summary['provenance']['git_head']}` "
             f"(dirty={summary['provenance']['git_dirty']})",
             f"- Binary SHA256: `{summary['provenance']['binary_sha256']}`",
             f"- Source windows per cell: {source_counts}",
             "", "| cell | structural | comm FCS old/new | comm exact | "
             "sensing FCS | sensing exact | target supp dB | CIR recovery dB | "
             "weak abs err dB | passed |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for item in summary["results"]:
        comp = item.get("comparison") or {}
        counts = comp.get("counts") or {}
        old_counts = counts.get("old") or {}
        new_counts = counts.get("new") or {}
        metrics = comp.get("metrics") or {}
        old_metrics = metrics.get("old") or {}
        new_metrics = metrics.get("new") or {}
        structural = (comp.get("structural") or {}).get("passed")
        lines.append(
            "| {cell} | {structural} | {cfcs} | {cexact} | {sfcs} | {sexact} | "
            "{supp} | {cir} | {weak} | {passed} |".format(
                cell=item["cell"],
                structural="pass" if structural else "fail",
                cfcs=f"{old_counts.get('comm_fcs', 'n/a')}/"
                     f"{new_counts.get('comm_fcs', 'n/a')}",
                cexact=f"{old_counts.get('comm_exact', 'n/a')}/"
                       f"{new_counts.get('comm_exact', 'n/a')}",
                sfcs=f"{old_counts.get('sensing_fcs', 'n/a')}/"
                     f"{new_counts.get('sensing_fcs', 'n/a')}",
                sexact=f"{old_counts.get('sensing_exact', 'n/a')}/"
                       f"{new_counts.get('sensing_exact', 'n/a')}",
                supp=_triple(old_metrics.get("target_suppression_db"),
                             new_metrics.get("target_suppression_db")),
                cir=_triple(old_metrics.get("cir_recovery_db"),
                            new_metrics.get("cir_recovery_db")),
                weak=_triple(old_metrics.get("weak_target_abs_error_db"),
                             new_metrics.get("weak_target_abs_error_db")),
                passed="PASS" if item["passed"] else "FAIL"))
    lines += ["", "## Failures", ""]
    if summary["failures"]:
        lines.extend(f"- {failure}" for failure in summary["failures"])
    else:
        lines.append("- none")
    lines += ["", "## Windows outside the 1 dB tolerance", ""]
    reported = False
    for item in summary["results"]:
        entries = (item.get("comparison") or {}).get(
            "windows_exceeding_tolerance") or []
        for entry in entries:
            reported = True
            lines.append(
                f"- {item['cell']} source w{int(entry['source_window']):06d}: "
                f"{', '.join(entry['metrics'])} delta={entry['delta']}")
    if not reported:
        lines.append("- none (per-cell aggregate gate; see comparison.json for "
                     "per-window values)")
    lines += ["", "## Scope", "",
              f"{DISCLAIMER}",
              "The old and new paths estimate their model independently; "
              "model_id, q and CFO are not required to match.",
              "MATLAB, native, FPGA/hardware and formal real-time throughput "
              "were not run."]
    (Path(out) / "comparison_report.md").write_text("\n".join(lines) + "\n")


def _triple(old, new):
    def cell(value):
        return "n/a" if value is None else f"{value:.2f}"
    delta = ""
    if old is not None and new is not None:
        delta = f" ({new - old:+.2f})"
    return f"{cell(old)}/{cell(new)}{delta}"


# --------------------------------------------------------------------------
# negative pool
# --------------------------------------------------------------------------
def run_negative(args, binary, env, out, prov):
    results, skipped = [], []
    # Negative cases are never generated here: use an existing pool or skip.
    for cell in args.cells:
        case = case_directory(args.inputs, cell, len(args.cells))
        if not (case / "public.json").is_file():
            skipped.append({"cell": cell, "case": str(case.resolve()),
                            "reason": f"negative case pool absent: {case}"})
            continue
        public, windows, hashes, templates = check_input(case)
        if public.get("cell") != cell:
            raise ValueError(f"input cell {public.get('cell')!r} does not "
                             f"match selected {cell!r}")
        plan = {"cell": cell, "case": case.resolve(), "public": public,
                "windows": windows, "hashes": hashes,
                "templates": templates}
        results.append(_negative_case(plan, binary, env, out, prov))
    if not results:
        status = "skipped"
    elif any(not item["passed"] for item in results):
        status = "failed"
    else:
        status = "partial" if skipped else "passed"
    passed = status in ("passed", "partial", "skipped")
    summary = dict(
        schema=SCHEMA, kind="negative", complete=True, passed=passed,
        status=status, created_utc=utc_now(),
        provenance=provenance_summary(out / "provenance.json", prov),
        results=results, skipped=skipped,
        gate_definition={
            "no_interference": "committed must be false and the whole output must be byte-identical to raw.cf32",
            "no_overlap": "the public target SYNC region must be byte-identical to raw.cf32; cancellation outside that region is allowed and a commit is not itself a failure",
        },
        notes=["negative cells are never generated by this driver; point "
               "--inputs at an existing pool or the cell is explicitly skipped"],
        disclaimer=DISCLAIMER)
    write_experiment(out, summary)
    write_negative_report(out, summary)
    if status == "skipped":
        print("negative skipped: " + "; ".join(
            item["reason"] for item in skipped), flush=True)
    return summary


def _negative_case(plan, binary, env, out, prov):
    cell = plan["cell"]
    case_out = out / cell
    case_out.mkdir(parents=True)
    run_dir = case_out / "new"
    run_variant(binary, plan["case"], run_dir, "discrete_cir", env, ["--dump-iq"],
                dict(cell=cell, role="negative", case=str(plan["case"]),
                     provenance=str(out / "provenance.json")))
    validated = validate_run(run_dir, "discrete_cir", plan["public"],
                             plan["windows"])
    annotate_input_hashes(run_dir, plan["windows"], plan["hashes"])
    failures = []
    committed_rows = 0
    identical_rows = 0
    target_only = cell == "no_overlap"
    target_lo = int(plan["public"].get("target_lo", 0))
    target_hi = int(plan["public"].get("target_hi", plan["public"].get("n", 0)))
    for row in validated["rows"]:
        source = source_index(row["source_window"])
        raw = plan["windows"][source] / "raw.cf32"
        output = run_dir / f"w{int(row['window_id']):06d}_full.cf32"
        if not output.is_file():
            failures.append(f"window {row['window_id']}: dumped output missing")
            continue
        if output.stat().st_size != raw.stat().st_size:
            failures.append(f"window {row['window_id']}: output size mismatch")
            continue
        committed = row.get("committed") is True
        if committed:
            committed_rows += 1
            if not target_only:
                failures.append(f"window {row['window_id']}: committed={committed}")
            else:
                sub = (row.get("discrete_cir") or {}).get("sub")
                if (isinstance(sub, list) and len(sub) == 2 and
                        int(sub[0]) < target_hi and int(sub[1]) > target_lo):
                    failures.append(f"window {row['window_id']}: cancellation overlaps the target region")
        raw_bytes = raw.read_bytes()
        out_bytes = output.read_bytes()
        if target_only:
            lo_b = target_lo * 8
            hi_b = target_hi * 8
            if out_bytes[lo_b:hi_b] != raw_bytes[lo_b:hi_b]:
                failures.append(f"window {row['window_id']}: target SYNC region differs from raw")
            else:
                identical_rows += 1
        else:
            if out_bytes != raw_bytes:
                failures.append(f"window {row['window_id']}: output differs from raw")
            else:
                identical_rows += 1
    item = {
        "cell": cell, "case": str(plan["case"]),
        "run": str(run_dir), "rows": len(validated["rows"]),
        "committed_rows": committed_rows,
        "byte_identical_rows": identical_rows,
        "target_only": target_only,
        "target_bounds": [target_lo, target_hi] if target_only else None,
        "reasons": sorted({row.get("reason", "missing")
                           for row in validated["rows"]}),
        "model_valid_rows": sum(row.get("model_valid") is True
                                for row in validated["rows"]),
        "failures": failures, "passed": not failures,
    }
    print(f"negative {cell}: rows={item['rows']} "
          f"committed={committed_rows} identical={identical_rows} "
          f"passed={item['passed']}", flush=True)
    return item


def write_negative_report(out, summary):
    lines = ["# discrete_cir negative-pool check", "",
             f"- Status: {summary['status']}",
             f"- Git HEAD: `{summary['provenance']['git_head']}`",
             f"- Binary SHA256: `{summary['provenance']['binary_sha256']}`",
             "", "| cell | rows | committed | byte-identical | model valid | "
             "passed |", "|---|---|---|---|---|---|"]
    for item in summary["results"]:
        lines.append(f"| {item['cell']} | {item['rows']} | "
                     f"{item['committed_rows']} | {item['byte_identical_rows']} | "
                     f"{item['model_valid_rows']} | "
                     f"{'PASS' if item['passed'] else 'FAIL'} |")
    lines += ["", "## Skipped", ""]
    if summary["skipped"]:
        lines.extend(f"- {item['cell']}: {item['reason']}"
                     for item in summary["skipped"])
    else:
        lines.append("- none")
    target_only = [item["cell"] for item in summary["results"]
                   if item.get("target_only")]
    if target_only:
        lines += ["", f"- {', '.join(target_only)}: the identical column is "
                  "checked over the public target SYNC region only; a commit "
                  "that cancels interference outside that region is allowed."]
    lines += ["", f"{DISCLAIMER}"]
    (Path(out) / "negative_report.md").write_text("\n".join(lines) + "\n")


# --------------------------------------------------------------------------
# timing
# --------------------------------------------------------------------------
def check_quality_summary(path, gate_cells, binary_sha, prov, library_hashes):
    path = Path(path)
    if not path.is_file():
        raise RuntimeError(f"timing requires a passed functional summary: {path}")
    data = read_json(path)
    if data.get("schema") != SCHEMA or data.get("kind") != "functional":
        raise RuntimeError("--quality-summary is not a functional summary")
    if data.get("complete") is not True or data.get("passed") is not True:
        raise RuntimeError("--quality-summary did not pass")
    recorded = data.get("provenance") or {}
    if recorded.get("binary_sha256") != binary_sha:
        raise RuntimeError(
            "quality summary was produced with a different binary; rerun the "
            "functional comparison for this build")
    # The binary hash is the primary binding.  Accept a later commit only when
    # the measurement-relevant C++/generator sources are byte-identical; tool
    # script edits do not change the measured binary.
    def measurement_sources(sources):
        return {name: digest for name, digest in (sources or {}).items()
                if name.startswith("gr-uwb/")}
    same_head = recorded.get("git_head") == prov["git_head"]
    same_sources = (measurement_sources(recorded.get("sources")) ==
                    measurement_sources(prov.get("sources")))
    if not (same_head or same_sources):
        raise RuntimeError(
            "quality summary commit and C++ source hashes do not match the "
            "current tree; rerun the functional comparison for this build")
    recorded_libraries = sorted(item.get("sha256")
                                for item in recorded.get("libraries") or [])
    if recorded_libraries != sorted(library_hashes):
        raise RuntimeError("quality summary loaded a different uwb library")
    by_cell = {item.get("cell"): item for item in data.get("results") or []}
    missing = []
    for cell in gate_cells:
        item = by_cell.get(cell)
        if not item or item.get("passed") is not True:
            missing.append(cell)
            continue
        comparison = item.get("comparison") or {}
        if comparison.get("passed") is not True:
            missing.append(cell)
    if missing:
        raise RuntimeError(
            "quality summary has no passed functional comparison for: "
            + ",".join(missing))
    return {"path": str(path.resolve()), "sha256": sha256(path),
            "git_head": recorded.get("git_head"),
            "binary_sha256": recorded.get("binary_sha256"),
            "head_match": bool(same_head),
            "sources_match": bool(same_sources),
            "cells": list(gate_cells),
            "source_windows": {cell: by_cell[cell].get("source_windows")
                               for cell in gate_cells},
            "source_hashes": {cell: by_cell[cell].get("source_hashes")
                              for cell in gate_cells}}


def timing_distribution(rows):
    if any(row_rejected(row) for row in rows):
        raise RuntimeError("timing run contains rejected rows")
    values = []
    for row in rows:
        value = (row.get("times_us") or {}).get("sic_total")
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise RuntimeError("timing row has no finite nonnegative sic_total")
        values.append(float(value))
    if not values:
        raise RuntimeError("timing run has no accepted rows")
    array = np.asarray(values, dtype=float)
    return {"n": int(array.size), "mean": float(array.mean()),
            "p50": float(np.percentile(array, 50)),
            "p95": float(np.percentile(array, 95)),
            "p99": float(np.percentile(array, 99)),
            "min": float(array.min()), "max": float(array.max())}


def timing_gate(old, new):
    mean_ratio = (new["mean"] / old["mean"]) if old["mean"] > 0 else None
    mean_passed = (new["mean"] <= TIMING_GATE["mean_ratio_max"] * old["mean"])
    p95_passed = new["p95"] <= old["p95"]
    return {"mean_ratio": mean_ratio,
            "mean_limit_us": TIMING_GATE["mean_ratio_max"] * old["mean"],
            "mean_passed": bool(mean_passed),
            "p95_passed": bool(p95_passed),
            "passed": bool(mean_passed and p95_passed)}


def run_timing(args, binary, env, out, prov):
    lib_hashes = [item["sha256"] for item in prov["libraries"]]
    quality = check_quality_summary(args.quality_summary, args.cells,
                                    prov["binary"]["sha256"], prov, lib_hashes)
    plans = prepare_cases(args.inputs, args.cells, args.source_windows)
    for plan in plans:
        expected = quality["source_windows"].get(plan["cell"])
        if expected != len(plan["windows"]):
            raise RuntimeError(
                f"timing case {plan['cell']} has {len(plan['windows'])} source "
                f"windows but the quality summary used {expected}")
        recorded_hashes = quality["source_hashes"].get(plan["cell"])
        if recorded_hashes != plan["hashes"]:
            raise RuntimeError(
                f"timing case {plan['cell']} source IQ differs from the quality "
                "run; regenerate or rerun the functional comparison")
    cells, failures = [], []
    for plan in plans:
        cell = plan["cell"]
        rounds, cell_failed = [], False
        for number in range(1, args.rounds + 1):
            if number % 2:
                order = [("old", "full"), ("new", "discrete_cir")]
            else:
                order = [("new", "discrete_cir"), ("old", "full")]
            dirs, stats = {}, {}
            for variant, reconstruction in order:
                run_dir = out / "timing" / cell / f"r{number}_{variant}"
                run_variant(binary, plan["case"], run_dir, reconstruction, env,
                            ["--repeat", str(args.observations),
                             "--warmup", str(args.warmup)],
                            dict(cell=cell, round=number, role=variant,
                                 case=str(plan["case"]),
                                 provenance=str(out / "provenance.json")))
                validated = validate_run(run_dir, reconstruction, plan["public"],
                                         plan["windows"], allow_repeats=True)
                if len(validated["rows"]) != args.observations:
                    raise RuntimeError(
                        f"{run_dir}: expected {args.observations} rows, got "
                        f"{len(validated['rows'])}")
                annotate_input_hashes(run_dir, plan["windows"], plan["hashes"])
                stats[variant] = timing_distribution(validated["rows"])
                dirs[variant] = str(run_dir)
            structural_path = out / "timing" / cell / f"r{number}_structural.json"
            verify_command = [sys.executable, VERIFIER, "--case", plan["case"],
                              "--old-run", dirs["old"], "--new-run", dirs["new"],
                              "--out", structural_path, "--summary-only",
                              "--require-provenance"]
            verify_rc = run_logged(verify_command,
                                   out / "timing" / cell / f"r{number}_verify.log",
                                   env)
            structural = read_json(structural_path) if structural_path.is_file() else None
            structural_passed = bool(structural and structural.get("passed"))
            gate = timing_gate(stats["old"], stats["new"])
            round_passed = structural_passed and gate["passed"]
            if not round_passed:
                cell_failed = True
            rounds.append({"round": number,
                           "order": [variant for variant, _ in order],
                           "source_windows": len(plan["windows"]),
                           "old": dict(stats["old"], run=dirs["old"]),
                           "new": dict(stats["new"], run=dirs["new"]),
                           "gate": gate,
                           "passed": round_passed,
                           "structural": {"passed": structural_passed,
                                          "verifier_rc": verify_rc,
                                          "path": str(structural_path)}})
            ratio = ("n/a" if gate["mean_ratio"] is None
                     else f"{gate['mean_ratio']:.3f}")
            print(f"timing {cell} round {number}: "
                  f"old mean={stats['old']['mean']:.1f}us "
                  f"new mean={stats['new']['mean']:.1f}us "
                  f"ratio={ratio} "
                  f"p95 old={stats['old']['p95']:.1f} "
                  f"new={stats['new']['p95']:.1f} "
                  f"passed={gate['passed']}", flush=True)
        pooled = {}
        for variant in ("old", "new"):
            rows = []
            for number in range(1, args.rounds + 1):
                rows.extend(load_rows(out / "timing" / cell /
                                      f"r{number}_{variant}"))
            pooled[variant] = timing_distribution(rows)
        overall = {"passed": not cell_failed,
                   "rounds_passed": sum(item["passed"] for item in rounds),
                   "rounds": len(rounds)}
        if cell_failed:
            failures.append({"cell": cell, "rounds": [
                {"round": item["round"],
                 "structural_passed": item["structural"]["passed"],
                 "mean_passed": item["gate"]["mean_passed"],
                 "p95_passed": item["gate"]["p95_passed"]}
                for item in rounds if not item["passed"]]})
        cells.append({"cell": cell, "case": str(plan["case"]),
                      "rounds": rounds, "pooled": pooled,
                      "gate": overall, "passed": not cell_failed})
    passed = bool(cells) and not failures
    summary = dict(
        schema=SCHEMA, kind="timing", complete=True, passed=passed,
        created_utc=utc_now(),
        provenance=provenance_summary(out / "provenance.json", prov),
        quality_gate=quality,
        lock_path=str(LOCK),
        gate_definition=TIMING_GATE,
        observations_per_variant_per_round=args.observations,
        warmup_per_variant_per_round=args.warmup,
        rounds=args.rounds,
        cells=cells, failures=failures,
        notes=[
            "rounds alternate AB then BA and were run sequentially under the "
            "exclusive benchmark lock",
            "each observation reuses a finite 16-window raw IQ pool; the 128 "
            "observations are not 128 independent radio packets",
            "pooled statistics are descriptive only; the gate is applied per "
            "round and every round must pass",
            "P99 from 128 samples is descriptive and not a tail guarantee",
        ],
        disclaimer=DISCLAIMER)
    write_experiment(out, summary)
    write_timing_csv(out, summary)
    write_timing_report(out, summary)
    return summary


def write_timing_csv(out, summary):
    path = Path(out) / "timing.csv"
    fieldnames = ["cell", "record_type", "round", "variant", "n", "mean_us",
                  "p50_us", "p95_us", "p99_us", "min_us", "max_us",
                  "mean_ratio", "mean_passed", "p95_passed", "passed"]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for cell in summary["cells"]:
            for item in cell["rounds"]:
                for variant in ("old", "new"):
                    stats = item[variant]
                    writer.writerow({
                        "cell": cell["cell"], "record_type": "run",
                        "round": item["round"], "variant": variant,
                        "n": stats["n"], "mean_us": f"{stats['mean']:.6f}",
                        "p50_us": f"{stats['p50']:.6f}",
                        "p95_us": f"{stats['p95']:.6f}",
                        "p99_us": f"{stats['p99']:.6f}",
                        "min_us": f"{stats['min']:.6f}",
                        "max_us": f"{stats['max']:.6f}",
                        "mean_ratio": "", "mean_passed": "",
                        "p95_passed": "", "passed": ""})
                gate = item["gate"]
                writer.writerow({
                    "cell": cell["cell"], "record_type": "gate",
                    "round": item["round"], "variant": "new/old", "n": "",
                    "mean_us": "", "p50_us": "", "p95_us": "",
                    "p99_us": "", "min_us": "", "max_us": "",
                    "mean_ratio": ("" if gate["mean_ratio"] is None
                                   else f"{gate['mean_ratio']:.6f}"),
                    "mean_passed": gate["mean_passed"],
                    "p95_passed": gate["p95_passed"],
                    "passed": gate["passed"]})
            for variant in ("old", "new"):
                stats = cell["pooled"][variant]
                writer.writerow({
                    "cell": cell["cell"], "record_type": "pooled",
                    "round": "all", "variant": variant, "n": stats["n"],
                    "mean_us": f"{stats['mean']:.6f}",
                    "p50_us": f"{stats['p50']:.6f}",
                    "p95_us": f"{stats['p95']:.6f}",
                    "p99_us": f"{stats['p99']:.6f}",
                    "min_us": f"{stats['min']:.6f}",
                    "max_us": f"{stats['max']:.6f}",
                    "mean_ratio": "", "mean_passed": "",
                    "p95_passed": "", "passed": ""})


def write_timing_report(out, summary):
    lines = ["# discrete_cir versus full-packet: paired timing", "",
             f"- Created: {summary.get('created_utc')}",
             f"- Git HEAD: `{summary['provenance']['git_head']}`",
             f"- Binary SHA256: `{summary['provenance']['binary_sha256']}`",
             f"- Lock: `{summary['lock_path']}`",
             f"- Gate: {TIMING_GATE['rule']}",
             f"- Quality summary: `{summary['quality_gate']['path']}` "
             f"(sha256 `{summary['quality_gate']['sha256']}`)",
             "", "| cell | round | order | old mean | new mean | ratio | "
             "old P95 | new P95 | mean gate | P95 gate | passed |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for cell in summary["cells"]:
        for item in cell["rounds"]:
            gate = item["gate"]
            ratio = ("n/a" if gate["mean_ratio"] is None
                     else f"{gate['mean_ratio']:.3f}")
            lines.append(
                f"| {cell['cell']} | {item['round']} | "
                f"{'/'.join(item['order'])} | "
                f"{item['old']['mean']:.1f} | {item['new']['mean']:.1f} | "
                f"{ratio} | "
                f"{item['old']['p95']:.1f} | {item['new']['p95']:.1f} | "
                f"{'pass' if gate['mean_passed'] else 'fail'} | "
                f"{'pass' if gate['p95_passed'] else 'fail'} | "
                f"{'PASS' if gate['passed'] else 'FAIL'} |")
    lines += ["", "## Failures", ""]
    if summary["failures"]:
        for failure in summary["failures"]:
            rounds = ", ".join(
                f"round {item['round']} (structural="
                f"{'pass' if item['structural_passed'] else 'fail'}, mean="
                f"{'pass' if item['mean_passed'] else 'fail'}, p95="
                f"{'pass' if item['p95_passed'] else 'fail'})"
                for item in failure["rounds"])
            lines.append(f"- {failure['cell']}: {rounds}")
    else:
        lines.append("- none")
    lines += ["", "## Pooled (descriptive, not the gate)", ""]
    for cell in summary["cells"]:
        lines.append(f"- {cell['cell']}: old mean "
                     f"{cell['pooled']['old']['mean']:.1f} us, new mean "
                     f"{cell['pooled']['new']['mean']:.1f} us, old P99 "
                     f"{cell['pooled']['old']['p99']:.1f} us, new P99 "
                     f"{cell['pooled']['new']['p99']:.1f} us")
    pool = sorted({item["source_windows"] for cell in summary["cells"]
                   for item in cell["rounds"]})
    lines += ["", "## Scope", "", DISCLAIMER,
              f"The {summary['observations_per_variant_per_round']} "
              f"observations per variant per round reuse a finite {pool} "
              "source-window pool and are not independent radio packets.",
              "Small-sample P99 is descriptive; mean and P95 carry the gate."]
    (Path(out) / "timing_report.md").write_text("\n".join(lines) + "\n")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--inputs", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--kind", required=True, choices=DEFAULT_CELLS)
    parser.add_argument("--cell", action="append",
                        help="repeat to select multiple cells")
    parser.add_argument("--windows", type=int,
                        help="source windows when generating a missing case")
    parser.add_argument("--seed", type=int, default=2000000)
    parser.add_argument("--observations", type=int, default=128,
                        help="timing observations per variant per round")
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--warmup", type=int, default=16)
    parser.add_argument("--old-run", type=Path,
                        help="reuse an archived old-path functional run")
    parser.add_argument("--binary", help="binary name inside <build>/apps")
    parser.add_argument("--quality-summary", type=Path,
                        help="passed functional experiment_summary.json "
                             "(required for --kind timing)")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.observations < 1 or args.rounds < 1 or args.warmup < 0:
        parser.error("invalid observation/round/warmup budget")
    if args.windows is not None and args.windows < 1:
        parser.error("--windows must be positive")
    if args.seed < 0:
        parser.error("--seed must be nonnegative")
    args.cells = args.cell or DEFAULT_CELLS[args.kind]
    if len(args.cells) != len(set(args.cells)):
        parser.error("duplicate --cell")
    if args.old_run and args.kind != "functional":
        parser.error("--old-run is only valid for --kind functional")
    if args.kind == "timing" and not args.quality_summary:
        parser.error("--kind timing requires --quality-summary "
                     "(quality precedes performance)")
    args.source_windows = args.windows if args.windows is not None else 16
    args.build_dir = args.build_dir.resolve()
    args.inputs = args.inputs.resolve()
    args.out = args.out.resolve()
    out = args.out
    if out.exists() and any(out.iterdir()):
        parser.error(f"output is not empty; choose a new run: {out}")
    binary = select_binary(args.build_dir, args.binary)
    if not binary_contains(binary, "discrete_cir"):
        raise RuntimeError(
            f"{binary} has no 'discrete_cir' marker; the benchmark must "
            "implement --reconstruction discrete_cir before this driver can "
            "run (select another --binary only if that binary has it)")
    env = clean_env()
    out.mkdir(parents=True, exist_ok=True)
    prov = provenance(binary, args.build_dir, env, args)
    write_json(out / "provenance.json", prov)
    write_json(out / "experiment_plan.json", dict(
        schema="sic-discrete-cir-experiment-plan/1",
        kind=args.kind, cells=args.cells,
        source_windows_requested=args.source_windows,
        observations_per_variant_per_round=args.observations,
        rounds=args.rounds, warmup_per_variant_per_round=args.warmup,
        old_run=str(args.old_run.resolve()) if args.old_run else None,
        quality_summary=(str(args.quality_summary.resolve())
                         if args.quality_summary else None),
        build_dir=str(args.build_dir), out=str(out),
        lock_path=str(LOCK), formal_cartesian_matrix=False,
        lock_scope="functional, negative and timing runs are serialized",
        notes=["timing runs are only allowed after a passed functional "
               "summary for the same binary/commit/library",
               "old and new rows are paired on identical source windows; "
               "model_id/q are not required to match"]))
    write_experiment(out, dict(
        schema=SCHEMA, kind=args.kind, complete=False, passed=None,
        provenance=provenance_summary(out / "provenance.json", prov),
        results=[], disclaimer=DISCLAIMER))
    summary = None
    try:
        with benchmark_lock():
            if args.kind == "functional":
                summary = run_functional(args, binary, env, out, prov)
            elif args.kind == "negative":
                summary = run_negative(args, binary, env, out, prov)
            else:
                summary = run_timing(args, binary, env, out, prov)
    except BaseException as exc:
        partial = {}
        try:
            partial = read_json(out / "experiment_summary.json")
        except (OSError, ValueError):
            pass
        partial.update(complete=False, passed=False,
                       error=f"{type(exc).__name__}: {exc}")
        write_experiment(out, partial)
        raise
    passed = summary is not None and summary.get("passed") is True
    print(f"{args.kind}: {'PASS' if passed else 'FAIL'} -> "
          f"{out}/experiment_summary.json", flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"experiment failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
