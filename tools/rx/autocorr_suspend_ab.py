#!/usr/bin/env python3
"""Serial ABBA paired comparison of `--autocorr-when-locked scan|suspend`.

Runs the native replay binary over one fixture, alternating the two arms in
A/B/B/A order (one ABBA block per `ceil(rounds/2)` pairs), parses the
START / SUMMARY* / LOCK_STATS / COST_TOTAL / FINAL lines, derives the target
and frontend metrics, and writes one JSON document with full provenance.

Only the replay path is a required input; every other live argument that is not
one of the fixed base arguments can be passed through with `--extra` (repeatable)
or after a bare `--`.

Example:
    python3 tools/rx/autocorr_suspend_ab.py \
        --replay /tmp/opencode/uwb-live-1ms/dw3000_1ms_1s.sc16 \
        --period-ms 1 --workers 2 --queue 24 --rounds 4 \
        --out docs/performance/data_autocorr_suspend_20261120/abba.json
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import os
import platform
import re
import shlex
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _common.paths import find_app  # noqa: E402

SCHEMA = "uwb-autocorr-suspend-abba/1"

# Phase 3: prefer the configured preset build.  The old spelling stays as the
# last-resort error target.
# TODO(openuwb-compat-removal): require the official binary.
DEFAULT_BINARY = str(find_app("openuwb_rx", required=False) or "openuwb_rx")
DEFAULT_OUT = "docs/performance/data_autocorr_suspend_20261120/abba.json"
DEFAULT_ROUNDS = 4
DEFAULT_WORKERS = 2
DEFAULT_QUEUE = 24
DEFAULT_TIMEOUT = 1800.0

# Fixed base arguments required by the validation plan.  `suspend` requires
# --interference off (the binary fails closed otherwise).
BASE_ARGS = [
    "--sync", "locked",
    "--interference", "off",
    "--fused", "on",
    "--cir", "full",
    "--taps", "eq16",
    "--print", "summary",
    "--summary", "100000",
]

# Arguments the tool owns; a passthrough occurrence is a warning, not an error.
OWNED_KEYS = {
    "--replay", "--sync", "--interference", "--fused", "--cir", "--taps",
    "--print", "--summary", "--workers", "--queue", "--seconds",
    "--autocorr-when-locked",
}

# (metric name, source block, key inside that block)
METRIC_SOURCES: List[Tuple[str, str, str]] = [
    # ---- target path (LOCK_STATS: target-lock scheduler counters) ----
    ("fcs_ok", "lock_stats", "fcs_ok"),
    ("not_detected", "lock_stats", "not_detected"),
    ("fcs_failed", "lock_stats", "fcs_failed"),
    ("identity_rejects", "lock_stats", "identity_rejects"),
    ("lock_entries", "lock_stats", "lock_entries"),
    ("holdover_entries", "lock_stats", "holdover_entries"),
    ("reacquire_entries", "lock_stats", "reacquire_entries"),
    ("expected_slots", "lock_stats", "expected_slots"),
    ("submitted", "lock_stats", "submitted"),
    ("dropped_queue", "lock_stats", "dropped_queue"),
    ("expired", "lock_stats", "expired"),
    ("candidates", "lock_stats", "candidates"),
    ("cand_target", "lock_stats", "cand_target"),
    ("cand_unknown", "lock_stats", "cand_unknown"),
    ("autocorr_ignored", "lock_stats", "autocorr_ignored"),
    ("decode_failed", "lock_stats", "decode_failed"),
    ("out_of_grid", "lock_stats", "out_of_grid"),
    ("stale", "lock_stats", "stale"),
    ("duplicate", "lock_stats", "duplicate"),
    ("discontinuities", "lock_stats", "discontinuities"),
    ("cand_submit_attempts", "lock_stats", "cand_submit_attempts"),
    ("cand_admitted", "lock_stats", "cand_admitted"),
    ("cand_dropped_queue", "lock_stats", "cand_dropped_queue"),
    ("cand_incomplete", "lock_stats", "cand_incomplete"),
    # ---- new gate counters (absent on a pre-change binary -> null) ----
    ("autocorr_suspend_events", "lock_stats", "autocorr_suspend_events"),
    ("autocorr_resume_events", "lock_stats", "autocorr_resume_events"),
    ("autocorr_suspended_samples", "lock_stats", "autocorr_suspended_samples"),
    ("autocorr_resume_backlog_samples", "lock_stats",
     "autocorr_resume_backlog_samples"),
    # ---- frontend (COST_TOTAL) ----
    ("frontend_scan_us", "cost_total", "frontend_scan_us"),
    ("frontend_us", "cost_total", "frontend_us"),
    ("frontend_copy_us", "cost_total", "frontend_copy_us"),
    ("acq_seeds", "cost_total", "acq_seeds"),
    ("frontend_samples", "cost_total", "frontend_samples"),
    ("reader_us", "cost_total", "reader_us"),
    # ---- overall demod drain (FINAL) ----
    ("final_completed", "final", "completed"),
    ("final_fcs_ok", "final", "fcs_ok"),
    ("final_demod_completed", "final", "demod_completed"),
    ("final_raw_drop_samples", "final", "raw_drop_samples"),
    ("final_gaps", "final", "gaps"),
]

DERIVED_METRICS = [
    "frontend_scan_fraction",
    "scan_us_per_sample",
    "target_fcs_pct",
]

# Metrics whose per-round scan/suspend difference must be explained.
TARGET_METRICS = [
    "fcs_ok", "not_detected", "fcs_failed", "identity_rejects",
    "lock_entries", "holdover_entries", "reacquire_entries",
]

# SUMMARY fail-counter aliases appended as bare KEY=VALUE at the end of a
# batch line, mapped onto the names asked for by the plan.
SUMMARY_FAIL_ALIASES = {
    "TIMING_FAILED": "timing_failed",
    "SFD_FAILED": "sfd_failed",
    "PAYLOAD_FAILED": "payload_failed",
    "CRC_CHECK_FAILED": "payload_failed",
    "DROPPED_QUEUE": "dropped_queue_failures",
    "INCOMPLETE_WINDOW": "incomplete_window",
    "INVALID_GAP": "invalid_gap",
}

KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(\S+)")


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------
def die(msg: str, code: int = 2) -> None:
    print("error: " + msg, file=sys.stderr)
    raise SystemExit(code)


def utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds")


def iso_from_epoch(epoch: float) -> str:
    return _dt.datetime.fromtimestamp(epoch, _dt.timezone.utc).isoformat(
        timespec="milliseconds")


def sha256_file(path: str, chunk: int = 8 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def coerce(tok: str) -> Any:
    """Numeric coercion that never invents values: keeps 'a/b' as text."""
    if "/" in tok:
        return tok
    try:
        return int(tok, 10)
    except ValueError:
        pass
    try:
        return float(tok)
    except ValueError:
        return tok


def parse_kv(line: str) -> Dict[str, Any]:
    return {k: coerce(v) for k, v in KV_RE.findall(line)}


def parse_ratio(tok: Any) -> Tuple[Optional[int], Optional[int]]:
    if isinstance(tok, str) and "/" in tok:
        a, b = tok.split("/", 1)
        try:
            return int(a), int(b)
        except ValueError:
            return None, None
    return None, None


def num(v: Any) -> Optional[float]:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def fmt(v: Any, digits: int = 3) -> str:
    if v is None:
        return "null"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def cell(v: Any, digits: int = 3) -> str:
    return "-" if v is None else fmt(v, digits)


# display precision per metric (deltas of per-sample costs are tiny)
METRIC_DIGITS = {
    "scan_us_per_sample": 6,
    "frontend_scan_fraction": 4,
}


def median(vals: Sequence[float]) -> Optional[float]:
    return float(statistics.median(vals)) if vals else None


def mean(vals: Sequence[float]) -> Optional[float]:
    return float(statistics.fmean(vals)) if vals else None


def r6(v: Optional[float]) -> Optional[float]:
    """Trim float-subtraction noise so the JSON stays readable."""
    return None if v is None else round(v, 6)


# --------------------------------------------------------------------------
# line parsing
# --------------------------------------------------------------------------
def parse_run_output(stdout: str, warnings: List[str], tag: str) -> Dict[str, Any]:
    """Split the console log into the blocks we care about.

    Every block is the LAST occurrence of its prefix: START/FINAL/LOCK_STATS/
    COST_TOTAL are printed once; SUMMARY lines repeat per batch, so all of them
    are kept in order.
    """
    blocks: Dict[str, Dict[str, Any]] = {
        "start": {}, "lock_stats": {}, "cost_total": {}, "final": {},
    }
    summaries: List[Dict[str, Any]] = []
    lock_events: Dict[str, int] = {}
    counts = {"START": 0, "LOCK_STATS": 0, "COST_TOTAL": 0, "FINAL": 0,
              "SUMMARY": 0, "SUMMARY_PARTIAL": 0, "LOCK": 0}

    for raw in stdout.splitlines():
        line = raw.strip()
        if not line:
            continue
        head, _, rest = line.partition(" ")
        if head == "START":
            blocks["start"] = parse_kv(line)
            counts["START"] += 1
        elif head == "LOCK_STATS":
            blocks["lock_stats"] = parse_kv(line)
            counts["LOCK_STATS"] += 1
        elif head == "COST_TOTAL":
            blocks["cost_total"] = parse_kv(line)
            counts["COST_TOTAL"] += 1
        elif head == "FINAL":
            blocks["final"] = parse_kv(line)
            counts["FINAL"] += 1
        elif head in ("SUMMARY", "SUMMARY_PARTIAL"):
            d = parse_kv(line)
            d["_kind"] = head
            summaries.append(d)
            counts[head] += 1
        elif head == "LOCK":
            m = re.search(r"\bstate=(\S+)", line)
            if m:
                lock_events[m.group(1)] = lock_events.get(m.group(1), 0) + 1
            counts["LOCK"] += 1

    for name, n in counts.items():
        if name in ("START", "LOCK_STATS", "COST_TOTAL", "FINAL") and n == 0:
            warnings.append(f"{tag}: no {name} line in output")
        if n > 1 and name in ("START", "LOCK_STATS", "COST_TOTAL", "FINAL"):
            warnings.append(f"{tag}: {name} printed {n} times, using the last")

    fail_totals: Dict[str, int] = {}
    for s in summaries:
        for k, v in s.items():
            if k in SUMMARY_FAIL_ALIASES and isinstance(v, int):
                alias = SUMMARY_FAIL_ALIASES[k]
                fail_totals[alias] = fail_totals.get(alias, 0) + v

    return {
        "start": blocks["start"],
        "lock_stats": blocks["lock_stats"],
        "cost_total": blocks["cost_total"],
        "final": blocks["final"],
        "summary_lines": summaries,
        "summary_failures": fail_totals,
        "lock_events": lock_events,
        "line_counts": counts,
    }


def extract_metrics(parsed: Dict[str, Any], warnings: List[str],
                    tag: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    missing: List[str] = []
    for name, src, key in METRIC_SOURCES:
        block = parsed.get(src) or {}
        if key in block:
            out[name] = block[key]
        else:
            out[name] = None
            missing.append(name)

    # FINAL fcs=a/b -> explicit total
    total = None
    if parsed.get("final"):
        ok, tot = parse_ratio(parsed["final"].get("fcs"))
        if tot is not None:
            out["final_fcs_ok"], out["final_fcs_total"] = ok, tot
            total = tot
    out.setdefault("final_fcs_ok", None)
    out.setdefault("final_fcs_total", None)

    # derived frontend ratios
    scan_us = num(out.get("frontend_scan_us"))
    front_us = num(out.get("frontend_us"))
    samples = num(out.get("frontend_samples"))
    out["frontend_scan_fraction"] = (
        scan_us / front_us if scan_us is not None and front_us else None)
    out["scan_us_per_sample"] = (
        scan_us / samples if scan_us is not None and samples else None)

    fcs_ok = num(out.get("fcs_ok"))
    slots = num(out.get("expected_slots"))
    out["target_fcs_pct"] = (
        100.0 * fcs_ok / slots if fcs_ok is not None and slots else None)

    # summary fail counters (absent counter -> 0 is the true meaning of an
    # absent batch line; keep the alias set so the JSON shape is stable)
    for alias in sorted(set(SUMMARY_FAIL_ALIASES.values())):
        out["summary_" + alias] = parsed.get("summary_failures", {}).get(alias, 0)

    if missing:
        warnings.append(f"{tag}: fields absent -> null: " + ", ".join(missing))
    return out


# --------------------------------------------------------------------------
# provenance
# --------------------------------------------------------------------------
def git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], capture_output=True, text=True,
                              check=False).stdout.strip()
    except OSError as e:  # git missing
        return f"<git unavailable: {e}>"


def cpu_info() -> Dict[str, Any]:
    model = "unknown"
    flags: List[str] = []
    cores = 0
    try:
        with open("/proc/cpuinfo", "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if line.startswith("model name") and model == "unknown":
                    model = line.split(":", 1)[1].strip()
                elif line.startswith("flags") and not flags:
                    flags = line.split(":", 1)[1].split()
                elif line.startswith("processor"):
                    cores += 1
    except OSError:
        pass
    wanted = ["avx", "avx2", "fma", "avx512f", "avx512bw", "avx512vl", "avx512dq"]
    return {
        "model": model,
        "logical_cpus": cores,
        "nproc": os.cpu_count(),
        "affinity_cpus": (len(os.sched_getaffinity(0))
                          if hasattr(os, "sched_getaffinity") else None),
        "isa": {k: (k in flags) for k in wanted},
        "isa_flags_subset": [k for k in wanted if k in flags],
    }


def build_cache_note(binary: str) -> Dict[str, Any]:
    build_dir = os.path.dirname(os.path.dirname(os.path.abspath(binary)))
    cache = os.path.join(build_dir, "CMakeCache.txt")
    note: Dict[str, Any] = {
        "build_dir": build_dir,
        "cache_file": cache if os.path.exists(cache) else None,
        "cmake_entries": {},
        "reproduction": (
            "cmake --preset native-avx2 "
            "-DCMAKE_BUILD_TYPE=Release -DENABLE_UHD_BACKEND=ON && "
            "cmake --build --preset native-avx2 -j4; "
            "every run used env -u LD_LIBRARY_PATH"),
    }
    if not note["cache_file"]:
        return note
    keep = ("CMAKE_BUILD_TYPE", "ENABLE_UHD_BACKEND", "CMAKE_CXX_FLAGS",
            "CMAKE_CXX_COMPILER", "CMAKE_CXX_FLAGS_RELEASE", "UHD_VERSION")
    try:
        with open(cache, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "=" not in line or line.startswith("//"):
                    continue
                k, v = line.rstrip("\n").split("=", 1)
                base = k.split(":", 1)[0]
                if base in keep:
                    note["cmake_entries"][base] = v
    except OSError as e:
        note["error"] = str(e)
    return note


def file_info(path: str, want_hash: bool) -> Dict[str, Any]:
    st = os.stat(path)
    info = {
        "path": os.path.abspath(path),
        "size": st.st_size,
        "mtime": iso_from_epoch(st.st_mtime),
        "mtime_epoch": st.st_mtime,
    }
    if want_hash:
        info["sha256"] = sha256_file(path)
    return info


# --------------------------------------------------------------------------
# command construction
# --------------------------------------------------------------------------
def flatten_extra(extras: Iterable[str]) -> List[str]:
    toks: List[str] = []
    for e in extras:
        if e is None:
            continue
        # allow both `--extra "--period-ms 1"` and `--extra --period-ms=1`
        toks.extend(shlex.split(e) if any(c.isspace() for c in e) else [e])
    return toks


def build_arm_args(args: argparse.Namespace, arm: str) -> List[str]:
    cmd = [args.binary]
    cmd += flatten_extra(args.extra)          # passthrough first ...
    cmd += BASE_ARGS                          # ... fixed base args win
    cmd += ["--workers", str(args.workers), "--queue", str(args.queue)]
    cmd += ["--replay", args.replay]
    if args.seconds and args.seconds > 0:
        cmd += ["--seconds", f"{args.seconds:g}"]
    cmd += ["--autocorr-when-locked", arm]    # arm selection is last
    return cmd


def plan_runs(rounds: int) -> List[Tuple[int, str]]:
    """A/B/B/A blocks; each block yields two (scan, suspend) pairs.

    Block k runs A1 B1 B2 A2, so pair 2k is (A1,B1) and pair 2k+1 is (B2,A2):
    the second pair is inner-adjacent, the first pair brackets the block.
    """
    blocks = (rounds + 1) // 2
    plan: List[Tuple[int, str]] = []
    for b in range(blocks):
        pair_a, pair_b = 2 * b + 1, 2 * b + 2
        plan.append((pair_a, "scan"))
        plan.append((pair_a, "suspend"))
        plan.append((pair_b, "suspend"))
        plan.append((pair_b, "scan"))
    return plan


# --------------------------------------------------------------------------
# execution
# --------------------------------------------------------------------------
def run_one(cmd: Sequence[str], timeout: float, logs_dir: str, idx: int,
            arm: str, env: Dict[str, str]) -> Dict[str, Any]:
    log_path = os.path.join(logs_dir, f"run_{idx:02d}_{arm}.log")
    started_wall = time.time()
    t0 = time.monotonic()
    timed_out = False
    rc: Optional[int] = None
    stdout = stderr = ""
    try:
        proc = subprocess.Popen(list(cmd), stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, env=env,
                                start_new_session=True)
        try:
            stdout, stderr = proc.communicate(timeout=timeout)
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
            try:
                os.killpg(proc.pid, 9)
            except ProcessLookupError:
                pass
            stdout, stderr = proc.communicate()
            rc = proc.returncode
    except OSError as e:
        stderr = f"<spawn failed: {e}>"
        rc = None
    wall = time.monotonic() - t0
    ended_wall = time.time()

    with open(log_path, "w", encoding="utf-8") as f:
        f.write("CMD: env -u LD_LIBRARY_PATH " + shlex.join(list(cmd)) + "\n")
        f.write(f"STARTED: {iso_from_epoch(started_wall)}  "
                f"ENDED: {iso_from_epoch(ended_wall)}  "
                f"WALL_S: {wall:.3f}  RC: {rc}  TIMEOUT: {timed_out}\n")
        f.write("--- STDOUT ---\n" + stdout)
        if stdout and not stdout.endswith("\n"):
            f.write("\n")
        f.write("--- STDERR ---\n" + stderr)
        if stderr and not stderr.endswith("\n"):
            f.write("\n")

    return {
        "index": idx,
        "arm": arm,
        "argv": list(cmd),
        "shell": "env -u LD_LIBRARY_PATH " + shlex.join(list(cmd)),
        "start_wall": iso_from_epoch(started_wall),
        "end_wall": iso_from_epoch(ended_wall),
        "start_epoch": started_wall,
        "end_epoch": ended_wall,
        "wall_s": wall,
        "returncode": rc,
        "timed_out": timed_out,
        "log": log_path,
        "stdout_bytes": len(stdout.encode()),
        "stderr_bytes": len(stderr.encode()),
        "stderr_tail": stderr[-4000:],
        "stdout": stdout,
    }


def pair_stats(runs: Sequence[Dict[str, Any]], plan: Sequence[Tuple[int, str]],
               warnings: List[str]) -> List[Dict[str, Any]]:
    """Pair the scan/suspend runs that belong to the same ABBA pair."""
    per_pair: Dict[int, Dict[str, Dict[str, Any]]] = {}
    for rec, (pair, arm) in zip(runs, plan):
        per_pair.setdefault(pair, {})[arm] = rec
    pairs: List[Dict[str, Any]] = []
    for pair in sorted(per_pair):
        g = per_pair[pair]
        if "scan" not in g or "suspend" not in g:
            warnings.append(f"pair {pair}: incomplete (missing arm)")
            continue
        pairs.append({
            "round": pair,
            "scan_run": g["scan"]["index"],
            "suspend_run": g["suspend"]["index"],
            "scan_wall_s": g["scan"]["wall_s"],
            "suspend_wall_s": g["suspend"]["wall_s"],
            "metrics": {},
        })
    return pairs


def aggregate(pairs: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    names = [m for m, _, _ in METRIC_SOURCES] + DERIVED_METRICS
    summary: Dict[str, Any] = {}
    for name in names:
        scan_vals: List[float] = []
        susp_vals: List[float] = []
        deltas: List[float] = []
        for p in pairs:
            m = p["metrics"].get(name)
            if not m:
                continue
            a, b = m.get("scan"), m.get("suspend")
            if a is not None:
                scan_vals.append(a)
            if b is not None:
                susp_vals.append(b)
            if a is not None and b is not None:
                deltas.append(b - a)
        if not scan_vals and not susp_vals:
            summary[name] = {
                "scan_values": [], "suspend_values": [], "deltas": [],
                "scan_mean": None, "suspend_mean": None,
                "delta_median": None, "delta_mean": None,
                "relative_change_pct": None, "n": 0,
            }
            continue
        sa, su = r6(mean(scan_vals)), r6(mean(susp_vals))
        summary[name] = {
            "scan_values": scan_vals,
            "suspend_values": susp_vals,
            "deltas": deltas,
            "scan_mean": sa,
            "suspend_mean": su,
            "scan_min": min(scan_vals) if scan_vals else None,
            "scan_max": max(scan_vals) if scan_vals else None,
            "suspend_min": min(susp_vals) if susp_vals else None,
            "suspend_max": max(susp_vals) if susp_vals else None,
            "delta_median": r6(median(deltas)),
            "delta_mean": r6(mean(deltas)),
            "relative_change_pct": (r6(100.0 * (su - sa) / sa)
                                    if sa not in (None, 0.0) and su is not None
                                    else None),
            "n": len(deltas),
        }
    return summary


def target_flags(pairs: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    flags: List[Dict[str, Any]] = []
    totals = {"scan": {}, "suspend": {}}
    for name in TARGET_METRICS:
        for arm in ("scan", "suspend"):
            vals = [p["metrics"].get(name, {}).get(arm) for p in pairs]
            vals = [v for v in vals if v is not None]
            totals[arm][name] = sum(vals) if vals else None
    for p in pairs:
        for name in TARGET_METRICS:
            a = p["metrics"].get(name, {}).get("scan")
            b = p["metrics"].get(name, {}).get("suspend")
            if a != b:
                flags.append({"round": p["round"], "metric": name,
                              "scan": a, "suspend": b,
                              "delta": (b - a) if isinstance(a, (int, float))
                              and isinstance(b, (int, float)) else None})
    return {"identical": not flags, "differences": flags,
            "totals_across_runs": totals}


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------
def print_table(agg: Dict[str, Any], tf: Dict[str, Any],
                pairs: Sequence[Dict[str, Any]]) -> None:
    order = ["fcs_ok", "not_detected", "fcs_failed", "identity_rejects",
             "lock_entries", "holdover_entries", "reacquire_entries",
             "expected_slots", "autocorr_ignored",
             "autocorr_suspend_events", "autocorr_resume_events",
             "autocorr_suspended_samples", "autocorr_resume_backlog_samples",
             "frontend_scan_us", "frontend_us", "frontend_scan_fraction",
             "scan_us_per_sample", "acq_seeds", "submitted", "dropped_queue",
             "candidates", "cand_unknown", "final_completed",
             "final_fcs_ok", "summary_timing_failed", "summary_sfd_failed",
             "summary_payload_failed"]
    header = (f"{'metric':<31} {'scan(mean)':>14} {'suspend(mean)':>15} "
              f"{'delta_med':>13} {'delta_mean':>13} {'chg%':>9}")
    print("\nABBA paired result (delta = suspend - scan, n = "
          f"{len(pairs)} pairs)")
    print(header)
    print("-" * len(header))
    for name in order:
        s = agg.get(name)
        if not s or s["n"] == 0:
            continue
        dig = METRIC_DIGITS.get(name, 3)
        print(f"{name:<31} {cell(s['scan_mean'], dig):>14} "
              f"{cell(s['suspend_mean'], dig):>15} "
              f"{cell(s['delta_median'], dig):>13} "
              f"{cell(s['delta_mean'], dig):>13} "
              f"{cell(s['relative_change_pct'], 1):>9}")
    if tf["identical"]:
        print("\ntarget counters (fcs_ok/not_detected/fcs_failed/"
              "identity_rejects/lock entries): identical in every pair")
    else:
        print("\nTARGET COUNTER DIFFERENCES (must be explained per packet):")
        for f in tf["differences"]:
            print(f"  round {f['round']}: {f['metric']} "
                  f"scan={f['scan']} suspend={f['suspend']} "
                  f"delta={f['delta']}")


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Serial ABBA comparison of --autocorr-when-locked "
                    "scan vs suspend on a replay fixture.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--binary", default=DEFAULT_BINARY)
    ap.add_argument("--replay", required=True)
    ap.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS,
                    help="number of scan/suspend pairs (ABBA blocks of 2 pairs)")
    ap.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    ap.add_argument("--queue", type=int, default=DEFAULT_QUEUE)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="replay duration in seconds; 0 = whole file")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT,
                    help="per-invocation timeout in seconds")
    ap.add_argument("--extra", action="append", default=[],
                    metavar="ARG",
                    help="extra live argument(s), repeatable; may contain a "
                         "quoted group such as --extra '--period-ms 1'")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the exact command lines and exit")
    ap.add_argument("--quiet", action="store_true")
    args, unknown = ap.parse_known_args(argv)

    # `-- ...` passthrough: argparse hands the remainder to `unknown`.
    if unknown:
        if unknown and unknown[0] == "--":
            unknown = unknown[1:]
        args.extra = list(args.extra) + list(unknown)

    if args.rounds < 1:
        die("--rounds must be >= 1")
    if args.workers < 1 or args.queue < 1:
        die("--workers/--queue must be >= 1")
    if args.timeout <= 0:
        die("--timeout must be > 0")

    for path, what in ((args.binary, "binary"), (args.replay, "replay")):
        if not os.path.exists(path):
            die(f"{what} not found: {path}")
    if not os.access(args.binary, os.X_OK):
        die(f"binary is not executable: {args.binary}")
    if os.path.getsize(args.replay) == 0:
        die(f"replay fixture is empty: {args.replay}")

    warnings: List[str] = []
    if args.rounds % 2:
        warnings.append(f"--rounds={args.rounds} is odd; ABBA needs pairs, "
                        f"running {args.rounds + 1} pairs (one full block)")

    plan = plan_runs(args.rounds if args.rounds % 2 == 0 else args.rounds + 1)
    blocks = (len(plan) + 3) // 4
    if not args.quiet:
        print(f"plan: {blocks} ABBA block(s), {len(plan)} runs "
              f"({len(plan)//2} pairs), order A/B/B/A", file=sys.stderr)

    # environment: an installed library must never shadow the build tree
    env = {k: v for k, v in os.environ.items() if k != "LD_LIBRARY_PATH"}
    if "LD_LIBRARY_PATH" in os.environ:
        warnings.append("LD_LIBRARY_PATH was removed from the child environment")

    # passthrough sanity: warn if it redefines an argument we own
    flat = flatten_extra(args.extra)
    for tok in flat:
        key = tok.split("=", 1)[0]
        if key in OWNED_KEYS:
            warnings.append(f"passthrough {key} is overridden by the fixed "
                            "base arguments (extras are placed first)")

    cmds = {arm: build_arm_args(args, arm) for arm in ("scan", "suspend")}
    if args.dry_run:
        print(f"# fixture: {os.path.abspath(args.replay)}")
        for i, (pair, arm) in enumerate(plan, 1):
            print(f"# run {i}/{len(plan)} pair {pair} arm {arm}")
            print("env -u LD_LIBRARY_PATH " + shlex.join(cmds[arm]))
        print(f"# out: {os.path.abspath(args.out)}")
        return 0

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    logs_dir = os.path.splitext(out_path)[0] + "_logs"
    os.makedirs(logs_dir, exist_ok=True)

    doc: Dict[str, Any] = {
        "schema": SCHEMA,
        "status": "running",
        "timestamp_utc": utc_now(),
        "tool": {
            "path": os.path.abspath(__file__),
            "sha256": sha256_file(os.path.abspath(__file__)),
            "python": platform.python_version(),
        },
        "git": {
            "commit": git("rev-parse", "HEAD"),
            "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "status_porcelain": git("status", "--porcelain"),
        },
        "binary": file_info(args.binary, want_hash=True),
        "replay": file_info(args.replay, want_hash=True),
        "host": {
            **cpu_info(),
            "kernel": platform.release(),
            "os": platform.platform(),
            "machine": platform.machine(),
        },
        "build": build_cache_note(args.binary),
        "run_config": {
            "rounds_pairs": len(plan) // 2,
            "rounds_requested": args.rounds,
            "abba_blocks": blocks,
            "order": "A/B/B/A (scan/suspend)",
            "workers": args.workers,
            "queue": args.queue,
            "seconds": args.seconds,
            "timeout_s": args.timeout,
            "base_args": BASE_ARGS,
            "extra_args": flat,
            "scan_argv": cmds["scan"],
            "suspend_argv": cmds["suspend"],
            "env_note": "child env = os.environ without LD_LIBRARY_PATH",
        },
        "runs": [],
        "pairs": [],
        "metric_summary": {},
        "target": {},
        "notes": [
            "fcs_ok/not_detected/fcs_failed/identity_rejects and the lock "
            "entries come from LOCK_STATS (target-lock scheduler counters).",
            "final_* comes from FINAL: it includes acquire/candidate windows, "
            "so it is not target PRR and must not be quoted as such.",
            "FINAL has no timing/sfd/payload counters; those come from the "
            "SUMMARY_* fail tokens (summary_timing_failed etc.).",
            "frontend_scan_fraction can exceed 1 by <1%: frontend_us and "
            "frontend_scan_us take separate clock reads.",
            "ABBA pairing: per block (A1,B1,B2,A2) pair 1 = (A1,B1) and "
            "pair 2 = (B2,A2); delta = suspend - scan.",
            "runs are serial and ordered; wall_s is dominated by replay "
            "pacing, frontend_scan_us is the metric of interest.",
        ],
        "warnings": warnings,
    }

    runs: List[Dict[str, Any]] = []
    failed = False
    for i, (pair, arm) in enumerate(plan, 1):
        if not args.quiet:
            print(f"[{i}/{len(plan)}] pair {pair} arm={arm} ...",
                  file=sys.stderr, flush=True)
        rec = run_one(cmds[arm], args.timeout, logs_dir, i, arm, env)
        parsed = parse_run_output(rec.pop("stdout"), warnings,
                                  f"run{i}/{arm}")
        rec["pair"] = pair
        rec["parsed"] = parsed
        rec["metrics"] = extract_metrics(parsed, warnings, f"run{i}/{arm}")
        runs.append(rec)
        doc["runs"] = runs
        if not args.quiet:
            m = rec["metrics"]
            print(f"      rc={rec['returncode']} wall={rec['wall_s']:.1f}s "
                  f"fcs_ok={fmt(m.get('fcs_ok'))} "
                  f"slots={fmt(m.get('expected_slots'))} "
                  f"scan_us={fmt(m.get('frontend_scan_us'), 0)} "
                  f"locks={fmt(m.get('lock_entries'))}",
                  file=sys.stderr, flush=True)
        if rec["timed_out"] or rec["returncode"] != 0:
            failed = True
            warnings.append(
                f"run {i} ({arm}) failed: rc={rec['returncode']} "
                f"timeout={rec['timed_out']} log={rec['log']}")
            tail = rec["stderr_tail"].strip().splitlines()[-5:]
            if tail:
                warnings.append("stderr tail: " + " | ".join(tail))
            break

    if not failed:
        pairs = pair_stats(runs, plan, warnings)
        by_index = {r["index"]: r for r in runs}
        for p in pairs:
            for name in [m for m, _, _ in METRIC_SOURCES] + DERIVED_METRICS:
                p["metrics"][name] = {
                    "scan": by_index[p["scan_run"]]["metrics"].get(name),
                    "suspend": by_index[p["suspend_run"]]["metrics"].get(name),
                }
        doc["pairs"] = pairs
        doc["metric_summary"] = aggregate(pairs)
        doc["target"] = target_flags(pairs)
        doc["status"] = "ok"
    else:
        doc["status"] = "failed"

    doc["finished_utc"] = utc_now()
    doc["warnings"] = sorted(set(warnings))
    gate_missing = any(
        r.get("returncode") == 0
        and r["metrics"].get("autocorr_suspend_events") is None
        for r in doc["runs"])
    doc["gate_counters_present"] = not gate_missing

    tmp = out_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, ensure_ascii=False)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, out_path)

    if gate_missing:
        print("warning: LOCK_STATS carries no autocorr_suspend_events/resume "
              "counters - the binary predates --autocorr-when-locked "
              "(the A/B arms cannot differ)", file=sys.stderr)

    if not failed:
        print_table(doc["metric_summary"], doc["target"], doc["pairs"])
        print(f"\nbinary commit: {doc['git']['commit'][:12]}  "
              f"replay sha256: {doc['replay']['sha256'][:16]}...  "
              f"bytes: {doc['replay']['size']}")
        print(f"json: {out_path}\nlogs: {logs_dir}")
        if doc["warnings"]:
            print("\nwarnings:")
            for w in doc["warnings"]:
                print("  - " + w)

    if failed:
        print("FAILED - see warnings and per-run logs", file=sys.stderr)
        print(f"json (partial): {out_path}", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
