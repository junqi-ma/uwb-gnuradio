#!/usr/bin/env python3
"""Paired functional/timing check for the sparse-pulse SIC experiment.

Old binary: the accepted unit-real Ipatov path.
New binary: the same source plus --sparse-pulse-reconstruct.

The script is orchestration only.  The C++ runner is the online signal path;
raw/clean IQ and sidecars are read here only after the runner has completed.
Formal timing requires an exclusive /tmp/uwb-sic-benchmark.lock and removes
LD_LIBRARY_PATH so the loaded library is the one under test.
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
import statistics
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LOCK = Path("/tmp/uwb-sic-benchmark.lock")
INPUTS = ROOT / "gr-uwb/build-sic-fullpacket/results/inputs16"
CORRUPT = (ROOT / "gr-uwb/build-sic-fullpacket-fused-cmf/results/"
           "inputs16_corrupt/payload_base_corrupt")
OLD_BUILD = ROOT / "gr-uwb/build-sic-fullpacket-unit-real"
NEW_BUILD = ROOT / "gr-uwb/build-sic-fullpacket-pulse-list"
RX_FLAGS = [
    "--rx-fast-sfd", "--rx-fast-cfo", "--rx-seed-refine-tail",
    "--rx-softchip-on-demand", "--rx-fused-payload-cmf", "--rx-fused-phr-cmf",
]
FUNCTIONAL = [
    ("payload_base", 8, False),
    ("payload_four_frac", 8, False),
    ("payload_cfo_m50k", 8, False),
    ("payload_cfo_p50k", 8, False),
    ("sync_base", 8, False),
    ("payload_len127", 2, False),
    ("corrupt", 2, True),
]
TIMING = [("payload_four_frac", "payload_four_frac"),
          ("payload_cfo_p50k", "payload_cfo_p50k")]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def git(*args: str) -> str:
    return subprocess.run(["git", "-C", str(ROOT), *args], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          check=True).stdout.strip()


def clean_env() -> dict[str, str]:
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    return env


def run_logged(command: list[str], log: Path, env: dict[str, str]) -> int:
    with log.open("w") as f:
        return subprocess.run(command, cwd=ROOT, env=env, text=True,
                              stdout=f, stderr=subprocess.STDOUT).returncode


def binary(build: Path) -> Path:
    return build / "apps/benchmark_fullpacket_compare_fast_rx"


def library_binding(path: Path, build: Path, env: dict[str, str]) -> dict:
    ldd = subprocess.run(["ldd", str(path)], env=env, text=True,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         check=True).stdout
    loaded = []
    for line in ldd.splitlines():
        if "libgnuradio-uwb" in line and "=>" in line:
            lib = Path(line.split("=>", 1)[1].strip().split()[0]).resolve()
            if not lib.is_relative_to(build.resolve()):
                raise RuntimeError(f"library outside build: {lib}")
            loaded.append({"path": str(lib), "sha256": sha256(lib)})
    if not loaded:
        raise RuntimeError("ldd did not resolve libgnuradio-uwb")
    return {"ldd": ldd, "libraries": loaded}


def case_path(cell: str, corrupt: bool) -> Path:
    return CORRUPT if corrupt else INPUTS / cell


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               allow_nan=False) + "\n")


def run_case(build: Path, cell: str, corrupt: bool, dest: Path,
             repeat: int, warmup: int, dump: bool, sparse: bool,
             log: Path, env: dict[str, str]) -> list[dict]:
    if dest.exists():
        raise RuntimeError(f"output already exists: {dest}")
    command = [
        str(binary(build)),
        "--case", str(case_path(cell, corrupt)),
        "--out", str(dest),
        "--repeat", str(repeat),
        "--warmup", str(warmup),
        "--mode", "full",
        "--reconstruction", "ipatov_cir",
        *RX_FLAGS,
    ]
    if dump:
        command.append("--dump-iq")
    if sparse:
        command.append("--sparse-pulse-reconstruct")
    if run_logged(command, log, env) != 0:
        raise RuntimeError(f"runner failed; see {log}")
    rows = [json.loads(line) for line in
            (dest / "windows.jsonl").read_text().splitlines() if line]
    if len(rows) != repeat:
        raise RuntimeError(f"{dest}: expected {repeat} rows, got {len(rows)}")
    return rows


def effect(case: Path, run: Path, rows: list[dict]) -> dict:
    pub = json.loads((case / "public.json").read_text())
    a, b = int(pub["target_lo"]), int(pub["target_hi"])
    before = after = target_before = target_after = 0.0
    per_window = []
    for row in rows:
        source = int(row["source_window"])
        window = case / "windows" / f"w{source:06d}"
        raw = np.fromfile(window / "raw.cf32", np.complex64)
        clean = np.fromfile(window / "clean.cf32", np.complex64)
        got = np.fromfile(run / f"w{row['window_id']:06d}_full.cf32",
                          np.complex64)
        if raw.size != got.size:
            raise RuntimeError(f"output length mismatch in {run}")
        raw_err = raw - clean
        got_err = got - clean
        e0 = float(np.vdot(raw_err, raw_err).real)
        e1 = float(np.vdot(got_err, got_err).real)
        t0 = float(np.vdot(raw_err[a:b], raw_err[a:b]).real)
        t1 = float(np.vdot(got_err[a:b], got_err[a:b]).real)
        before += e0
        after += e1
        target_before += t0
        target_after += t1
        per_window.append({
            "window_id": row["window_id"],
            "source_window": source,
            "all_suppression_db": 10.0 * math.log10(e0 / e1)
            if e0 > 0 and e1 > 0 else float("-inf"),
            "target_suppression_db": 10.0 * math.log10(t0 / t1)
            if t0 > 0 and t1 > 0 else float("-inf"),
            "committed": row["committed"],
            "comm_fcs": row["comm_fcs"],
            "sensing_fcs": row["sensing_fcs"],
        })
    return {
        "all_suppression_db": 10.0 * math.log10(before / after),
        "target_suppression_db": 10.0 * math.log10(target_before / target_after),
        "committed": sum(bool(r["committed"]) for r in rows),
        "comm_fcs": sum(bool(r["comm_fcs"]) for r in rows),
        "sensing_fcs": sum(bool(r["sensing_fcs"]) for r in rows),
        "per_window": per_window,
    }


DECODE_KEYS = [
    "comm_psdu_hex", "comm_fcs", "comm_status", "sensing_psdu_hex",
    "sensing_fcs", "sensing_status", "committed", "reason",
    "comm_phr_length", "comm_phr_bits", "comm_phr_secded_corrected",
    "comm_phr_secded_uncorrectable", "comm_phr_data_rate_mbps",
]
MODEL_KEYS = [
    "q", "cfo_hz", "lag_lo", "tap_count", "active_count", "impulse_template",
    "cir_repetitions", "fit_residual", "condition",
]


def compare_rows(old_rows: list[dict], new_rows: list[dict]) -> list[dict]:
    mismatches = []
    for i, (a, b) in enumerate(zip(old_rows, new_rows)):
        fields = [k for k in DECODE_KEYS if a.get(k) != b.get(k)]
        for k in MODEL_KEYS:
            if a.get("discrete_cir", {}).get(k) != b.get("discrete_cir", {}).get(k):
                fields.append("discrete_cir." + k)
        if fields:
            mismatches.append({"row": i, "fields": fields})
    if len(old_rows) != len(new_rows):
        mismatches.append({"row": -1, "fields": ["row_count"]})
    return mismatches


def metric_stats(rows: list[dict], fn) -> dict:
    values = [float(fn(r)) for r in rows]
    return {
        "mean": statistics.mean(values),
        "p95": float(np.percentile(values, 95)),
        "min": min(values),
        "max": max(values),
        "n": len(values),
    }


def metrics(rows: list[dict]) -> dict:
    def dget(row, *path):
        cur = row
        for key in path:
            if isinstance(cur, dict) and key in cur:
                cur = cur[key]
            else:
                return 0.0
        return float(cur)

    return {
        "tx_event_generation": metric_stats(rows, lambda r: dget(r, "times_us", "tx")),
        "channel_reconstruct": metric_stats(rows, lambda r: dget(r, "times_us", "channel")),
        "sic_total": metric_stats(rows, lambda r: dget(r, "times_us", "sic_total")),
        "estimate": metric_stats(rows, lambda r: dget(r, "times_us", "estimate")),
        "comm_demod": metric_stats(rows, lambda r: dget(r, "times_us", "comm_demod")),
        "convolution": metric_stats(
            rows, lambda r: dget(r, "discrete_cir", "times_us", "convolution")),
        "zero": metric_stats(rows, lambda r: dget(r, "discrete_cir", "times_us", "zero")),
        "phase": metric_stats(rows, lambda r: dget(r, "discrete_cir", "times_us", "phase")),
        "check": metric_stats(rows, lambda r: dget(r, "discrete_cir", "times_us", "check")),
        "cfo_rotation": metric_stats(
            rows, lambda r: dget(r, "discrete_cir", "times_us", "cfo_rot")),
        "subtract": metric_stats(
            rows, lambda r: dget(r, "discrete_cir", "times_us", "subtract")),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path)
    parser.add_argument("--old-build", type=Path, default=OLD_BUILD)
    parser.add_argument("--new-build", type=Path, default=NEW_BUILD)
    parser.add_argument("--functional-only", action="store_true")
    parser.add_argument("--timing-only", action="store_true")
    args = parser.parse_args()

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = args.out or (args.new_build / "results" / f"sparse_pulse_{stamp}")
    if out.exists():
        raise RuntimeError(f"output directory exists: {out}")
    out.mkdir(parents=True)
    env = clean_env()

    old_bin, new_bin = binary(args.old_build), binary(args.new_build)
    for b in (old_bin, new_bin):
        if not b.is_file():
            raise RuntimeError(f"missing runner: {b}")
    provenance = {
        "schema": "sparse-pulse-compare/1",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "git_head": git("rev-parse", "HEAD"),
        "git_dirty": bool(git("status", "--porcelain=v1",
                              "--untracked-files=normal")),
        "git_status": git("status", "--porcelain=v1",
                          "--untracked-files=normal").splitlines(),
        "old_build": str(args.old_build),
        "new_build": str(args.new_build),
        "old_binary": {"path": str(old_bin), "sha256": sha256(old_bin)},
        "new_binary": {"path": str(new_bin), "sha256": sha256(new_bin)},
        "old_binding": library_binding(old_bin, args.old_build, env),
        "new_binding": library_binding(new_bin, args.new_build, env),
        "rx_flags": RX_FLAGS,
        "sparse_flag": "--sparse-pulse-reconstruct",
        "reconstruction": "ipatov_cir",
        "cir_geometry": {"lag_lo": -8, "tap_count": 128,
                         "description": "frozen Ipatov 128-point correlation response"},
        "functional_cells": [{"cell": c, "windows": n, "corrupt": k}
                             for c, n, k in FUNCTIONAL],
        "timing_cells": [c for c, _ in TIMING],
        "timing_design": "ABBA, 8 warmup + 64 observations per run, 128/variant",
        "machine": {"platform": platform.platform(),
                    "cpu_count": os.cpu_count()},
        "lock": str(LOCK),
        "ld_library_path_removed": True,
        "notes": "Functional IQ is compared per window; timing rows are "
                 "also checked against the functional decode reference.",
    }
    # Offline provenance: public profile and source-window hashes, not inputs
    # to the online runner.  The same hashes let a later run prove it reused
    # exactly the same source windows.
    provenance["public_profiles"] = {}
    provenance["source_windows"] = {}
    seen = set()
    for cell, count, corrupt in FUNCTIONAL + [(c, 16, False) for _, c in TIMING]:
        if cell in seen:
            continue
        seen.add(cell)
        case = case_path(cell, corrupt)
        pub = json.loads((case / "public.json").read_text())
        provenance["public_profiles"][cell] = {
            k: pub.get(k) for k in (
                "comm_code", "sensing_code", "sync_reps", "sfd_mode",
                "pulse_shape", "sample_rate", "n", "observed_end",
                "roi_lo", "roi_hi", "consumer_lo", "consumer_hi")
        }
        provenance["source_windows"][cell] = [
            {"window": w.name, "raw_sha256": sha256(w / "raw.cf32")}
            for w in sorted(p for p in (case / "windows").iterdir()
                            if p.is_dir())
        ]
    patch = subprocess.run(["git", "-C", str(ROOT), "diff", "--binary"],
                           stdout=subprocess.PIPE, check=True).stdout
    (out / "code.patch").write_bytes(patch)
    provenance["code_patch_sha256"] = hashlib.sha256(patch).hexdigest()
    write_json(out / "provenance.json", provenance)

    summary = {"functional": {}, "timing": {}, "artifacts": {}}
    logs = out / "logs"
    logs.mkdir()

    with LOCK.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        print(f"output={out}", flush=True)
        if not args.timing_only:
            for cell, count, corrupt in FUNCTIONAL:
                case = case_path(cell, corrupt)
                rows = {}
                runs = {}
                for variant, build, sparse in (
                        ("old", args.old_build, False),
                        ("new", args.new_build, True)):
                    dest = out / "functional" / cell / variant
                    rows[variant] = run_case(
                        build, cell, corrupt, dest, count, 0, True, sparse,
                        logs / f"functional_{cell}_{variant}.log", env)
                    runs[variant] = dest
                entry = {
                    "windows": count,
                    "corrupt": corrupt,
                    "rows": len(rows["new"]),
                    "new_sparse": all(r.get("sparse_pulse_reconstruct") is True
                                      for r in rows["new"]),
                }
                if corrupt:
                    old_ok = all(not r["committed"] and not r["comm_fcs"]
                                 for r in rows["old"])
                    new_ok = all(not r["committed"] and not r["comm_fcs"]
                                 for r in rows["new"])
                    exact = 0
                    for row in rows["new"]:
                        raw = np.fromfile(case / "windows" /
                                          f"w{row['source_window']:06d}" /
                                          "raw.cf32", np.complex64)
                        got = np.fromfile(runs["new"] /
                                          f"w{row['window_id']:06d}_full.cf32",
                                          np.complex64)
                        exact += int(np.array_equal(raw, got))
                    entry.update({
                        "old_zero_commit": old_ok,
                        "new_zero_commit": new_ok,
                        "new_exact_bypass": exact == count,
                        "old_effect": None,
                        "new_effect": None,
                        "mismatches": compare_rows(rows["old"], rows["new"]),
                    })
                else:
                    mismatches = compare_rows(rows["old"], rows["new"])
                    old_effect = effect(case, runs["old"], rows["old"])
                    new_effect = effect(case, runs["new"], rows["new"])
                    deltas = [new_effect["per_window"][i]["all_suppression_db"] -
                              old_effect["per_window"][i]["all_suppression_db"]
                              for i in range(count)]
                    # The old binary predates tap_hash.  A one-window control
                    # run in the NEW build without the sparse switch proves the
                    # switch itself did not change CIR/q/CFO.
                    control_dest = out / "control" / cell / "nonsparse"
                    control_rows = run_case(
                        args.new_build, cell, False, control_dest, 1, 0,
                        False, False, logs / f"control_{cell}_nonsparse.log",
                        env)
                    new0 = rows["new"][0]["discrete_cir"]
                    ctl0 = control_rows[0]["discrete_cir"]
                    control = {
                        "windows": 1,
                        "tap_hash_match": new0.get("tap_hash") == ctl0.get("tap_hash"),
                        "model_fields_match": all(
                            new0.get(k) == ctl0.get(k) for k in MODEL_KEYS),
                        "decode_match": all(
                            rows["new"][0].get(k) == control_rows[0].get(k)
                            for k in DECODE_KEYS),
                    }
                    entry.update({
                        "old_effect": old_effect,
                        "new_effect": new_effect,
                        "all_suppression_delta_db":
                            old_effect["all_suppression_db"] -
                            new_effect["all_suppression_db"],
                        "target_suppression_delta_db":
                            old_effect["target_suppression_db"] -
                            new_effect["target_suppression_db"],
                        "worst_window_all_delta_db": min(deltas),
                        "best_window_all_delta_db": max(deltas),
                        "mismatches": mismatches,
                        "control": control,
                        "old_all_fcs": (old_effect["comm_fcs"] == count and
                                        old_effect["sensing_fcs"] == count),
                        "new_all_fcs": (new_effect["comm_fcs"] == count and
                                        new_effect["sensing_fcs"] == count),
                    })
                summary["functional"][cell] = entry
                write_json(out / "summary.json", summary)
                print(cell, json.dumps({k: entry[k] for k in entry
                                        if k not in ("old_effect", "new_effect")},
                                       sort_keys=True), flush=True)

        if not args.functional_only:
            # Timing uses all 16 source windows.  Build a decode reference over
            # the full fixture separately from the 8-window functional matrix.
            timing_reference = {}
            summary["timing_reference"] = {}
            for label, cell in TIMING:
                refs = {}
                for variant, build, sparse in (
                        ("old", args.old_build, False),
                        ("new", args.new_build, True)):
                    dest = out / "timing_reference" / cell / variant
                    refs[variant] = run_case(
                        build, cell, False, dest, 16, 0, False, sparse,
                        logs / f"timing_reference_{cell}_{variant}.log", env)
                mismatches = compare_rows(refs["old"], refs["new"])
                summary["timing_reference"][cell] = {
                    "windows": 16,
                    "mismatches": mismatches,
                    "old_all_fcs": all(r["comm_fcs"] and r["sensing_fcs"]
                                       for r in refs["old"]),
                    "new_all_fcs": all(r["comm_fcs"] and r["sensing_fcs"]
                                       for r in refs["new"]),
                }
                timing_reference[cell] = refs
                print("TIMING_REFERENCE", cell,
                      json.dumps(summary["timing_reference"][cell],
                                 sort_keys=True), flush=True)
            write_json(out / "summary.json", summary)

            for label, cell in TIMING:
                case = case_path(cell, False)
                observations = {"old": [], "new": []}
                for round_index, variant in enumerate(("new", "old", "old", "new")):
                    dest = out / "timing" / label / f"r{round_index}_{variant}"
                    sparse = variant == "new"
                    rows = run_case(
                        args.new_build if sparse else args.old_build,
                        cell, False, dest, 64, 8, False, sparse,
                        logs / f"timing_{label}_{round_index}_{variant}.log",
                        env)
                    reference = {int(r["source_window"]): r
                                 for r in timing_reference[cell][variant]}
                    for row in rows:
                        expected = reference[int(row["source_window"])]
                        bad = [k for k in DECODE_KEYS
                               if row.get(k) != expected.get(k)]
                        if bad:
                            raise RuntimeError(
                                f"{label} {variant} window {row['window_id']} "
                                f"decode mismatch: {bad}")
                    observations[variant].extend(rows)
                result = {v: metrics(rows) for v, rows in observations.items()}
                ratios = {}
                for key in ("tx_event_generation", "channel_reconstruct",
                            "sic_total", "convolution", "cfo_rotation"):
                    old_mean = result["old"][key]["mean"]
                    new_mean = result["new"][key]["mean"]
                    ratios[key] = {
                        "new_over_old_mean": new_mean / old_mean
                        if old_mean else None,
                        "mean_delta_us": new_mean - old_mean,
                    }
                summary["timing"][label] = {
                    "cell": cell,
                    "old": result["old"],
                    "new": result["new"],
                    "ratios": ratios,
                    "observations_per_variant": 128,
                }
                write_json(out / "summary.json", summary)
                print("TIMING", label, json.dumps(ratios, sort_keys=True),
                      flush=True)

    # CSV exports are deliberately flat and easy to archive.
    with (out / "functional.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["cell", "variant", "window", "all_suppression_db",
                         "target_suppression_db", "committed", "comm_fcs",
                         "sensing_fcs"])
        for cell, entry in summary["functional"].items():
            for variant in ("old", "new"):
                effect_data = entry.get(f"{variant}_effect")
                if not effect_data:
                    continue
                for window in effect_data["per_window"]:
                    writer.writerow([cell, variant, window["window_id"],
                                     window["all_suppression_db"],
                                     window["target_suppression_db"],
                                     window["committed"], window["comm_fcs"],
                                     window["sensing_fcs"]])
    with (out / "timing.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["cell", "metric", "old_mean_us", "old_p95_us",
                         "new_mean_us", "new_p95_us", "new_over_old_mean"])
        for label, entry in summary["timing"].items():
            for metric in entry["old"]:
                old = entry["old"][metric]
                new = entry["new"][metric]
                writer.writerow([label, metric, old["mean"], old["p95"],
                                 new["mean"], new["p95"],
                                 new["mean"] / old["mean"]
                                 if old["mean"] else ""])
    summary["artifacts"] = {
        "summary": str(out / "summary.json"),
        "provenance": str(out / "provenance.json"),
        "functional_csv": str(out / "functional.csv"),
        "timing_csv": str(out / "timing.csv"),
        "logs": str(logs),
    }
    write_json(out / "summary.json", summary)
    print("DONE", out, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
