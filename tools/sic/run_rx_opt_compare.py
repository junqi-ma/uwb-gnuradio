#!/usr/bin/env python3
"""Paired old/new RX-option comparison using the existing full-packet runner.

This driver does not duplicate any RX signal processing.  It runs the same
``benchmark_fullpacket_compare_fast_rx`` binary twice (legacy options vs the
explicit ``--rx-*`` experiment flags), binds every run to the build tree it
actually loaded, and compares:

* functional: per-source-window comm/sensing PSDU + FCS + status;
* timing: comm_demod mean/P95 and the existing stage/sub-stage diagnostics.

Formal runs take the exclusive ``/tmp/uwb-sic-benchmark.lock``.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
LOCK = Path("/tmp/uwb-sic-benchmark.lock")
DEFAULT_CELLS = ["sync_base", "payload_base", "payload_four_frac",
                 "payload_cfo_m50k", "payload_cfo_p50k", "payload_len16",
                 "payload_len127", "payload_snr8"]
TIMING_CELLS = ["sync_base", "payload_base", "payload_four_frac"]
NEW_ARGS = ["--rx-fast-sfd", "--rx-fast-cfo", "--rx-seed-refine-tail",
            "--rx-softchip-on-demand"]


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
                raise RuntimeError(f"runner loaded uwb library outside build: {candidate}")
            libraries.append({"path": str(candidate), "sha256": sha256(candidate)})
    if not libraries:
        raise RuntimeError("ldd found no uwb library")
    return {"ldd": completed.stdout, "libraries": libraries}


def load_rows(run: Path) -> list[dict]:
    path = run / "windows.jsonl"
    if not path.is_file():
        raise RuntimeError(f"missing {path}")
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def run_variant(runner: Path, case: Path, out: Path, env: dict, repeats: int,
                warmup: int, extra: list[str]) -> dict:
    if out.exists():
        raise RuntimeError(f"refusing to overwrite {out}")
    out.mkdir(parents=True)
    command = [str(runner), "--case", str(case), "--out", str(out),
               "--mode", "full", "--reconstruction", "full",
               "--repeat", str(repeats), "--warmup", str(warmup)] + extra
    with (out / "runner.log").open("w") as log:
        started = utc_now()
        completed = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                   stdout=log, stderr=subprocess.STDOUT)
    record = {
        "argv": command,
        "returncode": completed.returncode,
        "started_utc": started,
        "finished_utc": utc_now(),
        "repeats": repeats,
        "warmup": warmup,
        "extra_args": extra,
    }
    write_json(out / "command.json", record)
    if completed.returncode != 0:
        raise RuntimeError(f"runner failed ({completed.returncode}): {out}")
    rows = load_rows(out)
    if not rows:
        raise RuntimeError(f"runner produced no rows: {out}")
    return record


def p95(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(0.95 * (len(ordered) - 1)))))
    return ordered[index]


def summarize(rows: list[dict]) -> dict:
    accepted = [r for r in rows if not r.get("rejected")]
    comm = [float(r["times_us"]["comm_demod"]) for r in accepted]
    sfd = [float(r["comm_stage_us"]["sfd"]) for r in accepted]
    cfo = [float(r["comm_stage_us"]["cfo"]) for r in accepted]
    cir = [float(r["comm_stage_us"]["cir"]) for r in accepted]
    soft_fir = [float(r["rx_diag"].get("soft_fir_us", 0)) for r in accepted]
    extend = [float(r["rx_diag"].get("soft_extend_us", 0)) for r in accepted]
    post = [float(r["rx_diag"].get("post_us", 0)) for r in accepted]
    seed = [float(r["rx_diag"].get("seed_search_us", 0)) for r in accepted]
    rotate = [float(r["rx_diag"].get("cfo_rotate_us", 0)) for r in accepted]
    chips = [float(r["rx_diag"].get("softchip_final", 0)) for r in accepted]

    def stats(values: list[float]) -> dict:
        return {"mean": statistics.fmean(values) if values else 0.0,
                "p95": p95(values), "n": len(values)}

    return {
        "comm_demod": stats(comm),
        "stage_sfd": stats(sfd),
        "stage_cfo": stats(cfo),
        "stage_cir": stats(cir),
        "soft_fir": stats(soft_fir),
        "soft_extend": stats(extend),
        "post": stats(post),
        "seed_search": stats(seed),
        "cfo_rotate": stats(rotate),
        "chips": stats(chips),
        "softchip_fallbacks": sum(int(r["rx_diag"].get("softchip_fallbacks", 0))
                                  for r in accepted),
        "tail_fallbacks": sum(int(r["rx_diag"].get("tail_fallbacks", 0))
                              for r in accepted),
        "rot_block_rows": sum(1 for r in accepted
                              if int(r["rx_diag"].get("rotation_kernels", 0)) == 3),
    }


def functional_compare(old_rows: list[dict], new_rows: list[dict], cell: str) -> dict:
    fields = ("comm_fcs", "comm_status", "comm_psdu_hex", "sensing_fcs",
              "sensing_status", "sensing_psdu_hex", "committed", "reason")
    old_by_source = {int(r["source_window"]): r for r in old_rows}
    new_by_source = {int(r["source_window"]): r for r in new_rows}
    if set(old_by_source) != set(new_by_source):
        return {"cell": cell, "passed": False,
                "error": "source-window set mismatch"}
    mismatches = []
    for source in sorted(old_by_source):
        for field in fields:
            if old_by_source[source].get(field) != new_by_source[source].get(field):
                mismatches.append({"source_window": source, "field": field,
                                   "old": old_by_source[source].get(field),
                                   "new": new_by_source[source].get(field)})
    return {"cell": cell,
            "passed": not mismatches,
            "windows": len(old_by_source),
            "mismatches": mismatches,
            "old_comm_fcs": sum(bool(r["comm_fcs"]) for r in old_rows),
            "new_comm_fcs": sum(bool(r["comm_fcs"]) for r in new_rows),
            "old_sensing_fcs": sum(bool(r["sensing_fcs"]) for r in old_rows),
            "new_sensing_fcs": sum(bool(r["sensing_fcs"]) for r in new_rows)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build", default="build/sic-fullpacket-rxopt")
    parser.add_argument("--runner", default="apps/benchmark_fullpacket_compare_fast_rx")
    parser.add_argument("--inputs",
                        default="build/sic-fullpacket/results/inputs16")
    parser.add_argument("--out", required=True)
    parser.add_argument("--mode", choices=["functional", "timing", "both"],
                        default="both")
    parser.add_argument("--cells", nargs="+", default=None)
    parser.add_argument("--observations", type=int, default=128)
    parser.add_argument("--warmup", type=int, default=16)
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--new-args", nargs="+", default=NEW_ARGS)
    args = parser.parse_args()

    build = (ROOT / args.build).resolve()
    runner = (build / args.runner).resolve()
    inputs = (ROOT / args.inputs).resolve()
    out = (ROOT / args.out).resolve()
    if out.exists():
        raise RuntimeError(f"output exists; choose a new run: {out}")
    if not runner.is_file():
        raise RuntimeError(f"missing runner: {runner}")
    for cell in (args.cells or DEFAULT_CELLS):
        if not (inputs / cell / "public.json").is_file():
            raise RuntimeError(f"missing cell inputs: {inputs / cell}")
    out.mkdir(parents=True)
    env = clean_env()
    library = check_library(runner, build, env)
    flags_file = build / "apps/CMakeFiles" / (runner.name + ".dir") / "flags.make"
    status = git_output("status", "--porcelain=v1", "--untracked-files=normal")
    provenance = {
        "schema": "rx-opt-compare/1",
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
        "new_args": args.new_args,
        "lock_path": str(LOCK),
        "ld_library_path_removed": True,
        "full_wait_policy": "whole_public_context",
        "sources": {str(path.relative_to(ROOT)): sha256(path)
                    for path in [
                        ROOT / "tests/benchmarks/benchmark_fullpacket_compare.cc",
                        ROOT / "include/openuwb/runtime/fullpacket_rx_pipeline.h",
                        ROOT / "include/openuwb/phy/hrp/rx/demod_core.h",
                        ROOT / "include/openuwb/phy/hrp/rx/demod_result.h",
                        ROOT / "include/openuwb/phy/hrp/common/profile.h",
                        ROOT / "include/openuwb/phy/hrp/rx/rx_sync_opt.h",
                        ROOT / "include/openuwb/phy/hrp/rx/rx_cfo_opt.h",
                        ROOT / "include/openuwb/phy/hrp/rx/rx_softchip_opt.h",
                        Path(__file__).resolve(),
                    ] if path.is_file()},
    }
    write_json(out / "provenance.json", provenance)

    summary = {"schema": "rx-opt-compare/1", "created_utc": utc_now(),
               "mode": args.mode, "cells": {}, "functional": [],
               "timing": [], "provenance": str(out / "provenance.json")}

    cells = args.cells or DEFAULT_CELLS
    timing_cells = [c for c in cells if c in TIMING_CELLS]
    with LOCK.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.mode in ("functional", "both"):
            for cell in cells:
                case = inputs / cell
                windows = [p for p in (case / "windows").iterdir() if p.is_dir()]
                old_dir = out / "functional" / cell / "old"
                new_dir = out / "functional" / cell / "new"
                run_variant(runner, case, old_dir, env, len(windows), 0, [])
                run_variant(runner, case, new_dir, env, len(windows), 0,
                            args.new_args)
                result = functional_compare(load_rows(old_dir),
                                            load_rows(new_dir), cell)
                summary["functional"].append(result)
                print(f"functional {cell}: {'PASS' if result['passed'] else 'FAIL'}")
        if args.mode in ("timing", "both"):
            for cell in timing_cells:
                case = inputs / cell
                runs = []
                order = [("old", []), ("new", args.new_args),
                         ("new", args.new_args), ("old", [])]
                for index, (variant, extra) in enumerate(order):
                    run_dir = out / "timing" / cell / f"r{index + 1}_{variant}"
                    run_variant(runner, case, run_dir, env, args.observations,
                                args.warmup, extra)
                    rows = load_rows(run_dir)
                    runs.append({"round": index + 1, "variant": variant,
                                 "dir": str(run_dir), "summary": summarize(rows),
                                 "rows": rows})
                old_rows = [row for run in runs if run["variant"] == "old"
                            for row in run["rows"] if not row.get("rejected")]
                new_rows = [row for run in runs if run["variant"] == "new"
                            for row in run["rows"] if not row.get("rejected")]
                old_summary = summarize(old_rows)
                new_summary = summarize(new_rows)
                ratio = (new_summary["comm_demod"]["mean"] /
                         old_summary["comm_demod"]["mean"])
                p95_ratio = (new_summary["comm_demod"]["p95"] /
                             max(1e-9, old_summary["comm_demod"]["p95"]))
                result = {
                    "cell": cell,
                    "old": old_summary,
                    "new": new_summary,
                    "mean_ratio": ratio,
                    "p95_ratio": p95_ratio,
                    "target_mean_ratio_max": 0.80,
                    "regression_mean_ratio_max": 1.05,
                    "target_met": ratio <= 0.80 and p95_ratio <= 1.0,
                    "no_regression": ratio <= 1.05 and p95_ratio <= 1.05,
                    "rounds": [{k: run[k] for k in ("round", "variant", "summary")}
                               for run in runs],
                }
                summary["timing"].append(result)
                print(f"timing {cell}: mean {old_summary['comm_demod']['mean']:.1f}"
                      f" -> {new_summary['comm_demod']['mean']:.1f} us"
                      f" (ratio {ratio:.3f}), P95 ratio {p95_ratio:.3f}")
    write_json(out / "summary.json", summary)

    with (out / "timing.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell", "variant", "metric", "mean_us", "p95_us",
                         "n", "mean_ratio_vs_old", "p95_ratio_vs_old"])
        for item in summary["timing"]:
            for metric in ("comm_demod", "stage_sfd", "stage_cfo", "stage_cir",
                           "soft_fir", "soft_extend", "post", "seed_search",
                           "cfo_rotate", "chips"):
                old = item["old"][metric]
                new = item["new"][metric]
                mean_ratio = (new["mean"] / old["mean"]) if old["mean"] else 0.0
                p95_ratio = (new["p95"] / old["p95"]) if old["p95"] else 0.0
                writer.writerow([item["cell"], "old", metric, f"{old['mean']:.6f}",
                                 f"{old['p95']:.6f}", old["n"], "1.000000", "1.000000"])
                writer.writerow([item["cell"], "new", metric, f"{new['mean']:.6f}",
                                 f"{new['p95']:.6f}", new["n"],
                                 f"{mean_ratio:.6f}",
                                 f"{p95_ratio:.6f}"])
    with (out / "functional.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell", "passed", "windows", "mismatches",
                         "old_comm_fcs", "new_comm_fcs",
                         "old_sensing_fcs", "new_sensing_fcs"])
        for item in summary["functional"]:
            writer.writerow([item["cell"], item["passed"], item["windows"],
                             len(item["mismatches"]), item["old_comm_fcs"],
                             item["new_comm_fcs"], item["old_sensing_fcs"],
                             item["new_sensing_fcs"]])

    functional_ok = all(item["passed"] for item in summary["functional"])
    timing_ok = all(item["no_regression"] for item in summary["timing"])
    print(f"functional={'PASS' if functional_ok else 'FAIL'} "
          f"timing={'PASS' if timing_ok else 'FAIL'}")
    return 0 if functional_ok and timing_ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 - CLI boundary
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
