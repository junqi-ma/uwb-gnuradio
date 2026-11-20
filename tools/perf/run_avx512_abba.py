#!/usr/bin/env python3
"""ABBA driver for the AVX2 vs AVX-512 SC16 resampler comparison.

Plan: docs/performance/AVX512内核与编译期CPU选择开发规划_2026-11-16.md
Contract: docs/performance/AVX512_测量合同_2026-11-16.md

The driver runs the production per-window sequence of
benchmark_sc16_burst_core (reset + process_sc16 + flush) in ABBA order:

    round r: A B B A

where A = --kernel avx2 and B = --kernel avx512 (or the reverse for the
second half of the matrix).  Each invocation is a separate process, so the
measurement is not biased by in-process warm-up drift.  The per-round A and B
times are the mean of the two A / two B runs; the reported ratio is the median
of the per-round ratios.

It also checks that the FNV-1a checksum of the output is identical between
AVX2 and AVX-512 for every cell.  A checksum mismatch is a hard failure: the
kernels are supposed to be bit-identical, not merely close.

Usage:
    python3 tools/perf/run_avx512_abba.py \
        --bench build/avx512/apps/benchmark_sc16_burst_core \
        --out docs/performance/data_avx512_20261116/abba

The script takes an exclusive flock on /tmp/uwb-avx512-bench.lock so two
benchmark runs cannot overlap (the plan requires an exclusive benchmark).
"""

import argparse
import csv
import fcntl
import json
import os
import statistics
import subprocess
import sys
import time

TAPS = {
    "eq16": "testdata/resampler_65_48/taps_realtime_eq16.txt",
    "realtime": "testdata/resampler_65_48/taps_realtime_minorder.txt",
    "eq32": "testdata/resampler_65_48/taps_quality_eq32.txt",
    "quality": "testdata/resampler_65_48/taps_quality_minorder.txt",
}

# (rate, length, fixed_phase) cells.  56000 is the production packet window;
# 4096 is the short-window cell the plan also requires.
CELLS = []
for rate in ("65_48", "65_32"):
    for taps in TAPS:
        for length in (4096, 56000):
            for fp in (1, 0):
                CELLS.append((rate, taps, length, fp))


def run_once(bench, taps, rate, length, fixed_phase, kernel, reps, warmup,
             json_path):
    cmd = [
        bench,
        "--taps", taps,
        "--rate", rate,
        "--length", str(length),
        "--reps", str(reps),
        "--warmup", str(warmup),
        "--kernel", kernel,
        "--fixed-phase", str(fixed_phase),
        "--json", json_path,
    ]
    t0 = time.monotonic()
    proc = subprocess.run(cmd, capture_output=True, text=True)
    wall = time.monotonic() - t0
    if proc.returncode != 0:
        raise RuntimeError(
            "benchmark failed (%d): %s\n%s" %
            (proc.returncode, " ".join(cmd), proc.stderr[-2000:]))
    with open(json_path) as f:
        data = json.load(f)
    if not data.get("length_ok", False):
        raise RuntimeError("length_ok=false for %s" % " ".join(cmd))
    if not data.get("checksum_stable", False):
        raise RuntimeError("checksum_stable=false for %s" % " ".join(cmd))
    return data, wall


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--reps", type=int, default=50)
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--only", default="",
                    help="comma-separated subset: rate:taps:length:fp")
    args = ap.parse_args()

    bench = os.path.abspath(args.bench)
    outdir = os.path.abspath(args.out)
    os.makedirs(outdir, exist_ok=True)

    # Exclusive benchmark lock (plan: 基准独占).
    lock_path = "/tmp/uwb-avx512-bench.lock"
    lock = open(lock_path, "w")
    fcntl.flock(lock, fcntl.LOCK_EX)
    lock.write("pid=%d\n" % os.getpid())
    lock.flush()

    cells = CELLS
    if args.only:
        wanted = set(args.only.split(","))
        cells = [c for c in CELLS
                 if "%s:%s:%d:%d" % c in wanted]
        if not cells:
            raise SystemExit("--only matched no cell")

    rows = []
    summary = {
        "bench": bench,
        "bench_sha256": subprocess.check_output(
            ["sha256sum", bench], text=True).split()[0],
        "rounds": args.rounds,
        "reps": args.reps,
        "warmup": args.warmup,
        "cells": [],
    }

    for rate, taps_name, length, fp in cells:
        taps = TAPS[taps_name]
        cell_id = "%s_%s_%d_fp%d" % (rate, taps_name, length, fp)
        cell_dir = os.path.join(outdir, cell_id)
        os.makedirs(cell_dir, exist_ok=True)

        a_times, b_times = [], []
        a_totals, b_totals = [], []
        checksums = set()
        for r in range(args.rounds):
            # ABBA.  Alternate the order every round so a monotonic thermal
            # drift cannot favour one backend.
            order = ("avx2", "avx512", "avx512", "avx2")
            if r % 2 == 1:
                order = ("avx512", "avx2", "avx2", "avx512")
            for i, kernel in enumerate(order):
                jp = os.path.join(
                    cell_dir, "r%d_%s_%d.json" % (r, kernel, i))
                data, wall = run_once(
                    bench, taps, rate, length, fp, kernel,
                    args.reps, args.warmup, jp)
                core = data["timing_us"]["core"]
                total = data["timing_us"]["total"]
                checksums.add(data["checksum_fnv1a64"])
                if kernel == "avx2":
                    a_times.append(core["mean"])
                    a_totals.append(total["mean"])
                else:
                    b_times.append(core["mean"])
                    b_totals.append(total["mean"])
                rows.append({
                    "cell": cell_id,
                    "rate": rate,
                    "taps": taps_name,
                    "length": length,
                    "fixed_phase": fp,
                    "round": r,
                    "order_index": i,
                    "kernel": kernel,
                    "kernel_name": data["kernel"],
                    "sc16_variant": data["sc16_kernel_variant"],
                    "core_mean_us": core["mean"],
                    "core_p50_us": core["p50"],
                    "core_p95_us": core["p95"],
                    "core_p99_us": core["p99"],
                    "total_mean_us": total["mean"],
                    "checksum": data["checksum_fnv1a64"],
                    "wall_s": wall,
                })

        if len(checksums) != 1:
            raise RuntimeError(
                "checksum mismatch between AVX2 and AVX-512 in %s: %s" %
                (cell_id, sorted(checksums)))

        # Per-round A/B means: order is ABBA or BAAB, so average the two
        # entries of each kernel per round.
        per_round = []
        for r in range(args.rounds):
            rr = [x for x in rows if x["cell"] == cell_id and x["round"] == r]
            a = [x["core_mean_us"] for x in rr if x["kernel"] == "avx2"]
            b = [x["core_mean_us"] for x in rr if x["kernel"] == "avx512"]
            at = [x["total_mean_us"] for x in rr if x["kernel"] == "avx2"]
            bt = [x["total_mean_us"] for x in rr if x["kernel"] == "avx512"]
            per_round.append({
                "round": r,
                "avx2_core_mean_us": statistics.fmean(a),
                "avx512_core_mean_us": statistics.fmean(b),
                "avx2_total_mean_us": statistics.fmean(at),
                "avx512_total_mean_us": statistics.fmean(bt),
                "core_ratio_avx512_over_avx2":
                    statistics.fmean(b) / statistics.fmean(a),
                "total_ratio_avx512_over_avx2":
                    statistics.fmean(bt) / statistics.fmean(at),
            })

        core_ratios = [x["core_ratio_avx512_over_avx2"] for x in per_round]
        total_ratios = [x["total_ratio_avx512_over_avx2"] for x in per_round]
        entry = {
            "cell": cell_id,
            "rate": rate,
            "taps": taps_name,
            "length": length,
            "fixed_phase": fp,
            "checksum": next(iter(checksums)),
            "per_round": per_round,
            "core_ratio_median": statistics.median(core_ratios),
            "core_gain_pct_median": 100.0 * (1.0 - statistics.median(core_ratios)),
            "total_ratio_median": statistics.median(total_ratios),
            "total_gain_pct_median": 100.0 * (1.0 - statistics.median(total_ratios)),
            "core_ratio_all_same_direction":
                all(x < 1.0 for x in core_ratios) or all(x > 1.0 for x in core_ratios),
        }
        summary["cells"].append(entry)
        print("%-34s core %+6.2f%%  total %+6.2f%%  (ratio %.4f, %s)" % (
            cell_id,
            entry["core_gain_pct_median"],
            entry["total_gain_pct_median"],
            entry["core_ratio_median"],
            "same-dir" if entry["core_ratio_all_same_direction"] else "MIXED"))

    with open(os.path.join(outdir, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(outdir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("\nwrote %s" % os.path.join(outdir, "summary.json"))


if __name__ == "__main__":
    main()
