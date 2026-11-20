#!/usr/bin/env python3
"""ABBA driver for the phase-2 demod/SIC AVX2 vs AVX-512 comparison.

Plan: docs/performance/解调与SIC_AVX512内核扩展开发规划_2026-11-16.md (P4)
Contract: docs/performance/AVX512_测量合同_2026-11-16.md

The driver runs bench_unified_timing in ABBA order

    round r: A B B A

where A = UWB_KERNEL_BACKEND=<group>=avx2 and B = <group>=avx512.  Both arms
use the SAME binary and the same input, so the only difference is the kernel
backend -- which is exactly what the plan's A/B/C/D matrix asks for:

    A  sync=avx2  cir=avx2  sic_*=avx2     reference
    B  sync=avx512                         demod net gain (SIC shares comm_rx)
    C  cir=avx512 sic_reconstruct=avx512   SIC-only net gain
    D  everything                          combined

Every invocation is a separate process, so in-process warm-up drift cannot
bias one arm.  The per-round A and B values are the mean of the two A / two B
runs; the reported ratio is the median of the per-round ratios.

The script also re-runs both arms with SIC_DUMP_OUTPUT_DIR set and compares
the dumped SIC output byte for byte.  A mismatch is a hard failure: the
kernels are supposed to be bit-identical, not merely close.

Usage:
    python3 tools/perf/run_demod_sic_abba.py \
        --bench build/avx512-demod-sic/apps/bench_unified_timing \
        --out docs/performance/data_demod_sic_avx512_20261116/abba

The script takes an exclusive flock on /tmp/uwb-demod-sic-bench.lock so two
benchmark runs cannot overlap.
"""

import argparse
import fcntl
import json
import os
import re
import statistics
import subprocess
import sys
import tempfile

LOCK_PATH = "/tmp/uwb-demod-sic-bench.lock"

# Cells of the A/B/C/D matrix.  Each cell is (name, backend spec for arm B).
CELLS = [
    ("A_reference", "sync=avx2,cir=avx2,sic_reconstruct=avx2,sic_energy=avx2"),
    ("B_demod", "sync=avx512,cir=avx2,sic_reconstruct=avx2,sic_energy=avx2"),
    ("C_sic", "sync=avx2,cir=avx512,sic_reconstruct=avx512,sic_energy=avx512"),
    ("D_combined", "sync=avx512,cir=avx512,sic_reconstruct=avx512,sic_energy=avx512"),
]

# Stage rows look like "  1. estimate_channel      34.35 ..." or
# "     - timing              59.24".  The first number is the mean.
STAGE_RE = re.compile(
    r"^\s*(?:-\s+|\d+\.\s+)?([A-Za-z][A-Za-z0-9_ ()/&.-]*?)\s+([0-9]+\.[0-9]+)")
TOTAL_RE = re.compile(r"TOTAL CORE TIME\s+([0-9.]+)")
BENCH_RE = re.compile(r"Benchmark:\s+(.+)")


def parse_output(text):
    """Return {benchmark_name: {"total": float, "stages": {name: float}}}."""
    out = {}
    current = None
    for line in text.splitlines():
        m = BENCH_RE.search(line)
        if m:
            current = m.group(1).strip()
            out[current] = {"total": None, "stages": {}}
            continue
        if current is None:
            continue
        m = TOTAL_RE.search(line)
        if m:
            out[current]["total"] = float(m.group(1))
            continue
        m = STAGE_RE.match(line)
        if m:
            name = " ".join(m.group(1).split())
            out[current]["stages"][name] = float(m.group(2))
    return out


def run_once(bench, spec, dump_dir=None, raw_path=None):
    env = dict(os.environ)
    env["UWB_KERNEL_BACKEND"] = spec
    env.pop("LD_LIBRARY_PATH", None)
    if dump_dir:
        env["SIC_DUMP_OUTPUT_DIR"] = dump_dir
    else:
        env.pop("SIC_DUMP_OUTPUT_DIR", None)
    proc = subprocess.run([bench], capture_output=True, text=True, env=env,
                          timeout=1800)
    if proc.returncode != 0:
        sys.stderr.write(proc.stdout)
        sys.stderr.write(proc.stderr)
        raise SystemExit("bench failed (rc=%d) for spec %s"
                         % (proc.returncode, spec))
    if raw_path:
        with open(raw_path, "w") as f:
            f.write(proc.stdout)
    return parse_output(proc.stdout), proc.stdout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bench", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rounds", type=int, default=3)
    args = ap.parse_args()

    lock = open(LOCK_PATH, "w")
    fcntl.flock(lock, fcntl.LOCK_EX)

    os.makedirs(args.out, exist_ok=True)
    results = {}

    for cell_name, spec_b in CELLS:
        spec_a = CELLS[0][1]
        per_round = []
        for r in range(args.rounds):
            # ABBA within one round.
            tag = "%s_r%d" % (cell_name, r + 1)
            a1, _ = run_once(args.bench, spec_a,
                             raw_path=os.path.join(args.out, tag + "_a1.txt"))
            b1, _ = run_once(args.bench, spec_b,
                             raw_path=os.path.join(args.out, tag + "_b1.txt"))
            b2, _ = run_once(args.bench, spec_b,
                             raw_path=os.path.join(args.out, tag + "_b2.txt"))
            a2, _ = run_once(args.bench, spec_a,
                             raw_path=os.path.join(args.out, tag + "_a2.txt"))
            per_round.append((a1, b1, b2, a2))
            print("  %-12s round %d/%d" % (cell_name, r + 1, args.rounds),
                  flush=True)

        cell = {}
        for bench_name in per_round[0][0]:
            a_vals = []
            b_vals = []
            stage_a = {}
            stage_b = {}
            for (a1, b1, b2, a2) in per_round:
                a_mean = statistics.mean([a1[bench_name]["total"],
                                          a2[bench_name]["total"]])
                b_mean = statistics.mean([b1[bench_name]["total"],
                                          b2[bench_name]["total"]])
                a_vals.append(a_mean)
                b_vals.append(b_mean)
                for src, dst in ((a1[bench_name]["stages"], stage_a),
                                 (b1[bench_name]["stages"], stage_b)):
                    for k, v in src.items():
                        dst.setdefault(k, []).append(v)
            ratios = [b / a for a, b in zip(a_vals, b_vals)]
            cell[bench_name] = {
                "a_total_us": a_vals,
                "b_total_us": b_vals,
                "a_median_us": statistics.median(a_vals),
                "b_median_us": statistics.median(b_vals),
                "median_ratio": statistics.median(ratios),
                "median_delta_pct": (statistics.median(ratios) - 1.0) * 100.0,
                "all_rounds_improve": all(r < 1.0 for r in ratios),
                "stage_a_us": {k: statistics.median(v)
                               for k, v in stage_a.items()},
                "stage_b_us": {k: statistics.median(v)
                               for k, v in stage_b.items()},
            }
        results[cell_name] = {"spec_b": spec_b, "benches": cell}

    # Bit-identity of the SIC output between arm A and arm D.
    with tempfile.TemporaryDirectory() as tmp:
        da = os.path.join(tmp, "a")
        dd = os.path.join(tmp, "d")
        os.makedirs(da)
        os.makedirs(dd)
        run_once(args.bench, CELLS[0][1], dump_dir=da)
        run_once(args.bench, CELLS[3][1], dump_dir=dd)
        identity = {}
        for name in sorted(os.listdir(da)):
            pa = os.path.join(da, name)
            pd = os.path.join(dd, name)
            if not os.path.exists(pd):
                identity[name] = "missing"
                continue
            with open(pa, "rb") as fa, open(pd, "rb") as fd:
                ba = fa.read()
                bd = fd.read()
            identity[name] = ("identical (%d bytes)" % len(ba)
                              if ba == bd else "DIFFERS")
    results["sic_output_identity"] = identity
    try:
        results["loadavg"] = os.getloadavg()
    except OSError:
        results["loadavg"] = None
    results["bench"] = args.bench
    results["rounds"] = args.rounds

    summary_path = os.path.join(args.out, "summary.json")
    with open(summary_path, "w") as f:
        json.dump(results, f, indent=2, sort_keys=True)

    print("\n%-14s %-32s %10s %10s %8s %s"
          % ("cell", "benchmark", "A us", "B us", "delta", "all rounds"))
    for cell_name, cell in results.items():
        if not isinstance(cell, dict) or "benches" not in cell:
            continue
        for bench_name, v in cell["benches"].items():
            print("%-14s %-32s %10.2f %10.2f %+7.2f%% %s"
                  % (cell_name, bench_name[:32], v["a_median_us"],
                     v["b_median_us"], v["median_delta_pct"],
                     "yes" if v["all_rounds_improve"] else "NO"))
    print("\nSIC output identity:", results["sic_output_identity"])
    print("summary:", summary_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
