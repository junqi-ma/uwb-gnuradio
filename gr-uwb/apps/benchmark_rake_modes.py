#!/usr/bin/env python3
"""Repeatable Full/Top-4/Top-8 UWB demodulation performance comparison."""

import argparse
import re
import statistics
import subprocess
import sys
from pathlib import Path


MODES = (0, 8, 4)


def run(command):
    result = subprocess.run(command, text=True, capture_output=True)
    if result.returncode:
        sys.stderr.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise RuntimeError(f"command failed ({result.returncode}): {' '.join(command)}")
    return result.stdout


def number(output, pattern, name):
    match = re.search(pattern, output)
    if not match:
        raise RuntimeError(f"cannot parse {name} from benchmark output")
    return float(match.group(1))


def stage_once(binary, rake_k):
    output = run([binary, "unused", "demod-stage-profile",
                  "--rake-top-k", str(rake_k)])
    if "payload[ok]" not in output or "fcs_pass=1" not in output:
        raise RuntimeError(f"RAKE Top-{rake_k}: golden demodulation failed")
    return {
        "timing": number(output, r"timing=([0-9.]+)", "timing"),
        "cir": number(output, r"cir=([0-9.]+)", "cir"),
        "total": number(output, r"total=([0-9.]+)", "total"),
        "fir": number(output, r"fir=([0-9.]+)", "FIR"),
    }


def async_once(binary, rake_k, jobs, workers, queue):
    output = run([binary, "unused", "demod-async", "--repeat", str(jobs),
                  "--workers", str(workers), "--queue", str(queue),
                  "--rake-top-k", str(rake_k)])
    done = int(number(output, r"done=([0-9]+)", "completed jobs"))
    dropped = int(number(output, r"dropped=([0-9]+)", "dropped jobs"))
    if done != jobs or dropped:
        raise RuntimeError(
            f"RAKE Top-{rake_k}: done={done}/{jobs}, dropped={dropped}")
    return number(output, r"throughput\s*:\s*([0-9.]+) jobs/s", "throughput")


def robust_top4(binary, repetitions):
    output = run([binary, "unused", "demod-robust", "--rake-top-k", "4",
                  "--robust-reps", str(repetitions)])
    all_pass_rows = output.count("100.0%")
    collisions = len(re.findall(
        r"^\s+[0-9.]+\s+\d+\s+PASS\s+", output, re.MULTILINE))
    if all_pass_rows < 17 or "20/20 pass" not in output or collisions != 3:
        raise RuntimeError(
            "Top-4 robustness failed: expected all AWGN/CFO/multipath/"
            "collision cases to pass")


def med(values):
    return statistics.median(values)


def label(k):
    return "Full-38" if k == 0 else f"Top-{k}"


def main():
    parser = argparse.ArgumentParser(
        description="Compare Full-38, Top-8 X4 and Top-4 X4 demodulation")
    parser.add_argument("--binary", default="gr-uwb/build/apps/benchmark_detector")
    parser.add_argument("--stage-runs", type=int, default=7)
    parser.add_argument("--async-runs", type=int, default=3)
    parser.add_argument("--jobs", type=int, default=3000)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--queue", type=int, default=4096)
    parser.add_argument("--robust-reps", type=int, default=5,
                        help="AWGN/CFO repetitions for Top-4; 0 skips")
    args = parser.parse_args()

    binary = str(Path(args.binary))
    if args.stage_runs < 1 or args.async_runs < 1 or args.jobs < 1:
        parser.error("run and job counts must be positive")
    if args.queue < args.jobs:
        parser.error("--queue must be >= --jobs for the unpaced async test")
    stage = {k: [] for k in MODES}
    peak = {k: [] for k in MODES}

    # Rotate order on every pass to reduce temperature/frequency ordering bias.
    for iteration in range(args.stage_runs):
        order = MODES[iteration % len(MODES):] + MODES[:iteration % len(MODES)]
        for k in order:
            stage[k].append(stage_once(binary, k))
    for iteration in range(args.async_runs):
        order = MODES[iteration % len(MODES):] + MODES[:iteration % len(MODES)]
        for k in order:
            peak[k].append(async_once(
                binary, k, args.jobs, args.workers, args.queue))
    if args.robust_reps > 0:
        robust_top4(binary, args.robust_reps)

    full_total = med([x["total"] for x in stage[0]])
    full_fir = med([x["fir"] for x in stage[0]])
    full_peak = med(peak[0])
    print(f"stage_runs={args.stage_runs} async_runs={args.async_runs} "
          f"jobs={args.jobs} workers={args.workers} queue={args.queue}")
    print("mode       timing_us  fir_us  cir_us  total_us  total_speedup  peak_pkt_s  peak_speedup")
    for k in MODES:
        timing = med([x["timing"] for x in stage[k]])
        fir = med([x["fir"] for x in stage[k]])
        cir = med([x["cir"] for x in stage[k]])
        total = med([x["total"] for x in stage[k]])
        throughput = med(peak[k])
        print(f"{label(k):<10} {timing:9.1f} {fir:7.1f} {cir:7.1f} "
              f"{total:9.1f} {full_total / total:13.3f} "
              f"{throughput:11.1f} {throughput / full_peak:12.3f}")

    if med([x["fir"] for x in stage[4]]) >= full_fir:
        print("FAIL: Top-4 X4 FIR is not faster than Full-38", file=sys.stderr)
        return 3
    if min(med(peak[k]) for k in MODES) < 300.0:
        print("FAIL: at least one mode is below 300 pkt/s", file=sys.stderr)
        return 4
    robust_text = "robustness" if args.robust_reps > 0 else "robustness skipped"
    print("PASS: golden FCS, Top-4 FIR speedup, 300 pkt/s peak capacity, "
          f"and {robust_text}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
