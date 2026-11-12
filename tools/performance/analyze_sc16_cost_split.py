#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-window cost split: resampler vs SIC vs sensing-CIR (and the demod tail).

The pipeline records four disjoint worker timers per window
(`resample_us`, `sic_us`, `sensing_us`, and `e2e_us` around all of them), so
the split can be read directly instead of inferred.

`queue_us` is deliberately excluded from the verdict: file replay is unpaced,
so windows are offered far faster than they can be served and the queue wait
is an artefact of the replay harness, not a property of the signal chain.
What matters for the bottleneck question is the *service time* per window.

Usage
-----
    python3 tools/performance/analyze_sc16_cost_split.py \
        --binary gr-uwb/build-sc16-r1r2/apps/uwb_uhd_sc16_pipeline_runner \
        --out docs/performance/data_sc16_cost_split
"""
import argparse
import csv
import json
import os
import statistics
import subprocess
import sys

FIXTURE = "testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin"
PROFILES = ["realtime_minorder", "realtime_eq16",
            "quality_minorder", "quality_eq32"]
CONSUMERS = ["sic", "sensing", "both"]
FIELDS = ["resample_us", "sic_us", "sensing_us", "queue_us", "e2e_us"]


def run(binary, profile, consumer, args, env):
    out = os.path.join(args.out, "raw",
                       f"{profile}__{consumer}.json")
    cmd = [binary, "--file", FIXTURE,
           "--native-rate", "737280000", "--work-rate", "998400000",
           "--benchmark", "--dsp-workers", "1",
           "--pool-size", str(max(8, args.windows)),
           "--t0", str(args.t0),
           "--period", str(args.period), "--capture", str(args.capture),
           "--sic-comm-code", "9", "--sic-policy", args.sic_policy,
           "--max-windows", str(args.windows),
           "--consumer", consumer,
           "--resampler-profile", profile,
           "--json-out", out]
    if args.cpu is not None:
        cmd = ["taskset", "-c", str(args.cpu)] + cmd
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if p.returncode != 0:
        raise SystemExit(f"run failed: {p.stderr[-1200:]}")
    return json.load(open(out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--out", required=True)
    # This fixture tiles the code-9 native packet (140982 samples). Start at
    # its second packet so the first window has a complete pre-guard.
    ap.add_argument("--period", type=int, default=140982)
    ap.add_argument("--t0", type=int, default=140982)
    ap.add_argument("--sic-policy", choices=["preamble", "fullpacket", "auto"],
                    default="preamble")
    ap.add_argument("--capture", type=int, default=56000)
    ap.add_argument("--windows", type=int, default=30)
    ap.add_argument("--cpu", type=int, default=6)
    args = ap.parse_args()

    os.makedirs(os.path.join(args.out, "raw"), exist_ok=True)
    env = dict(os.environ)
    env.pop("LD_LIBRARY_PATH", None)

    rows = []
    for prof in PROFILES:
        for cons in CONSUMERS:
            d = run(args.binary, prof, cons, args, env)
            ws = [w for w in d["windows"] if w["state"] == "Completed"]
            if not ws:
                print(f"  {prof:18s} {cons:8s} no completed windows")
                continue
            rec = {"profile": prof, "consumer": cons, "windows": len(ws)}
            for f in FIELDS:
                vals = [w["timing"][f] for w in ws]
                rec[f + "_p50"] = statistics.median(vals)
                rec[f + "_mean"] = statistics.fmean(vals)
            work = rec["resample_us_p50"] + rec["sic_us_p50"] + rec["sensing_us_p50"]
            rec["worker_us"] = work
            rec["resample_pct"] = 100 * rec["resample_us_p50"] / work if work else 0
            rec["sic_pct"] = 100 * rec["sic_us_p50"] / work if work else 0
            rec["sensing_pct"] = 100 * rec["sensing_us_p50"] / work if work else 0
            rec["msps"] = args.capture / work * 1e3 if work else 0
            rows.append(rec)
            print(f"  {prof:18s} {cons:8s} n={len(ws):3d} "
                  f"res={rec['resample_us_p50']:7.1f} "
                  f"sic={rec['sic_us_p50']:7.1f} "
                  f"sens={rec['sensing_us_p50']:7.1f} "
                  f"worker={work:7.1f} us "
                  f"({rec['resample_pct']:4.1f}%/{rec['sic_pct']:4.1f}%/{rec['sensing_pct']:4.1f}%)")

    p = os.path.join(args.out, "cost_split.csv")
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
