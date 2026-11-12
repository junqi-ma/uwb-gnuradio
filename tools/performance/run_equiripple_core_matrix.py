#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ABBA core-kernel matrix for the 65/48 equiripple experiment.

Task book: docs/performance/OpenCode_65_48等波纹FIR设计实验任务书_2026-11-11.md §6

Design
------
* A = Kaiser baseline taps, B = equiripple candidate taps.  Both run through
  the *same* binary, the same `--fixed-phase 1 --convert-cache 0` settings and
  the same kernel dispatch, so the only difference is the coefficient vector.
  Any speed difference is therefore attributable to the filter, not to a new
  kernel build.  (Task book §6 step 1.)
* Every run is serialised on /tmp/uwb-sic-benchmark.lock; no two runs, and no
  design or QA work, may overlap.
* ABBA ordering per group: A B B A.  The paired statistic is computed from the
  two A's and two B's *within a group*, then aggregated across groups.  Pooled
  medians are reported too but are never used to claim a paired effect.
* Raw per-run JSON is kept out of git; the aggregated CSV is committed.

Usage
-----
    python3 tools/performance/run_equiripple_core_matrix.py \
        --binary gr-uwb/build-sc16-equiripple/apps/benchmark_sc16_burst_core \
        --out docs/performance/data_65_48_equiripple
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

LOCK = "/tmp/uwb-sic-benchmark.lock"

# A = baseline, B = candidate
CONFIGS = [
    dict(name="realtime_56k", profile="realtime", length=56000, reps=10000),
    dict(name="realtime_4k", profile="realtime", length=4000, reps=100000),
    dict(name="quality_56k", profile="quality", length=56000, reps=10000),
    dict(name="quality_4k", profile="quality", length=4000, reps=100000),
]

VARIANTS = {
    "K": "testdata/resampler_65_48/taps_{profile}_minorder.txt",
    "EQ": "testdata/resampler_65_48_equiripple/taps_{profile}_eq_{tag}.txt",
}
TAGS = {"realtime": "hpad16", "quality": "hpad32"}
CONS_TAGS = {"realtime": "conservative", "quality": "conservative"}


def run_one(binary, taps, length, reps, warmup, out_json, extra, cpu=None):
    cmd = ([f"taskset", "-c", str(cpu)] if cpu is not None else []) + [
           binary, "--taps", taps, "--length", str(length),
           "--reps", str(reps), "--warmup", str(warmup),
           "--fixed-phase", "1", "--convert-cache", "0",
           "--json", out_json] + extra
    env = dict(os.environ)
    env.pop("LD_LIBRARY_PATH", None)
    t0 = time.time()
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if p.returncode != 0:
        raise SystemExit(f"run failed ({p.returncode}): {' '.join(cmd)}\n"
                         f"{p.stderr[-2000:]}")
    with open(out_json) as f:
        d = json.load(f)
    d["_wall_s"] = time.time() - t0
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--pairs", type=int, default=5)
    ap.add_argument("--warmup", type=int, default=500)
    ap.add_argument("--only", default=None,
                    help="comma separated config names")
    ap.add_argument("--variants", default="K,EQ")
    ap.add_argument("--length-override", type=int, default=None,
                    help="replace every config's window length (e.g. a full "
                         "256-SYNC + 127 B packet = 328137 native samples)")
    ap.add_argument("--reps-override", type=int, default=None)
    ap.add_argument("--cpu", type=int, default=None,
                    help="pin every run to this CPU (taskset -c)")
    ap.add_argument("--binary-map", default="",
                    help="per-variant binary override, e.g. K=pathA,EQ=pathB. "
                         "Lets one ABBA session compare two builds directly, "
                         "which is the only way to separate a kernel change "
                         "from cross-build drift.")
    ap.add_argument("--extra", default="",
                    help="extra args passed to every run, space separated")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    raw_dir = os.path.join(args.out, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    configs = CONFIGS
    if args.length_override:
        configs = [dict(c) for c in configs]
        for c in configs:
            c["length"] = args.length_override
            c["name"] = c["name"].split("_")[0] + f"_{args.length_override}"
            if args.reps_override:
                c["reps"] = args.reps_override
    if args.only:
        want = set(args.only.split(","))
        configs = [c for c in configs if c["name"] in want]
    variants = args.variants.split(",")
    binmap = {}
    for item in args.binary_map.split(","):
        if item.strip():
            k, v = item.split("=", 1)
            binmap[k.strip()] = v.strip()
    extra = args.extra.split() if args.extra.strip() else []

    lock_f = open(LOCK, "w")
    fcntl.flock(lock_f, fcntl.LOCK_EX)
    print(f"acquired {LOCK}")

    rows = []
    try:
        for cfg in configs:
            prof = cfg["profile"]
            taps = {}
            for v in variants:
                if v == "K":
                    taps[v] = VARIANTS["K"].format(profile=prof)
                elif v == "EQ":
                    taps[v] = VARIANTS["EQ"].format(profile=prof,
                                                    tag=TAGS[prof])
                elif v == "KG":
                    # Kaiser coefficients, second binary (see --binary-map)
                    taps[v] = VARIANTS["K"].format(profile=prof)
                elif v == "EQG":
                    # same coefficients as EQ; only the binary differs (used
                    # with --binary-map to isolate a kernel change)
                    taps[v] = VARIANTS["EQ"].format(profile=prof,
                                                    tag=TAGS[prof])
                elif v == "CONS":
                    taps[v] = VARIANTS["EQ"].format(profile=prof,
                                                    tag=CONS_TAGS[prof])
                else:
                    raise SystemExit(f"unknown variant {v}")
                if not os.path.exists(taps[v]):
                    raise SystemExit(f"missing taps: {taps[v]}")

            # ABBA with exactly two variants; for 3+ variants fall back to a
            # balanced rotation so every variant still gets equal exposure.
            if len(variants) == 2:
                order = [variants[0], variants[1], variants[1], variants[0]]
            else:
                order = list(variants) + list(reversed(variants))

            for pair in range(args.pairs):
                for slot, v in enumerate(order):
                    tag = f"{cfg['name']}__{v}__p{pair}__s{slot}"
                    jp = os.path.join(raw_dir, tag + ".json")
                    d = run_one(binmap.get(v, args.binary), taps[v], cfg["length"],
                                cfg["reps"], args.warmup, jp, extra,
                                cpu=args.cpu)
                    core = d["timing_us"]["core"]
                    rows.append(dict(
                        config=cfg["name"], profile=prof, variant=v,
                        pair=pair, slot=slot, length=cfg["length"],
                        reps=cfg["reps"], taps=d["taps_count"],
                        taps_fnv1a64=d["taps_fnv1a64"],
                        core_mean=core["mean"], core_p50=core["p50"],
                        core_p95=core["p95"], core_p99=core["p99"],
                        flush_p50=d["timing_us"]["flush"]["p50"],
                        wall_s=d["_wall_s"],
                        taps_path=taps[v]))
                    print(f"  {cfg['name']:14s} {v:5s} pair{pair} slot{slot} "
                          f"T={d['taps_count']:5d} "
                          f"p50={core['p50']:9.2f} mean={core['mean']:9.2f} "
                          f"p95={core['p95']:9.2f}")
    finally:
        fcntl.flock(lock_f, fcntl.LOCK_UN)
        lock_f.close()
        print(f"released {LOCK}")

    csvp = os.path.join(args.out, "core_runs.csv")
    with open(csvp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {csvp} ({len(rows)} runs)")

    # ---- paired summary ------------------------------------------------
    summary = []
    for cfg in configs:
        base = variants[0]
        for v in variants[1:]:
            grp = {}
            for r in rows:
                if r["config"] != cfg["name"]:
                    continue
                grp.setdefault((r["pair"], r["variant"]), []).append(r)
            deltas = []
            for pair in range(args.pairs):
                a = grp.get((pair, base), [])
                b = grp.get((pair, v), [])
                if not a or not b:
                    continue
                for metric in ("core_p50", "core_mean"):
                    pass
                a50 = statistics.median([x["core_p50"] for x in a])
                b50 = statistics.median([x["core_p50"] for x in b])
                am = statistics.median([x["core_mean"] for x in a])
                bm = statistics.median([x["core_mean"] for x in b])
                deltas.append(dict(pair=pair, a_p50=a50, b_p50=b50,
                                   ratio_p50=b50 / a50,
                                   a_mean=am, b_mean=bm,
                                   ratio_mean=bm / am))
            if not deltas:
                continue
            r50 = [d["ratio_p50"] for d in deltas]
            rm = [d["ratio_mean"] for d in deltas]
            pooled_a = statistics.median(
                [x["core_p50"] for x in rows
                 if x["config"] == cfg["name"] and x["variant"] == base])
            pooled_b = statistics.median(
                [x["core_p50"] for x in rows
                 if x["config"] == cfg["name"] and x["variant"] == v])
            summary.append(dict(
                config=cfg["name"], base=base, cand=v, pairs=len(deltas),
                paired_ratio_p50_median=statistics.median(r50),
                paired_ratio_p50_min=min(r50), paired_ratio_p50_max=max(r50),
                paired_delta_p50_pct=(statistics.median(r50) - 1) * 100,
                paired_ratio_mean_median=statistics.median(rm),
                paired_delta_mean_pct=(statistics.median(rm) - 1) * 100,
                pooled_p50_base=pooled_a, pooled_p50_cand=pooled_b,
                pooled_delta_p50_pct=(pooled_b / pooled_a - 1) * 100,
                taps_base=None, taps_cand=None))
    sp = os.path.join(args.out, "core_summary.csv")
    with open(sp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)
    print(f"wrote {sp}")
    for s in summary:
        print(f"  {s['config']:14s} {s['base']}->{s['cand']:5s} "
              f"paired p50 {s['paired_delta_p50_pct']:+6.2f}% "
              f"[{s['paired_ratio_p50_min']:.3f},{s['paired_ratio_p50_max']:.3f}] "
              f"mean {s['paired_delta_mean_pct']:+6.2f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
