#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G2 sweep: independent float64 reference vs the C++ core, for every
candidate tap set, input mode and length.

Covers the T-dependent surface that a frequency-response check cannot see:
polyphase arm split, history length, flush, and the output-length contract.

Usage
-----
    python3 tools/performance/run_equiripple_g2.py \
        --binary build/sc16-equiripple/apps/benchmark_sc16_burst_core \
        --out docs/performance/data_65_48_equiripple_g2
"""
import argparse
import csv
import json
import os
import subprocess
import sys

TAPS = {
    "kaiser_realtime": "testdata/resampler_65_48/taps_realtime_minorder.txt",
    "kaiser_quality": "testdata/resampler_65_48/taps_quality_minorder.txt",
    "eq_realtime_hpad16": "testdata/resampler_65_48_equiripple/taps_realtime_eq_hpad16.txt",
    "eq_quality_hpad32": "testdata/resampler_65_48_equiripple/taps_quality_eq_hpad32.txt",
    "eq_realtime_cons": "testdata/resampler_65_48_equiripple/taps_realtime_eq_conservative.txt",
    "eq_quality_cons": "testdata/resampler_65_48_equiripple/taps_quality_eq_conservative.txt",
}
MODES = ["random", "rails", "impulse", "zero"]
LENGTHS = [1, 2, 64, 65, 66, 1000, 4000, 56000]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=0x1234567)
    ap.add_argument("--only", default=None)
    ap.add_argument("--lengths", default=None)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    tmp = os.path.join(args.out, "raw")
    os.makedirs(tmp, exist_ok=True)

    taps = TAPS
    if args.only:
        want = set(args.only.split(","))
        taps = {k: v for k, v in TAPS.items() if k in want}
    lengths = ([int(x) for x in args.lengths.split(",")] if args.lengths
               else LENGTHS)

    env = dict(os.environ)
    env.pop("LD_LIBRARY_PATH", None)
    rows = []
    for name, tpath in taps.items():
        if not os.path.exists(tpath):
            print(f"  skip {name}: missing {tpath}")
            continue
        for mode in MODES:
            for n in lengths:
                tag = f"{name}__{mode}__{n}"
                iq = os.path.join(tmp, tag + ".bin")
                js = os.path.join(tmp, tag + ".json")
                cmd = [args.binary, "--taps", tpath, "--length", str(n),
                       "--reps", "2", "--warmup", "1", "--fixed-phase", "1",
                       "--convert-cache", "0", "--input", mode,
                       "--seed", str(args.seed), "--dump-iq", iq,
                       "--json", js]
                p = subprocess.run(cmd, capture_output=True, text=True, env=env)
                if p.returncode != 0:
                    rows.append(dict(taps=name, mode=mode, length=n,
                                     status="core_failed",
                                     detail=p.stderr.strip()[-160:]))
                    print(f"  {name:22s} {mode:8s} N={n:6d} CORE FAILED")
                    continue
                meta = json.load(open(js))
                vcmd = [sys.executable,
                        "testdata/verify_resampler_core_reference.py",
                        "--taps", tpath, "--iq", iq, "--length", str(n),
                        "--input", mode, "--seed", str(args.seed),
                        "--json", js + ".ref"]
                v = subprocess.run(vcmd, capture_output=True, text=True)
                if v.returncode != 0:
                    rows.append(dict(taps=name, mode=mode, length=n,
                                     status="reference_failed",
                                     detail=(v.stdout + v.stderr).strip()[-200:]))
                    print(f"  {name:22s} {mode:8s} N={n:6d} REF FAILED")
                    continue
                ref = json.load(open(js + ".ref"))
                rows.append(dict(
                    taps=name, mode=mode, length=n, status="ok",
                    taps_count=meta["taps_count"],
                    arm_length=meta["arm_length"],
                    arm_length_pad=meta["arm_length_pad"],
                    expected_out=ref["expected_out"],
                    dumped_out=ref["dumped_out"],
                    length_ok=ref["length_ok"],
                    max_abs_err=ref["max_abs_err"],
                    rel_l2=ref["rel_l2"],
                    rel_max=ref["rel_max"],
                    ref_peak=ref["ref_peak"],
                    checksum=meta.get("checksum_fnv1a64"),
                    checksum_stable=meta.get("checksum_stable")))
                print(f"  {name:22s} {mode:8s} N={n:6d} "
                      f"T={meta['taps_count']:5d} out={ref['expected_out']:6d} "
                      f"relL2={ref['rel_l2']:.2e} max={ref['max_abs_err']:.2e}")

    csvp = os.path.join(args.out, "g2_reference.csv")
    fields = ["taps", "mode", "length", "status", "taps_count", "arm_length",
              "arm_length_pad", "expected_out", "dumped_out", "length_ok",
              "max_abs_err", "rel_l2", "rel_max", "ref_peak", "checksum",
              "checksum_stable", "detail"]
    with open(csvp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {csvp} ({len(rows)} cases)")

    ok = [r for r in rows if r["status"] == "ok"]
    bad = [r for r in rows if r["status"] != "ok"]
    len_bad = [r for r in ok if not r["length_ok"]]
    worst = max(ok, key=lambda r: r["rel_l2"]) if ok else None
    print(f"ok={len(ok)} failed={len(bad)} length_mismatch={len(len_bad)}")
    if worst:
        print(f"worst rel L2 {worst['rel_l2']:.3e} "
              f"({worst['taps']} {worst['mode']} N={worst['length']})")
    return 0 if not bad and not len_bad else 1


if __name__ == "__main__":
    sys.exit(main())
