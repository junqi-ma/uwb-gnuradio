#!/usr/bin/env python3
"""ABBA comparison of two uwb_dw3000_live binaries on the low-SNR matrix.

Runs A,B,B,A per case so slow machine drift cancels, and reports the median of
the per-run means for each side.
"""
from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import platform
import json
import os
import re
import statistics
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_live import parse  # noqa: E402

EVAL = "."
FIX = {
    "+4": f"{EVAL}/f4.sc16",
    "-4": f"{EVAL}/fm4p0.sc16",
    "-10": f"{EVAL}/fm10.sc16",
    "-12": f"{EVAL}/fm12_200.sc16",
    "-14": f"{EVAL}/fm14.sc16",
    "noise": f"{EVAL}/noise_0p5s.sc16",
}
KEYV = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")
STAGES = ["st_timing", "st_cfo", "st_sfd", "st_cir", "st_nssfd", "st_phr",
          "st_payload", "st_total"]

CASES = [
    ("+4", "locked", 1, 256, []),
    ("-4", "locked", 1, 256, []),
    ("-10", "locked", 1, 256, []),
    ("-12", "locked", 1, 256, []),
    ("-12", "acquire", 1, 256, []),
    ("+4", "locked", 4, 24, []),
    ("-4", "locked", 4, 24, []),
    ("-10", "locked", 4, 24, []),
    ("-12", "locked", 4, 24, []),
    ("-12", "acquire", 4, 24, []),
    ("-14", "locked", 4, 24, ["--lock-fcs", "off", "--stop-after-sfd", "on"]),
    ("-14", "locked", 1, 256, ["--lock-fcs", "off", "--stop-after-sfd", "on"]),
    ("noise", "acquire", 4, 24, ["--tone", "on"]),
    ("noise", "acquire", 4, 24, ["--tone", "off"]),
]


def run(binary, fixture, workers, queue, extra, mode="summary", timeout=1800):
    cmd = [binary, "--replay", fixture, "--seconds", "0",
           "--period-ms", "1.0", "--mode", "continuous",
           "--workers", str(workers), "--queue", str(queue),
           "--cir", "full", "--fused", "on", "--taps", "eq16",
           "--tone", "off", "--print", mode] + extra
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                          check=True, env=env)


def summary_means(stdout):
    sums = [l for l in stdout.splitlines() if l.startswith(("SUMMARY ", "SUMMARY_PARTIAL "))]
    out = {}
    for k in ("compute_mean", "demod_mean", "compute_p95", "demod_p95"):
        vals = []
        for s in sums:
            m = re.search(rf"{k}=([\d.]+)", s)
            if m:
                vals.append(float(m.group(1)))
        if vals:
            out[k] = sum(vals) / len(vals)
    return out


def stage_means(stdout):
    acc = {k: [] for k in STAGES}
    for line in stdout.splitlines():
        if not line.startswith("PACKET "):
            continue
        d = dict(KEYV.findall(line))
        for k in STAGES:
            if k in d:
                try:
                    acc[k].append(float(d[k]))
                except ValueError:
                    pass
    return {k: (sum(v) / len(v) if v else None) for k, v in acc.items()}


def measure(binary, fixture, workers, queue, extra):
    ps = run(binary, fixture, workers, queue, extra, "summary")
    pp = run(binary, fixture, workers, queue, extra, "packets")
    d = parse(ps.stdout)
    if "FINAL" not in d or "COST_TOTAL" not in d:
        raise RuntimeError("replay output missing FINAL/COST_TOTAL")
    fin = d.get("FINAL", {})
    l = d.get("LOCK_STATS", {})
    c = d.get("COST_TOTAL", {})
    rec = summary_means(ps.stdout)
    rec.update({
        "command": ps.args,
        "start": d.get("START", {}),
        "fcs": fin.get("fcs"),
        "raw_drop": fin.get("raw_drop_samples"),
        "dropped_queue": l.get("dropped_queue"),
        "lock_entries": l.get("lock_entries"),
        "holdover": l.get("holdover_entries"),
        "reacquire": l.get("reacquire_entries"),
        "final_state": l.get("final_state"),
        "acq_seeds": c.get("acq_seeds"),
        "candidates": l.get("candidates"),
        "frontend_us": c.get("frontend_us"),
        "stages": stage_means(pp.stdout),
    })
    return rec


def med(recs, key):
    vals = [r[key] for r in recs if r.get(key) is not None]
    if not vals:
        return None
    if isinstance(vals[0], str):
        return vals[-1]
    return statistics.median(vals)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("a_binary")
    ap.add_argument("b_binary")
    ap.add_argument("out")
    ap.add_argument("--fixtures", required=True)
    ap.add_argument("--only", help="case name substring")
    args = ap.parse_args()
    a_bin, b_bin, out, only = args.a_binary, args.b_binary, args.out, args.only
    fixtures = {k: str(Path(args.fixtures) / Path(v).name) for k, v in FIX.items()}
    res = {}
    for snr, sync, workers, queue, extra in CASES:
        name = f"{snr}/{sync}/w{workers}"
        if snr == "noise":
            name += f"/tone-{extra[-1]}"
        if only and only not in name:
            continue
        extra = ["--sync", sync] + extra
        A, B = [], []
        for order in (("A", "B"), ("B", "A")):
            for side in order:
                binary = a_bin if side == "A" else b_bin
                rec = measure(binary, fixtures[snr], workers, queue, extra)
                (A if side == "A" else B).append(rec)
        row = {"runs": {"A": A, "B": B}}
        for side, recs in (("A", A), ("B", B)):
            row[side] = {k: med(recs, k) for k in
                         ("compute_mean", "demod_mean", "compute_p95",
                          "demod_p95", "fcs", "raw_drop", "dropped_queue",
                          "lock_entries", "holdover", "reacquire",
                          "final_state", "acq_seeds", "candidates",
                          "frontend_us")}
            row[side]["stages"] = {k: med([r["stages"] for r in recs], k)
                                   for k in STAGES}
        res[name] = row
        sa, sb = row["A"]["stages"], row["B"]["stages"]
        stg = " ".join(
            f"{k.replace('st_','')}={sa[k]:.0f}->{sb[k]:.0f}"
            for k in STAGES if sa.get(k) is not None and sb.get(k) is not None)
        print(f"{name:20s} comp={row['A']['compute_mean']}->{row['B']['compute_mean']} "
              f"dem={row['A']['demod_mean']}->{row['B']['demod_mean']} "
              f"fcs={row['A']['fcs']}->{row['B']['fcs']} | {stg}", flush=True)
    if not res:
        ap.error("no cases selected")
    def sha256(path):
        digest = hashlib.sha256()
        with open(path, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    used = {k: v for k, v in fixtures.items()
            if any(name.startswith(k + "/") for name in res)}
    metadata = {"platform": platform.platform(),
                "cpu": Path("/proc/cpuinfo").read_text() if Path("/proc/cpuinfo").exists() else platform.processor(),
                "binaries": {v: sha256(v) for v in (a_bin, b_bin)},
                "fixtures": {v: sha256(v) for v in used.values()},
                "statistics": "median of two run values per side; SUMMARY means are unweighted across console batches; stages from separate packet-print runs"}
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        json.dump({"metadata": metadata, "cases": res}, f, indent=1)
    print("wrote", out)


if __name__ == "__main__":
    main()
