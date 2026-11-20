#!/usr/bin/env python3
"""Summarize uwb_dw3000_live --print packets logs for the 2026-11-20
DW3000 1 ms capture payload-failure analysis.

Usage:
    python3 analyze_failures.py logs/offline-w1.log.gz [more logs...]
    python3 analyze_failures.py logs/*.log.gz --json summary.json

Only PACKET lines are consumed.  A packet is counted as a success when
status=OK and fcs=1 and crc=1; target-path packets are reported separately
from acquire-path candidates so the startup candidates do not inflate the
failure rate.
"""
import argparse
import collections
import gzip
import json
import re
import sys


def open_text(path):
    if path.endswith(".gz"):
        return gzip.open(path, "rt", errors="replace")
    return open(path, "r", errors="replace")


def parse(path):
    rows = []
    with open_text(path) as f:
        for line in f:
            if line.startswith("PACKET "):
                rows.append(dict(re.findall(r"(\w+)=([^\s]+)", line)))
    return rows


def is_ok(r):
    return r.get("status") == "OK" and r.get("fcs") == "1" and r.get("crc") == "1"


def summarize(path):
    rows = parse(path)
    target = [r for r in rows if r.get("path") == "target"]
    acquire = [r for r in rows if r.get("path") != "target"]
    tfail = collections.Counter(r["status"] for r in target if not is_ok(r))
    afail = collections.Counter(r["status"] for r in acquire if not is_ok(r))
    slots = {r.get("slot"): r for r in target if r.get("slot")}
    return {
        "log": path,
        "submitted": len(rows),
        "submitted_ok": sum(1 for r in rows if is_ok(r)),
        "target": len(target),
        "target_ok": sum(1 for r in target if is_ok(r)),
        "target_fail": dict(tfail),
        "acquire": len(acquire),
        "acquire_ok": sum(1 for r in acquire if is_ok(r)),
        "acquire_fail": dict(afail),
        "focus": {
            s: {
                k: slots[s].get(k)
                for k in ("status", "spc_ppm", "grid_reject", "st_fused",
                          "pay_fused_calls", "pay_legacy_calls",
                          "tmetric", "sfdmetric", "cirmetric", "cfo_khz")
            }
            for s in ("9", "85", "253", "295", "401", "468")
            if s in slots
        },
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--json")
    args = ap.parse_args()
    results = [summarize(p) for p in args.logs]
    for r in results:
        pct = 100.0 * r["target_ok"] / r["target"] if r["target"] else 0.0
        print(f"{r['log']}: target {r['target_ok']}/{r['target']} "
              f"({pct:.2f}%) "
              f"submitted {r['submitted_ok']}/{r['submitted']} "
              f"target_fail={r['target_fail']} "
              f"acquire_fail={r['acquire_fail']}")
        for slot, f in r["focus"].items():
            print(f"  slot {slot}: {f['status']} spc_ppm={f['spc_ppm']} "
                  f"grid_reject={f['grid_reject']} st_fused={f['st_fused']} "
                  f"pay_legacy={f['pay_legacy_calls']} "
                  f"pay_fused={f['pay_fused_calls']}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2, sort_keys=True)
            f.write("\n")


if __name__ == "__main__":
    sys.exit(main())
