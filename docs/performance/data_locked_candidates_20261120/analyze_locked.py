#!/usr/bin/env python3
"""Summarize the 2026-11-20 locked-mode live run.

Usage:
    python3 analyze_locked.py live-locked-2w-key.log [--json summary.json]

Separates the scheduled target slots from the extra autocorrelation
candidate windows.  `SUMMARY`/`FINAL` count every completed demod attempt,
so `FINAL fcs/completed` is NOT the target-packet PRR when candidates are
submitted.
"""
import argparse
import json
import re

FAIL_KEYS = ("TIMING_FAILED", "SFD_FAILED", "PHR_FAILED",
             "PAYLOAD_FAILED", "FCS_FAILED", "INCOMPLETE_WINDOW",
             "DROPPED_QUEUE", "QUEUE_FULL")


def kv(line):
    return dict(re.findall(r"(\w+)=([^\s]+)", line))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--json")
    args = ap.parse_args()

    batches, cost, lock, final = [], {}, {}, {}
    for line in open(args.log):
        if line.startswith("SUMMARY ") or line.startswith("SUMMARY_PARTIAL "):
            d = kv(line)
            batches.append({
                "completed": int(d["completed"]),
                "batch": int(d["batch"]),
                "batch_pct": float(d["batch_pct"]),
                "fail": {k: int(d[k]) for k in FAIL_KEYS if k in d},
                "compute_mean": float(d["compute_mean"]),
                "compute_p95": float(d["compute_p95"]),
            })
        elif line.startswith("COST_TOTAL "):
            cost = kv(line)
        elif line.startswith("LOCK_STATS "):
            lock = kv(line)
        elif line.startswith("FINAL "):
            final = kv(line)

    fail_totals = {}
    for b in batches:
        for k, v in b["fail"].items():
            fail_totals[k] = fail_totals.get(k, 0) + v
    fail_total = sum(fail_totals.values())

    expected = int(lock["expected_slots"])
    target_ok = int(lock["fcs_ok"])
    target_submitted = int(lock["submitted"])
    completed = int(final["completed"])
    fcs_total = int(final["fcs"].split("/")[0])
    cand_left = completed - target_submitted
    cand_ok = fcs_total - target_ok

    out = {
        "acq_seeds": int(cost["acq_seeds"]),
        "candidates": int(lock["candidates"]),
        "cand_target": int(lock["cand_target"]),
        "cand_unknown": int(lock["cand_unknown"]),
        "cand_admitted": int(lock["cand_admitted"]),
        "expected_slots": expected,
        "target_ok": target_ok,
        "target_submitted": target_submitted,
        "not_detected": int(lock["not_detected"]),
        "fcs_failed": int(lock["fcs_failed"]),
        "identity_rejects": int(lock["identity_rejects"]),
        "completed": completed,
        "fcs_total": fcs_total,
        "aggregate_pct": 100.0 * fcs_total / completed,
        "target_pct": 100.0 * target_ok / expected,
        "candidate_extra": cand_left,
        "candidate_ok": cand_ok,
        "candidate_fail": fail_total - (expected - target_ok),
        "fail_totals": fail_totals,
        "fail_total": fail_total,
        "batches": batches,
    }

    print(f"acq_seeds={out['acq_seeds']} candidates={out['candidates']} "
          f"target={out['cand_target']} unknown={out['cand_unknown']} "
          f"admitted={out['cand_admitted']}")
    print(f"target: {target_ok}/{expected} = {out['target_pct']:.2f}% "
          f"(not_detected={out['not_detected']} fcs_failed={out['fcs_failed']} "
          f"identity_rejects={out['identity_rejects']})")
    print(f"aggregate: {fcs_total}/{completed} = {out['aggregate_pct']:.2f}%")
    print(f"extra candidate windows≈{cand_left}, candidate_ok={cand_ok}, "
          f"candidate_fail≈{out['candidate_fail']}")
    print(f"failures={fail_total}: {fail_totals}")
    print(f"first batch={batches[0]['batch_pct']}% "
          f"best={max(b['batch_pct'] for b in batches):.1f}% "
          f"worst={min(b['batch_pct'] for b in batches):.1f}%")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
            f.write("\n")


if __name__ == "__main__":
    main()
