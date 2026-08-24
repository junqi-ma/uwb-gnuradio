#!/usr/bin/env python3
"""Aggregate SYNC polarity dump decode results (Python GR decoder).

Reads each dump_gain*/scheduled_dump.csv plus the clean baseline and
prints a markdown table of FCS rates and timing/SFD medians.  This is the
host-side counterpart to UWB_demodulation/run_analyze_sync_polarity.m and
does not require MATLAB.

Usage:

  python3 testdata/analyze_sync_polarity.py \\
    --root /mnt/f/UWB基带数据/qm35_sync_polarity_notched_20260824 \\
    --clean-dump /mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump

Add ``--uncoded-root /mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817``
to include the uncoded gain1/2/3 whole-file mixes decoded via
testdata/decode_scheduled_sc16_dump.py dumps (if present).
"""
from __future__ import annotations

import argparse
import csv
import json
import glob
import os
from pathlib import Path
import statistics

def load_csv(path: Path):
    rows = list(csv.DictReader(path.open(newline="")))
    return rows

def summarize(path: Path, label: str):
    rows = load_csv(path)
    n = len(rows)
    fcs = sum(1 for r in rows if r.get("qm35_fcs_pass") in ("1", 1, True, "true"))
    # timing and sfd may be empty strings
    def vals(key):
        vs = []
        for r in rows:
            v = r.get(key, "")
            if v not in ("", None):
                try:
                    vs.append(float(v))
                except: pass
        return vs
    t_vals = vals("qm35_timing_metric")
    s_vals = vals("qm35_sfd_metric")
    t_med = statistics.median(t_vals) if t_vals else float("nan")
    s_med = statistics.median(s_vals) if s_vals else float("nan")
    return {
        "dataset": label,
        "path": str(path),
        "n_windows": n,
        "fcs_pass": fcs,
        "fcs_rate": fcs / n if n else 0,
        "timing_median": t_med,
        "sfd_median": s_med,
    }

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, type=Path, help="Polarity dump root (contains dump_gain*)")
    ap.add_argument("--clean-dump", type=Path, default=Path("/mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump"))
    ap.add_argument("--uncoded-root", type=Path, default=None)
    args = ap.parse_args()

    records = []
    for gain in ("gain1", "gain2", "gain3"):
        csv_path = args.root / f"dump_{gain}" / "scheduled_dump.csv"
        if csv_path.is_file():
            records.append(summarize(csv_path, f"polarity_decoded_{gain}"))
        else:
            # try alternative naming dump_gainN vs gainN
            alt = args.root / f"dump_gain{gain[-1]}" / "scheduled_dump.csv"
            if alt.is_file():
                records.append(summarize(alt, f"polarity_decoded_{gain}"))

        # raw (should fail)
        raw_dump = args.root / f"dump_{gain}_raw" / "scheduled_dump.csv"
        if raw_dump.is_file():
            records.append(summarize(raw_dump, f"polarity_raw_{gain}"))

    if args.clean_dump and (args.clean_dump / "scheduled_dump.csv").is_file():
        records.append(summarize(args.clean_dump / "scheduled_dump.csv", "clean_baseline"))
    elif args.clean_dump and (args.clean_dump / "scheduled_dump_cpp.csv").is_file():
        records.append(summarize(args.clean_dump / "scheduled_dump_cpp.csv", "clean_baseline_cpp"))

    if args.uncoded_root:
        for gain in ("gain1", "gain2", "gain3"):
            # uncoded dumps may be under different layout; try to find csv
            cands = [
                args.uncoded_root / f"dump_{gain}" / "scheduled_dump.csv",
                args.uncoded_root / f"qm35_clean_plus_dw1000_{gain}_dump" / "scheduled_dump.csv",
            ]
            for c in cands:
                if c.is_file():
                    records.append(summarize(c, f"uncoded_{gain}"))
                    break

    if not records:
        print("no records found")
        return 1

    # Print markdown table
    print("| dataset | n | FCS | rate | timing_med | sfd_med |")
    print("|---|---|---:|---:|---:|---:|")
    for r in records:
        print(f"| {r['dataset']} | {r['n_windows']} | {r['fcs_pass']} | {r['fcs_rate']:.3f} | {r['timing_median']:.3f} | {r['sfd_median']:.3f} |")

    # Also write summary json/csv
    out_json = args.root / "sync_polarity_summary.json"
    out_json.write_text(json.dumps(records, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {out_json}")

    # Quick Go/No-Go check for first round (alternating only)
    # Expect decoded >= uncoded or >= 0.98 for gain1, and raw == 0
    # Find decoded gain1
    dec1 = next((r for r in records if r["dataset"] == "polarity_decoded_gain1"), None)
    raw1 = next((r for r in records if r["dataset"] == "polarity_raw_gain1"), None)
    if dec1 and dec1["fcs_rate"] < 0.9:
        print(f"WARN: decoded gain1 FCS {dec1['fcs_rate']:.3f} < 0.9")
    if raw1 and raw1["fcs_rate"] > 0.1:
        print(f"WARN: raw (no RX decode) unexpectedly high FCS {raw1['fcs_rate']:.3f}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
