#!/usr/bin/env python3
"""Aggregate the S2-B matrix: recompute PER/CI per cell from windows.jsonl,
verify every cell, and emit CSVs + (optional) figures.

Reads ``<matrix>/cells/*/`` produced by ``uwb_sim_sic_payload.py --matrix``.
Every number is recomputed from the raw per-window rows with
``payload_metrics.py``; the per-cell ``summary.json`` is only cross-checked, not
trusted.  Figures are written as PNG when matplotlib is available, otherwise the
figure data is written as JSON.

Usage:
  env -u LD_LIBRARY_PATH python3 tools/sic/s2b_aggregate.py --matrix <dir>
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import payload_metrics as PM  # noqa: E402
import verify_s2b as V       # noqa: E402


def cell_rows(cell_dir):
    return PM.load_run(cell_dir)["records"]


def summarize_cell(cell_dir, cell_id):
    recs = cell_rows(cell_dir)
    out = {"cell": cell_id, "windows": len(recs), "paths": {}}
    for path in ("clean", "raw", "sic"):
        if not any(path in (r.get("demod") or {}) for r in recs):
            continue
        out["paths"][path] = PM.per_psdu(recs, path)
    out["paired"] = PM.paired_counts(recs)
    out["median_suppression_db"] = PM.median_suppression(recs)
    out["false_commit_windows"] = sum(
        1 for r in recs if (r.get("sic") or {}).get("committed")
        and not (r.get("interferer_present")))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--matrix", required=True)
    ap.add_argument("--no-figures", action="store_true")
    args = ap.parse_args(argv)
    matrix = args.matrix
    cells_dir = os.path.join(matrix, "cells")
    if not os.path.isdir(cells_dir):
        print("no cells dir under %s" % matrix)
        return 2
    cell_ids = sorted(os.listdir(cells_dir))
    rows = []
    verify_fail = []
    for cid in cell_ids:
        cdir = os.path.join(cells_dir, cid)
        if not os.path.isdir(cdir):
            continue
        summ = summarize_cell(cdir, cid)
        rows.append(summ)
        # independent per-cell verification
        rc = V.verify_run(cdir)
        if rc != 0:
            verify_fail.append(cid)
    # CSV
    csv_path = os.path.join(matrix, "matrix_cells.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["cell", "path", "errors", "total", "per",
                    "ci_lo", "ci_hi", "median_suppression_db",
                    "raw_fail_sic_ok", "raw_ok_sic_fail"])
        for r in rows:
            for path, d in r["paths"].items():
                ci = d["per_wilson95"]
                w.writerow([r["cell"], path, d["errors"], d["total"],
                            d["per"], ci[0], ci[1],
                            r["median_suppression_db"],
                            r["paired"]["raw_fail_sic_ok"],
                            r["paired"]["raw_ok_sic_fail"]])
    with open(os.path.join(matrix, "matrix_recomputed.json"), "w") as fh:
        json.dump({"cells": rows, "verify_failures": verify_fail}, fh,
                  indent=2, sort_keys=True)
    print("cells:", len(rows), "verify_failures:", verify_fail, flush=True)
    print("wrote", csv_path, flush=True)
    if not args.no_figures:
        _figures(matrix, rows)
    return 0


def _figures(matrix, rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("matplotlib unavailable (%r); figure data only" % (exc,),
              flush=True)
        _figure_data(matrix, rows)
        return
    # PER vs SNR per ISR (B2 cells)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharey=True)
    isrs = [0.0, 10.0, 20.0]
    for ax, isr in zip(axes, isrs):
        for path, color in (("clean", "k"), ("raw", "r"), ("sic", "b")):
            xs, ys, lo, hi = [], [], [], []
            for r in rows:
                if not r["cell"].startswith("b2_"):
                    continue
                if "isr%.0f" % isr not in r["cell"]:
                    continue
                try:
                    snr = float(r["cell"].split("snr")[1].split("_")[0])
                except Exception:
                    continue
                p = r["paths"].get(path)
                if not p:
                    continue
                xs.append(snr)
                ys.append(p["per"])
                lo.append(p["per_wilson95"][0])
                hi.append(p["per_wilson95"][1])
            if xs:
                order = sorted(range(len(xs)), key=lambda i: xs[i])
                xs = [xs[i] for i in order]
                ys = [ys[i] for i in order]
                lo = [lo[i] for i in order]
                hi = [hi[i] for i in order]
                ax.plot(xs, ys, "-o", color=color, label=path)
                ax.fill_between(xs, lo, hi, color=color, alpha=0.15)
        ax.set_title("ISR = %.0f dB" % isr)
        ax.set_xlabel("sample SNR (dB)")
        ax.grid(True, alpha=0.3)
    axes[0].set_ylabel("PER_PSDU")
    axes[0].legend()
    fig.suptitle("S2-B PER vs SNR (Wilson 95% band); recomputed from windows.jsonl")
    fig.tight_layout()
    fig.savefig(os.path.join(matrix, "figure_per_vs_snr.png"), dpi=130)

    # rescued / damaged bars
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    names = [r["cell"] for r in rows]
    rescued = [r["paired"]["raw_fail_sic_ok"] for r in rows]
    damaged = [r["paired"]["raw_ok_sic_fail"] for r in rows]
    import numpy as np
    idx = np.arange(len(names))
    ax2.bar(idx - 0.2, rescued, 0.4, label="raw_fail_sic_ok", color="b")
    ax2.bar(idx + 0.2, damaged, 0.4, label="raw_ok_sic_fail", color="r")
    ax2.set_xticks(idx)
    ax2.set_xticklabels(names, rotation=90, fontsize=6)
    ax2.legend()
    ax2.set_title("S2-B paired outcomes per cell")
    fig2.tight_layout()
    fig2.savefig(os.path.join(matrix, "figure_paired.png"), dpi=130)
    print("figures written", flush=True)
    _figure_data(matrix, rows)


def _figure_data(matrix, rows):
    with open(os.path.join(matrix, "figure_data.json"), "w") as fh:
        json.dump(rows, fh, indent=2, sort_keys=True)


if __name__ == "__main__":
    raise SystemExit(main())
