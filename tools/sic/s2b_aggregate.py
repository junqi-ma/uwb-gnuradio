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
    """Read ONLY the per-window rows (no recon arrays) for the recompute."""
    recs = []
    wp = os.path.join(cell_dir, "windows.jsonl")
    if os.path.exists(wp):
        with open(wp) as fh:
            for line in fh:
                if line.strip():
                    recs.append(json.loads(line))
    return recs


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


def check_matrix(matrix, rows):
    """Cross-check the declared expected cells against the sub-run directories.

    B-R02/B-R03: not finding a directory is NOT a pass; missing/extra cells and
    any non-zero child rc are failures.
    """
    problems = []
    mj = os.path.join(matrix, "matrix.json")
    if not os.path.exists(mj):
        problems.append("matrix.json missing")
        return problems
    with open(mj) as fh:
        agg = json.load(fh)
    expected = set(agg.get("expected_cells") or [])
    cells_dir = os.path.join(matrix, "cells")
    found = set(d for d in (os.listdir(cells_dir)
                            if os.path.isdir(cells_dir) else [])
                if os.path.isdir(os.path.join(cells_dir, d)))
    missing = sorted(expected - found)
    extra = sorted(found - expected)
    if missing:
        problems.append("missing cells: %s" % missing)
    if extra:
        problems.append("unexpected cells: %s" % extra)
    failed_rc = [c.get("cell") for c in (agg.get("cells") or [])
                 if c.get("rc") != 0]
    if failed_rc:
        problems.append("child rc != 0: %s" % failed_rc)
    # B3 axis/count check
    b3_expected = agg.get("b3_expected_cells") or []
    found_b3 = [c for c in found if c.startswith("b3_")]
    if len(found_b3) != 12:
        problems.append("expected 12 B3 cells, found %d" % len(found_b3))
    want_counts = agg.get("counts") or {}
    for cid in found_b3:
        cdir = os.path.join(cells_dir, cid)
        recs = cell_rows(cdir)
        if recs and want_counts.get("b3") and len(recs) != want_counts["b3"]:
            problems.append("cell %s has %d windows != %d"
                            % (cid, len(recs), want_counts["b3"]))
    # official four-path hash binding present in every four cell
    for cid in found_b3:
        planp = os.path.join(cells_dir, cid, "plan.json")
        if not os.path.exists(planp):
            problems.append("cell %s missing plan.json" % cid)
            continue
    return problems


def compare_legacy_counts(matrix, rows):
    """B-R01 closure: the corrected-CI recompute must leave every path's error
    count and the rescue/damage counts IDENTICAL to the original summary."""
    mism = []
    for r in rows:
        sp = os.path.join(matrix, "cells", r["cell"], "summary.json")
        if not os.path.exists(sp):
            mism.append("%s: no summary" % r["cell"])
            continue
        with open(sp) as fh:
            old = json.load(fh)
        for path, d in r["paths"].items():
            op = (old.get("per") or {}).get(path) or {}
            if op.get("errors") != d["errors"] or op.get("total") != d["total"]:
                mism.append("%s/%s errors %s->%s total %s->%s"
                            % (r["cell"], path, op.get("errors"), d["errors"],
                               op.get("total"), d["total"]))
        opair = old.get("paired") or {}
        if (opair.get("raw_fail_sic_ok") != r["paired"]["raw_fail_sic_ok"]
                or opair.get("raw_ok_sic_fail") != r["paired"]["raw_ok_sic_fail"]):
            mism.append("%s paired %s vs %s"
                        % (r["cell"], opair, r["paired"]))
    return mism


def write_derived_from(matrix, outdir, rows, legacy):
    import hashlib
    mj = os.path.join(matrix, "matrix.json")
    mjsha = None
    if os.path.exists(mj):
        with open(mj, "rb") as fh:
            mjsha = hashlib.sha256(fh.read()).hexdigest()
    cell_hashes = {}
    for r in rows:
        wp = os.path.join(matrix, "cells", r["cell"], "windows.jsonl")
        if os.path.exists(wp):
            with open(wp, "rb") as fh:
                cell_hashes[r["cell"]] = hashlib.sha256(fh.read()).hexdigest()
    doc = {
        "derived_from": os.path.abspath(matrix),
        "original_matrix_json_sha256": mjsha,
        "per_cell_windows_jsonl_sha256": cell_hashes,
        "legacy_mode": bool(legacy),
        "reason": ("B-R01: PER confidence intervals for the original 21100 "
                   "windows were recomputed from the per-window records with the "
                   "error-proportion Wilson interval; the original run and its "
                   "figures are NOT modified and are marked superseded."),
        "note": ("error counts and rescue/damage are expected to be IDENTICAL to "
                 "the original summary; only the CI/figures change"),
    }
    with open(os.path.join(outdir, "derived_from.json"), "w") as fh:
        json.dump(doc, fh, indent=2, sort_keys=True)
    with open(os.path.join(outdir, "SUPERSEDED.md"), "w") as fh:
        fh.write("# Superseded\n\n"
                 "The PER confidence intervals and figures produced before "
                 "2026-10-18 (B-R01) used the success-rate Wilson interval and "
                 "are SUPERSEDED.  The corrected error-proportion intervals are "
                 "in this directory.\n\nOriginal figures superseded:\n"
                 "- `%s/figure_per_vs_snr.png`\n- `%s/figure_paired.png`\n"
                 % (os.path.abspath(matrix), os.path.abspath(matrix)))
    return doc


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--matrix", required=True)
    ap.add_argument("--out-dir", default=None,
                    help="write outputs here (default: the matrix dir)")
    ap.add_argument("--legacy", action="store_true",
                    help="recompute CI/figures from an OLD run without plan.json "
                         "(no strict per-cell verification)")
    ap.add_argument("--no-figures", action="store_true")
    args = ap.parse_args(argv)
    matrix = args.matrix
    outdir = args.out_dir or matrix
    os.makedirs(outdir, exist_ok=True)
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
        rows.append(summarize_cell(cdir, cid))
        if not args.legacy:
            if V.verify_run(cdir) != 0:
                verify_fail.append(cid)
    if args.legacy:
        write_derived_from(matrix, outdir, rows, legacy=True)
        mism = compare_legacy_counts(matrix, rows)
        print("legacy recompute: cells=%d count_mismatches=%s"
              % (len(rows), mism[:6]), flush=True)
    else:
        matrix_problems = check_matrix(matrix, rows)
        print("cells:", len(rows), "verify_failures:", verify_fail,
              "matrix_problems:", matrix_problems, flush=True)
        mism = []
    # CSV
    csv_path = os.path.join(outdir, "matrix_cells.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["cell", "path", "errors", "total", "per",
                    "per_ci_lo", "per_ci_hi", "success_ci_lo", "success_ci_hi",
                    "per_fcs_errors", "median_suppression_db",
                    "raw_fail_sic_ok", "raw_ok_sic_fail"])
        for r in rows:
            for path, d in r["paths"].items():
                ci = d["per_wilson95"]
                sc = d.get("success_wilson95") or [None, None]
                w.writerow([r["cell"], path, d["errors"], d["total"],
                            d["per"], ci[0], ci[1], sc[0], sc[1],
                            d.get("per_fcs_errors"),
                            r["median_suppression_db"],
                            r["paired"]["raw_fail_sic_ok"],
                            r["paired"]["raw_ok_sic_fail"]])
    with open(os.path.join(outdir, "matrix_recomputed.json"), "w") as fh:
        json.dump({"cells": rows, "verify_failures": verify_fail,
                   "legacy": bool(args.legacy),
                   "count_mismatches": mism}, fh, indent=2, sort_keys=True)
    print("wrote", csv_path, flush=True)
    if not args.no_figures:
        _figures(outdir, rows)
    if args.legacy:
        return 7 if mism else 0
    return 6 if (verify_fail or matrix_problems) else 0


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
