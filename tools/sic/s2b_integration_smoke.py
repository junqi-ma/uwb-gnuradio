#!/usr/bin/env python3
"""Run the S2-B integration smoke. This is not the formal 32-cell matrix.

The formal matrix is ``uwb_sim_sic_payload.py --matrix`` (32 cells, 24200
packets) and is checked by ``s2b_aggregate.py`` against ``matrix.json``.
This runner executes the 11 representative conditions from the 2026-10-18
integration guide, eight acceptance windows each, then calls ``verify_s2b.py``
on each cell. It writes ``integration_smoke.json`` with schema
``sic-s2b-integration-smoke/1`` and never writes a formal ``matrix.json``.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


SCHEMA = "sic-s2b-integration-smoke/1"
# Same child-process fields as run_matrix() in uwb_sim_sic_payload.py.
# windows=8 is the integration sample; it is not a new statistical study.
CELLS = [
    ("b0_clean_snr30", "b0", "center", "off", 0.0, 30.0, 0.0, "single"),
    ("b1_center_isr10_snr30", "b1", "center", "on", 10.0, 30.0, 0.0, "single"),
    ("b2_snr8.0_isr20", "b2", "center", "on", 20.0, 8.0, 0.0, "single"),
    ("b2_snr-3.5_isr20", "b2", "center", "on", 20.0, -3.5, 0.0, "single"),
    ("b3_pos_early_trans", "b3", "early", "on", 10.0, -3.5, 0.0, "single"),
    ("b3_pos_late_trans", "b3", "late", "on", 10.0, -3.5, 0.0, "single"),
    ("b3_cfo-50000_isr10_four", "b3", "center", "on", 10.0, 30.0, -50000.0, "four"),
    ("b3_cfo50000_isr10_four", "b3", "center", "on", 10.0, 30.0, 50000.0, "four"),
    ("b4_no_sensing", "b4", "center", "off", 10.0, 30.0, 0.0, "single"),
    ("b4_wrong_code11", "b4", "center", "wrong_code", 10.0, 30.0, 0.0, "single"),
    ("b4_no_overlap", "b4", "no_overlap", "on", 10.0, 30.0, 0.0, "single"),
]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def run_cell(repo: Path, build: Path, out: Path, spec, windows: int) -> dict:
    cell, phase, position, interferer, isr, snr, cfo, paths = spec
    cell_out = out / cell
    cell_out.mkdir(parents=True, exist_ok=False)
    cmd = [
        sys.executable, str(repo / "gr-uwb/apps/uwb_sim_sic_payload.py"),
        "--build-dir", str(build),
        "--output", str(cell_out),
        "--phase", phase,
        "--seed-set", "acceptance",
        "--windows", str(windows),
        "--archive-windows", str(windows),
        "--snr-db", str(snr),
        "--isr-db", str(isr),
        "--position", position,
        "--cfo-hz", str(cfo),
        "--comm-channel", "single",
        "--interferer-paths", paths,
        "--interferer", interferer,
    ]
    log = cell_out / "integration_smoke_driver.log"
    t0 = time.time()
    with log.open("w") as handle:
        rc = subprocess.call(cmd, stdout=handle, stderr=subprocess.STDOUT, cwd=repo)
    verify_cmd = [
        sys.executable, str(repo / "tools/sic/verify_s2b.py"),
        "--results", str(cell_out),
        "--repo", str(repo),
    ]
    vlog = cell_out / "integration_smoke_verify.log"
    with vlog.open("w") as handle:
        vrc = subprocess.call(verify_cmd, stdout=handle, stderr=subprocess.STDOUT, cwd=repo)
    return {
        "cell": cell,
        "formal_matrix_cell": False,
        "command": cmd,
        "driver_rc": rc,
        "verifier_rc": vrc,
        "wall_s": round(time.time() - t0, 3),
        "output": str(cell_out),
    }


def main(argv=None) -> int:
    repo = repo_root()
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--build-dir", type=Path,
                    default=repo / "gr-uwb/build-sic-payload")
    ap.add_argument("--output", type=Path,
                    default=repo / "gr-uwb/build-sic-payload/results/integration_20261018")
    ap.add_argument("--windows", type=int, default=8)
    args = ap.parse_args(argv)
    if args.output.exists() and any(args.output.iterdir()):
        print("refusing to write into a non-empty output: %s" % args.output, file=sys.stderr)
        return 2
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for spec in CELLS:
        print("== integration smoke %s" % spec[0], flush=True)
        rows.append(run_cell(repo, args.build_dir.resolve(), args.output, spec, args.windows))
        print("   driver=%s verify=%s" % (rows[-1]["driver_rc"], rows[-1]["verifier_rc"]),
              flush=True)
    failed = [row["cell"] for row in rows if row["driver_rc"] or row["verifier_rc"]]
    payload = {
        "schema": SCHEMA,
        "formal_matrix": False,
        "formal_matrix_completeness_checked": False,
        "purpose": "11-cell integration smoke, 8 acceptance windows, not the 32-cell matrix",
        "coverage_list": [spec[0] for spec in CELLS],
        "windows_per_cell": args.windows,
        "seed_set": "acceptance",
        "build_dir": str(args.build_dir.resolve()),
        "repo": str(repo),
        "cells": rows,
        "failed_cells": failed,
    }
    out_json = args.output / "integration_smoke.json"
    out_json.write_text(json.dumps(payload, indent=2) + "\n")
    print("smoke summary:", out_json, "failed:", failed or "none", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
