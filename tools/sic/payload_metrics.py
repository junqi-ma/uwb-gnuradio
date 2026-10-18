#!/usr/bin/env python3
"""S2-B payload metrics: PER / Wilson CI / paired counts / suppression.

Pure NumPy, no GNU Radio.  Shared by the driver's summary and by the
independent verifier (``verify_s2b.py``), which re-derives every number from the
raw per-window JSONL and the archived arrays rather than trusting ``summary``.

Definitions (design contract §6):
  * PER_FCS  : undetected / no output / timeout / header fail / length error /
               FCS fail  -> error.  Denominator = planned sent packets.
  * PER_PSDU : decoded PSDU != the transmitted wire PSDU (which INCLUDES the
               single appended FCS) -> error.  Same denominator.
"""
from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np

PATHS = ("clean", "raw", "sic")


def wilson_ci(k: int, n: int, z: float = 1.96) -> Tuple[Optional[float],
                                                        Optional[float]]:
    """Wilson score interval for k successes in n trials (0/n is not 0 risk)."""
    if n <= 0:
        return (None, None)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def per_ci(errors: int, total: int, z: float = 1.96
           ) -> Tuple[Optional[float], Optional[float]]:
    """Wilson 95% interval for the PER itself (error proportion).

    B-R01: the PER interval is the Wilson interval of ``errors/total`` — NOT the
    success-rate interval.  ``wilson_ci(successes, total)`` is only the CI of the
    success rate and must be reported under a separate key
    (``success_wilson95``); using it as ``per_wilson95`` is the B-R01 defect.
    """
    if total <= 0:
        return (None, None)
    if errors < 0 or errors > total:
        return (None, None)
    return wilson_ci(int(errors), int(total), z)


def _per_summary(records: List[Dict], path: str, success_pred) -> Dict:
    n = 0
    ok = 0
    status_counts: Dict[str, int] = {}
    for r in records:
        d = (r.get("demod") or {}).get(path) or {}
        n += 1
        st = str(d.get("status", "missing"))
        status_counts[st] = status_counts.get(st, 0) + 1
        if success_pred(d):
            ok += 1
    errors = n - ok
    per_lo, per_hi = per_ci(errors, n)
    ok_lo, ok_hi = wilson_ci(ok, n)
    return {"errors": errors, "success": ok, "total": n,
            "per": ((errors) / n if n else None),
            "per_wilson95": [per_lo, per_hi],
            "success_wilson95": [ok_lo, ok_hi],
            "status_counts": status_counts}


def per_psdu(records: List[Dict], path: str) -> Dict:
    """PER_PSDU: decoded PSDU != transmitted wire PSDU -> error."""
    return _per_summary(records, path,
                        lambda d: d.get("psdu_match") is True)


def per_fcs(records: List[Dict], path: str) -> Dict:
    """PER_FCS: a packet succeeds only if status == success AND fcs_pass."""
    out = _per_summary(records, path,
                       lambda d: (str(d.get("status")) == "success"
                                  and d.get("fcs_pass") is True))
    out["fcs_pass"] = sum(1 for r in records
                          if ((r.get("demod") or {}).get(path) or {}
                              ).get("fcs_pass") is True)
    return out


def paired_counts(records: List[Dict]) -> Dict:
    rescued = damaged = 0
    for r in records:
        raw_ok = (r.get("demod", {}).get("raw") or {}).get("psdu_match") is True
        sic = (r.get("demod", {}).get("sic") or {})
        if not sic:
            continue
        sic_ok = sic.get("psdu_match") is True
        if (not raw_ok) and sic_ok:
            rescued += 1
        if raw_ok and (not sic_ok):
            damaged += 1
    return {"raw_fail_sic_ok": rescued, "raw_ok_sic_fail": damaged}


def suppression_db(i_true: np.ndarray, clean: np.ndarray,
                   out: np.ndarray, lo: int, hi: int,
                   eps: float = 1e-30) -> Optional[float]:
    """10log10(E_i / E_residual) over [lo,hi); residual = out - clean."""
    if out is None or out.size == 0:
        return None
    a = int(max(0, lo))
    b = int(min(hi, i_true.size, clean.size, out.size))
    if b <= a:
        return None
    e_i = float(np.sum(np.abs(i_true[a:b].astype(np.complex128)) ** 2))
    resid = (out[a:b].astype(np.complex128)
             - clean[a:b].astype(np.complex128))
    e_r = float(np.sum(np.abs(resid) ** 2))
    if e_i <= 0.0:
        return None
    if e_r <= eps:
        return float("inf")
    return 10.0 * math.log10(e_i / e_r)


def mask_outside_changed(y: np.ndarray, out: np.ndarray, sub_lo: Optional[int],
                         sub_hi: Optional[int]) -> int:
    if y is None or out is None or out.size != y.size:
        return -1
    if sub_hi is None or sub_lo is None or sub_hi <= sub_lo:
        return int(np.count_nonzero(out != y))
    outside = np.ones(y.size, dtype=bool)
    outside[int(sub_lo):int(sub_hi)] = False
    return int(np.count_nonzero(out[outside] != y[outside]))


def load_run(run_dir: str) -> Dict:
    out = {"dir": run_dir, "manifest": None, "summary": None, "index": None,
           "records": [], "recon": {}}
    for key, name in (("manifest", "run_manifest.json"),
                      ("summary", "summary.json"), ("index", "index.json")):
        p = os.path.join(run_dir, name)
        if os.path.exists(p):
            with open(p) as fh:
                out[key] = json.load(fh)
    wpath = os.path.join(run_dir, "windows.jsonl")
    if os.path.exists(wpath):
        with open(wpath) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    out["records"].append(json.loads(line))
    rdir = os.path.join(run_dir, "recon")
    if os.path.isdir(rdir):
        for fn in sorted(os.listdir(rdir)):
            if fn.endswith(".npz"):
                out["recon"][fn] = np.load(os.path.join(rdir, fn))
    return out


def median_suppression(records: List[Dict]) -> Optional[float]:
    vals = [r["suppression_db_overlap"] for r in records
            if r.get("suppression_db_overlap") is not None]
    return float(np.median(vals)) if vals else None
