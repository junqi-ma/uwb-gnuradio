#!/usr/bin/env python3
"""Independent verifier for S2-A (accumulation-after-SIC CIR evaluation).

It does NOT import the evaluation driver (``gr-uwb/apps/uwb_sim_sic_accum.py``)
or ``tools/sic/accum_metrics.py``: the small amount of algebra it needs is
re-implemented here (sparse matched filter, accumulation, residual/coherence /
target metrics) so the check is not a self-consistency replay of the same code.

What it proves
--------------
1. The fixed operator archived in the run is the one the public modulator
   implies: the code is recovered independently from the archived shaped SYNC
   symbol + pulse taps (least squares), rebuilt into the bare sampled code and
   compared with the archived operator vector.
2. Every archived window's ``H_clean/H_raw/H_sic`` is reproduced from the
   archived IQ with the verifier's own matched filter.
3. Every archived window's accumulation table (``C_CIR``, NMSE, ``E_raw``,
   ``E_sic``, ``G``) and target / coherence metrics are reproduced from the
   archived CIR arrays and compared to ``windows.jsonl``.
4. The aggregate ``summary.json`` / ``cells_summary.csv`` are recomputed from
   ``windows.jsonl`` (denominators included: N/A / +-Inf / failed windows stay
   counted).
5. Provenance / discipline: library inside this worktree, seed set membership
   and uniqueness, ``uses_truth == False`` on every online window, frozen model
   never re-fit (``apply_refit_count == 0``), archive policy, ``not_run``
   declarations, receiver-layer labelling.

Usage
-----
    python3 tools/sic/verify_s2a.py --run <run_dir> [--verbose]
    python3 tools/sic/verify_s2a.py --self-test
    python3 tools/sic/verify_s2a.py --reject-qa --run <run_dir>

Exit code 0 = all checks pass. ``--reject-qa`` exits 0 only when EVERY
mutation is rejected.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

RESULTS = []


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def record(name, ok, detail=""):
    RESULTS.append((bool(ok), name, detail))
    return bool(ok)


def fail(name, detail=""):
    return record(name, False, detail)


def ok(name, cond, detail=""):
    return record(name, bool(cond), detail)


# ---------------------------------------------------------------------------
# IO
# ---------------------------------------------------------------------------
def load_json(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def load_jsonl(path):
    out = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def sha256_file(path):
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def sha256_c64(a):
    import hashlib
    return hashlib.sha256(np.ascontiguousarray(a, dtype=np.complex64)
                          .tobytes()).hexdigest()


# ---------------------------------------------------------------------------
# Independent primitives
# ---------------------------------------------------------------------------
def build_sampled_code(chips, sps=1016, spread=4, spp=2):
    """chips -> spread code -> samples/pulse (buildUwbReference.m order)."""
    chips = np.asarray(chips, dtype=np.float64).ravel()
    spread_c = np.zeros(chips.size * spread, dtype=np.float64)
    spread_c[0::spread] = chips
    sampled = np.zeros(spread_c.size * spp, dtype=np.float64)
    sampled[0::spp] = spread_c
    if sampled.size != int(sps):
        raise ValueError("sampled code length %d != sps %d"
                         % (sampled.size, sps))
    return sampled.astype(np.complex128)


def recover_chips_ls(template, taps, sps, code_len, stride):
    """Independent ternary-code recovery (dense least squares)."""
    A = np.zeros((int(sps), int(code_len)), dtype=np.float64)
    for c in range(int(code_len)):
        off = c * int(stride)
        k = min(int(taps.size), int(sps) - off)
        if k > 0:
            A[off:off + k, c] = taps[:k]
    y = np.asarray(template[:sps], dtype=np.complex128).real.astype(np.float64)
    coef, _res, rank, _sv = np.linalg.lstsq(A, y, rcond=None)
    scale = float(np.max(np.abs(coef)))
    if scale <= 0.0:
        raise ValueError("degenerate sync symbol")
    norm = coef / scale
    chips = np.rint(norm).astype(np.int8)
    resid = float(np.linalg.norm(A @ coef - y)) / max(1e-30,
                                                      float(np.linalg.norm(y)))
    return chips, {"rank": int(rank), "resid_rel": resid,
                   "max_int_dev": float(np.max(np.abs(norm - chips))),
                   "nonzero": int(np.count_nonzero(chips)),
                   "energy": int(np.sum(chips.astype(np.int64) ** 2))}


def matched_filter(iq, code, energy):
    """Sparse direct matched filter: out[k] = sum_m y[k+m] conj(code[m]) / E."""
    y = np.asarray(iq, dtype=np.complex128)
    c = np.asarray(code, dtype=np.complex128)
    out_len = int(y.size) - int(c.size) + 1
    if out_len <= 0:
        raise ValueError("window shorter than code")
    acc = np.zeros(out_len, dtype=np.complex128)
    for m in np.flatnonzero(np.abs(c) > 0.0):
        acc += np.conj(c[m]) * y[m:m + out_len]
    return acc / float(energy)


def cir_tensor(mf, origin, sps, reps, tap_lo, tap_hi):
    ntaps = int(tap_hi) - int(tap_lo)
    H = np.full((int(reps), ntaps), np.nan, dtype=np.complex128)
    base = np.arange(int(reps))[:, None] * int(sps) + np.arange(
        int(tap_lo), int(tap_hi))[None, :] + int(origin)
    valid = (base >= 0) & (base <= mf.size - 1)
    H[valid] = mf[base[valid]]
    return H


def accum(H, start, n):
    return np.asarray(H, dtype=np.complex128)[start:start + int(n)].mean(axis=0)


def energy(H, lo=None, hi=None):
    H = np.asarray(H)
    seg = H[..., int(lo or 0):(None if hi is None else int(hi))]
    return float(np.sum(np.abs(seg) ** 2))


def ratio_db(num, den):
    if num <= 0.0 and den <= 0.0:
        return None, "na"
    if den <= 0.0:
        return float("inf"), "pos_inf"
    if num <= 0.0:
        return float("-inf"), "neg_inf"
    return 10.0 * math.log10(num / den), "numeric"


def coherence(E, lo, hi):
    seg = np.asarray(E, dtype=np.complex128)[:, int(lo):int(hi)]
    N = int(seg.shape[0])
    tot = float(np.sum(np.abs(seg) ** 2))
    if tot <= 0.0:
        return {"eta": None, "lag1": None, "centered_lag1": None,
                "phase_slope_rad_per_symbol": None, "energy": 0.0}
    s = seg.sum(axis=0)
    eta = float(np.sum(np.abs(s) ** 2) / (N * tot))
    lag1 = None
    clag1 = None
    if N > 1:
        a, b = seg[1:], seg[:-1]
        den = math.sqrt(float(np.sum(np.abs(a) ** 2))
                        * float(np.sum(np.abs(b) ** 2)))
        lag1 = None if den <= 0 else float(abs(np.vdot(a, b)) / den)
        ac, bc = a - a.mean(axis=0), b - b.mean(axis=0)
        denc = math.sqrt(float(np.sum(np.abs(ac) ** 2))
                         * float(np.sum(np.abs(bc) ** 2)))
        clag1 = None if denc <= 0 else float(abs(np.vdot(ac, bc)) / denc)
    dt = int(np.argmax(np.abs(seg).mean(axis=0)))
    phase = np.unwrap(np.angle(seg[:, dt]))
    slope = float(np.polyfit(np.arange(N, dtype=np.float64), phase, 1)[0]) \
        if N > 1 else 0.0
    return {"eta": eta, "lag1": lag1, "centered_lag1": clag1,
            "phase_slope_rad_per_symbol": slope, "energy": tot}


def percentile(vals, p):
    v = [float(x) for x in vals if x is not None and isinstance(
        x, (int, float)) and math.isfinite(float(x))]
    return float(np.percentile(v, p)) if v else None


def mean_of(vals):
    v = [float(x) for x in vals if x is not None and isinstance(
        x, (int, float)) and math.isfinite(float(x))]
    return float(np.mean(v)) if v else None


# ---------------------------------------------------------------------------
# Run loading
# ---------------------------------------------------------------------------
REQUIRED = ("run_manifest.json", "summary.json", "windows.jsonl",
            "operator.json", "cells.json", "accum_table.csv",
            "cells_summary.csv")


def load_run(run_dir):
    art = {"dir": run_dir}
    art["manifest"] = load_json(os.path.join(run_dir, "run_manifest.json"))
    art["summary"] = load_json(os.path.join(run_dir, "summary.json"))
    art["windows"] = load_jsonl(os.path.join(run_dir, "windows.jsonl"))
    art["operator"] = load_json(os.path.join(run_dir, "operator.json"))
    art["cells"] = load_json(os.path.join(run_dir, "cells.json"))
    op_npz = os.path.join(run_dir, "operator", "operator_arrays.npz")
    art["operator_arrays"] = dict(np.load(op_npz, allow_pickle=False)) \
        if os.path.exists(op_npz) else None
    return art


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
def check_manifest(art):
    m = art["manifest"]
    ok(("manifest", "task is S2-A"), m.get("task") == "S2-A",
       str(m.get("task")))
    lib = (m.get("loaded_library") or {})
    expected = str(lib.get("expected_lib") or "")
    worktree = str((m.get("git") or {}).get("worktree") or "")
    ok(("manifest", "built library inside this worktree"),
       bool(expected) and bool(worktree)
       and os.path.abspath(expected).startswith(os.path.abspath(worktree)),
       "%s vs %s" % (expected, worktree))
    ldd = " ".join(lib.get("ldd_module") or []) or \
        " ".join(lib.get("ldd_lib") or [])
    ok(("manifest", "ldd resolves the worktree library"),
       bool(expected) and expected in ldd, ldd[:160])
    ok(("manifest", "fixed SIC mode recorded"),
       (m.get("frozen_config") or {}).get("sic_model_mode")
       == "estimate8_frozen")
    ok(("manifest", "online_validation_reps == 0"),
       (m.get("frozen_config") or {}).get("online_validation_reps") == 0)
    ok(("manifest", "fit_reps == 8"),
       (m.get("frozen_config") or {}).get("fit_reps") == 8)
    nr = m.get("not_run") or {}
    ok(("manifest", "MATLAB declared not run"), "matlab" in nr,
       str(sorted(nr)))
    ok(("manifest", "native grid declared not run"), "native" in nr)
    ih = m.get("inputs_hashed") or {}
    ok(("manifest", "input hashes present"),
       all(len(v.get("sha256", "")) == 64 for v in ih.values()) and bool(ih),
       "%d inputs" % len(ih))


def check_contracts(art):
    run_op = art["operator"]
    repo_root = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir))
    repo_op_path = os.path.join(repo_root, "testdata", "sic_accum",
                                "operator.json")
    if os.path.exists(repo_op_path):
        same = sha256_file(repo_op_path) == sha256_file(
            os.path.join(art["dir"], "operator.json"))
        ok(("contract", "run operator.json matches the repo frozen copy"), same)
    o = run_op.get("operator") or {}
    r = run_op.get("roi") or {}
    sets = run_op.get("repetition_sets") or {}
    ok(("contract", "convention is average"),
       str(o.get("convention", "")).startswith("average"))
    ok(("contract", "phase rotation is identity"),
       str(o.get("phase_rotation", "")).startswith("identity"))
    ok(("contract", "gain normalization is fixed code_energy"),
       "code_energy" in str(o.get("normalization", "")))
    ok(("contract", "fixed N sweep declared"),
       sets.get("full_N") == [1, 2, 4, 8, 16, 32, 64]
       and sets.get("indep_nominal_N") == [1, 2, 4, 8, 16, 32, 56])
    ok(("contract", "independent region is support-disjoint by policy"),
       sets.get("indep_policy") == "support_disjoint"
       and sets.get("indep_strict_holdout") is True
       and sets.get("legacy_indep_start") == 8
       and "legacy_nominal_index" in str(sets.get("legacy_indep_note", "")))
    ok(("contract", "schema 2 coherence contract present"),
       (run_op.get("coherence") or {}).get("schema_version") == 2
       and "N * ||mu||^2" in str((run_op.get("coherence") or {}).get(
           "decomposition", {}).get("e_coherent", "")))
    ok(("contract", "operator declares the nonzero-template support"),
       (run_op.get("operator") or {}).get("support_source")
       == "nonzero_template_indices"
       and (run_op.get("operator") or {}).get("correlation_support", {}).get(
           "nz_count") == 64)
    tgt = r.get("target_offsets") or []
    ok(("contract", "target ROI declared and non-empty"),
       len(tgt) == 2 and tgt[1] > tgt[0], str(tgt))
    ok(("contract", "target tap inside the ROI"),
       len(tgt) == 2 and tgt[0] <= int(r.get("target_tap_offset", -1)) < tgt[1])
    ok(("contract", "A2 grid = 9 + 3"), _grid_ok(art["cells"]))
    ok(("contract", "SNR fixed at 30 dB"),
       float((run_op.get("evaluation") or {}).get("snr_db", -1)) == 30.0)


def _grid_ok(cells):
    got = set((c.get("isr_db"), c.get("cfo_hz"), c.get("comm_paths"),
               float(c.get("start_offset_work", 0))) for c in cells.get(
                   "cells", []))
    want = {(isr, cfo, "single", 0.0) for isr in (0, 10, 20)
            for cfo in (0, -50000, 50000)}
    want |= {(0, 50000, "four", 0.0), (10, 0, "four", 0.0),
             (10, 0, "single", 0.4)}
    return got == want


def check_operator(art):
    o = art["operator"]["operator"]
    arrays = art["operator_arrays"]
    ok(("operator", "operator arrays archived"), arrays is not None)
    if arrays is None:
        return
    sps = int(o["sps"])
    code = np.asarray(arrays["sampled_code"], dtype=np.complex128)
    chips = np.asarray(arrays["chips"], dtype=np.int8)
    taps = np.asarray(arrays["pulse_taps"], dtype=np.float64)
    template = np.asarray(arrays["sensing_template"], dtype=np.complex128)
    ok(("operator", "sampled code length == sps"), code.size == sps,
       "%d vs %d" % (code.size, sps))
    rebuilt = build_sampled_code(chips, sps=sps,
                                 spread=int(o.get("spread_factor", 4)),
                                 spp=int(o.get("samples_per_pulse", 2)))
    ok(("operator", "archived sampled_code == rebuild(chips)"),
       np.array_equal(code, rebuilt))
    ch, diag = recover_chips_ls(template, taps, sps,
                                int(o.get("code_len_chips", 127)),
                                int(o.get("chip_stride", 8)))
    ok(("operator", "chips re-derived from template+taps (exact)"),
       diag["rank"] == 127 and diag["resid_rel"] <= 1e-6
       and diag["max_int_dev"] <= 1e-3 and ch.size == chips.size
       and np.array_equal(ch, chips), json.dumps(diag, sort_keys=True))
    ce = float(np.real(np.vdot(code, code)))
    declared_ce = float(o.get("code_energy", 64.0))
    ok(("operator", "code energy == declared"),
       abs(ce - declared_ce) < 1e-9, "%.6f vs %.6f" % (ce, declared_ce))
    ok(("operator", "code is ternary 64-chip"),
       int(np.count_nonzero(np.abs(code) > 0.5)) == 64 and diag["energy"] == 64)


def _archive_files(run_dir, sub="archive"):
    d = os.path.join(run_dir, sub)
    if not os.path.isdir(d):
        return []
    return sorted(f for f in os.listdir(d) if f.endswith(".npz"))


def check_archive_policy(art):
    run_dir = art["dir"]
    decl = (art["summary"].get("archive") or {})
    files = _archive_files(run_dir, "archive")
    worst_files = _archive_files(run_dir, "archive_worst")
    ok(("archive", "declared count == fixed + worst files on disk"),
       int(decl.get("archived_windows", -1)) == len(files) + len(worst_files),
       "%s vs %d+%d" % (decl.get("archived_windows"), len(files),
                        len(worst_files)))
    n_arch = int((art["operator"].get("archive_policy") or {}).get(
        "windows_per_cell", 8))
    by_cell = {}
    bad = []
    for f in files:
        cell, _, rest = f.rpartition("_")
        wid = int(rest.split(".")[0])
        by_cell.setdefault(cell, []).append(wid)
    for cell, ids in by_cell.items():
        if sorted(ids) != list(range(len(ids))) or len(ids) > n_arch:
            bad.append((cell, ids))
    ok(("archive", "first-N-by-index selection (no metric-based picking)"),
       not bad, str(bad[:3]))
    ok(("archive", "every cell that was run has archived windows"),
       set(by_cell) == set(art["summary"].get("cells") or []),
       "%d vs %d cells" % (len(by_cell), len(art["summary"].get("cells") or [])))
    # A-R04: worst-window evidence must exist (no empty markers) and must be
    # declared as selection-biased evidence, not as an unbiased sample.
    want = decl.get("worst_windows") or []
    badw = []
    for e in want:
        cell = e.get("cell_id")
        wid = int(e.get("window_id", -1))
        reasons = list(e.get("reasons") or [e.get("reason")])
        path = e.get("path")
        if e.get("already_archived"):
            if not os.path.exists(os.path.join(run_dir, "archive",
                                               "%s_%05d.npz" % (cell, wid))):
                badw.append((cell, wid, reasons, "missing_fixed"))
            continue
        if not path:
            badw.append((cell, wid, reasons, "empty_marker"))
            continue
        full = os.path.join(run_dir, path)
        if not os.path.exists(full):
            badw.append((cell, wid, reasons, "missing"))
            continue
        z = np.load(full, allow_pickle=False)
        meta = json.loads(bytes(z["meta_json"]).decode("utf-8"))
        declared = str(meta.get("archive_reason") or "")
        if any(r not in declared for r in reasons if r):
            badw.append((cell, wid, reasons, "reason_mismatch", declared))
    ok(("archive", "worst-window evidence archived (no empty markers)"),
       not badw and len(want) > 0, str(badw[:3]))
    ok(("archive", "worst windows declared as selection-biased evidence"),
       "NOT used" in str(decl.get("worst_windows_rule", "")),
       str(decl.get("worst_windows_rule"))[:80])


def check_windows(art):
    wins = art["windows"]
    m = art["manifest"]
    s = art["summary"]
    tot = s.get("totals") or {}
    ok(("windows", "jsonl line count == totals.windows"),
       len(wins) == int(tot.get("windows", -1)),
       "%d vs %s" % (len(wins), tot.get("windows")))
    per_cell = {}
    for w in wins:
        per_cell[w["cell_id"]] = per_cell.get(w["cell_id"], 0) + 1
    ok(("windows", "per-cell counts == windows_per_cell"),
       all(v == int(s.get("windows_per_cell", -1)) for v in per_cell.values()),
       str(sorted(set(per_cell.values()))))
    ok(("windows", "failure/bypass counters consistent"),
       int(tot.get("committed", 0)) + int(tot.get("bypassed", 0)) == len(wins),
       "%s+%s" % (tot.get("committed"), tot.get("bypassed")))
    committed = sum(1 for w in wins if w["sic"]["committed"])
    bypassed = sum(1 for w in wins if not w["sic"]["committed"])
    ok(("windows", "committed/bypassed recount matches"),
       committed == int(tot.get("committed", -1))
       and bypassed == int(tot.get("bypassed", -1)),
       "%d/%d" % (committed, bypassed))
    ok(("windows", "no online window used truth"),
       all(not w["sic"].get("uses_truth", False) for w in wins)
       and int(tot.get("uses_truth", -1)) == 0)
    bad_refit = [w["window_id"] for w in wins
                 if w["sic"].get("apply_refit_count") not in (0, 0.0)]
    ok(("windows", "frozen model never re-fit in apply (refit_count==0)"),
       not bad_refit, str(bad_refit[:5]))
    # seed discipline
    op = art["operator"]["seeds"]
    base = int(op[art["summary"]["seed_set"]])
    stride = int(op["stride_per_cell"])
    order = (m.get("seeds") or {}).get("cell_order") or []
    bad_seed = []
    seen = set()
    for w in wins:
        if w["cell_id"] not in order:
            bad_seed.append(("cell", w["cell_id"]))
            continue
        want = base + order.index(w["cell_id"]) * stride + int(w["window_id"])
        if int(w["seed"]) != want:
            bad_seed.append((w["cell_id"], w["window_id"], w["seed"], want))
        if int(w["seed"]) in seen:
            bad_seed.append(("dup", w["seed"]))
        seen.add(int(w["seed"]))
    ok(("windows", "seed = base + cell*stride + window, all unique"),
       not bad_seed, str(bad_seed[:3]))
    cal = int(op["calibration"])
    acc = int(op["acceptance"])
    kinds = set(w["kind"] for w in wins)
    want_kind = art["summary"]["seed_set"]
    ok(("windows", "seed set membership matches the declared set"),
       kinds == {want_kind}, str(kinds))
    ok(("windows", "calibration and acceptance seeds disjoint"),
       cal + 12 * stride <= acc or acc + 12 * stride <= cal)


def _legal_run(nz_min, nz_max, origin, sps, sync_reps, tap_lo, tap_hi,
               visible_end, sub_lo, sub_hi, n):
    """Independent derivation of the maximal contiguous support-disjoint run."""
    reps = []
    excluded = {}
    for r in range(int(sync_reps)):
        lo = int(origin) + int(r) * int(sps) + int(tap_lo) + int(nz_min)
        hi = int(origin) + int(r) * int(sps) + int(tap_hi) - 1 + int(nz_max) + 1
        bad = []
        if lo < 0 or hi > int(n):
            bad.append("outside_window")
        if lo < int(visible_end):
            bad.append("reads_training_or_guard")
        if lo < int(sub_lo) or hi > int(sub_hi):
            bad.append("outside_applied_subtraction")
        if bad:
            excluded[r] = bad
        else:
            reps.append(r)
    runs = []
    for r in reps:
        if runs and r == runs[-1][-1] + 1:
            runs[-1].append(r)
        else:
            runs.append([r])
    run = max(runs, key=len) if runs else []
    return run, excluded


def _truncate_n(available, nominal):
    out = [int(n) for n in nominal if int(n) <= int(available)]
    if int(available) >= 32 and int(available) not in out:
        out.append(int(available))
    return out


def check_support(art):
    """A-R01: every window's repetition sets must be support-disjoint.

    The verifier re-derives the maximal legal run from the operator's nonzero
    template support, the recorded visibility limit and the recorded applied
    subtraction bounds, and requires the artifact to match exactly.  A label
    that says "independent" but actually reads training/guard or un-subtracted
    tail samples is rejected.
    """
    o = art["operator"]["operator"]
    arrays = art["operator_arrays"]
    bad = []
    bad_n = []
    bad_legacy = []
    bad_roi = []
    bad_cov = []
    for w in art["windows"]:
        sup = w.get("support") or {}
        if not sup:
            bad.append((w["cell_id"], w["window_id"], "missing"))
            continue
        code = np.asarray(arrays["sampled_code"], dtype=np.complex128)
        nz = np.flatnonzero(np.abs(code) > 0)
        if nz.size == 0:
            bad.append((w["cell_id"], w["window_id"], "empty_code"))
            continue
        if sup.get("no_applied_subtraction") or int(sup.get("sub_hi", -1)) < 0:
            # bypass/failed window: nothing was subtracted, so the window set
            # must be the PUBLIC nominal cell set, and that cell set must still
            # be the maximal legal run of its own recorded bounds.
            cs = sup.get("cell_set") or {}
            b = cs.get("bounds") or {}
            if not b:
                bad.append((w["cell_id"], w["window_id"], "no_bounds"))
                continue
            run, excluded = _legal_run(
                nz[0], nz[-1], int(o["origin"]), int(o["sps"]),
                int(o["sync_reps"]), int(o["tap_lo"]), int(o["tap_hi"]),
                int(b["visible_end"]), int(b["sub_lo"]), int(b["sub_hi"]),
                int(b["n"]))
            if (int(cs["first"]) != (run[0] if run else -1)
                    or int(cs["last"]) != (run[-1] if run else -1)
                    or int(cs["available"]) != len(run)):
                bad.append((w["cell_id"], w["window_id"], "cell_set"))
            if (w.get("support") or {}).get("set_used") != "cell":
                bad.append((w["cell_id"], w["window_id"], "bypass_not_cell"))
            continue
        run, excluded = _legal_run(
            nz[0], nz[-1], int(o["origin"]), int(o["sps"]),
            int(o["sync_reps"]), int(o["tap_lo"]), int(o["tap_hi"]),
            int(sup["visible_end"]), int(sup["sub_lo"]), int(sup["sub_hi"]),
            int(w["n_samples"]))
        sets = (w.get("accum") or {}).get("sets") or {}
        ind = sets.get("indep")
        if not ind:
            bad.append((w["cell_id"], w["window_id"], "missing_indep_set"))
            continue
        want_first = run[0] if run else -1
        want_last = run[-1] if run else -1
        if (int(ind["first"]) != want_first or int(ind["last"]) != want_last
                or int(ind["available"]) != len(run)):
            bad.append((w["cell_id"], w["window_id"], "set",
                        [ind["first"], ind["last"], ind["available"]],
                        [want_first, want_last, len(run)]))
        if bool(ind.get("strict_holdout")) is not True:
            bad.append((w["cell_id"], w["window_id"], "not_strict"))
        # the honest N list: a prefix of the nominal list plus the real tail
        nominal = list(ind.get("n_nominal") or [1, 2, 4, 8, 16, 32, 56])
        if list(ind.get("n_list") or []) != _truncate_n(len(run), nominal):
            bad_n.append((w["cell_id"], w["window_id"],
                          ind.get("n_list"), _truncate_n(len(run), nominal)))
        if int(ind["first"]) > 0 and (int(ind["first"]) - 1) not in excluded:
            bad.append((w["cell_id"], w["window_id"], "first_not_maximal"))
        if run and (int(ind["last"]) + 1) not in excluded \
                and int(ind["last"]) + 1 < int(o["sync_reps"]):
            bad.append((w["cell_id"], w["window_id"], "last_not_maximal"))
        # the legacy label must be declared NOT support-disjoint
        leg = sets.get("legacy_nominal_index")
        if leg is not None:
            if bool(leg.get("strict_holdout")) is not False:
                bad_legacy.append((w["cell_id"], w["window_id"], "strict"))
            first_leg = int(leg.get("first", 8))
            if first_leg not in excluded:
                # its first repetition is legal, so it cannot be used to claim
                # legacy leakage; only a *label* problem if it claims holdout
                pass
        # per-ROI appendix sets must cover the main set (narrower tap span)
        for rname, app in (w.get("roi_sets_appendix") or {}).items():
            st = app.get("set") or {}
            if int(st.get("first", 10 ** 9)) > int(ind["first"]) or \
                    int(st.get("last", -1)) < int(ind["last"]):
                bad_roi.append((w["cell_id"], w["window_id"], rname,
                                st.get("first"), st.get("last")))
        cov = sup.get("coverage") or {}
        if cov.get("cir_support"):
            lo, hi = int(cov["cir_support"][0]), int(cov["cir_support"][1])
            slo, shi = int(sup["sub_lo"]), int(sup["sub_hi"])
            inter = max(0, min(hi, shi) - max(lo, slo))
            want = inter / max(1, hi - lo)
            got = cov.get("cir_overlap_sub_ratio")
            if got is None or abs(float(got) - want) > 1e-9:
                bad_cov.append((w["cell_id"], w["window_id"], got, want))
    ok(("support", "every window's indep set == the maximal legal run"), not bad,
       str(bad[:3]))
    ok(("support", "honest N list (prefix + real tail, no relabel)"), not bad_n,
       str(bad_n[:3]))
    ok(("support", "legacy [8,8+N) is declared non-holdout"), not bad_legacy,
       str(bad_legacy[:3]))
    ok(("support", "per-ROI appendix sets contain the main set"), not bad_roi,
       str(bad_roi[:3]))
    ok(("support", "CIR/subtraction coverage arithmetic consistent"),
       not bad_cov, str(bad_cov[:3]))
    committed = sum(1 for w in art["windows"]
                    if (w.get("sic") or {}).get("committed"))
    if committed > 0:
        w0 = next(w for w in art["windows"]
                  if (w.get("sic") or {}).get("committed"))
        sup = w0.get("support") or {}
        ok(("support", "training span and visibility are recorded per "
            "committed window"),
           int(sup.get("visible_end", -1)) > 0
           and int(sup.get("training_hi", -1)) > 0
           and int(sup.get("sub_hi", -1)) > int(sup.get("sub_lo", -1)),
           str({k: sup.get(k) for k in ("visible_end", "training_lo",
                                        "training_hi", "sub_lo", "sub_hi")}))
    else:
        w0 = art["windows"][0]
        cs = (w0.get("support") or {}).get("cell_set") or {}
        ok(("support", "bypass run: public nominal cell set is recorded with "
            "explicit bounds"),
           bool(cs.get("bounds")) and int(cs.get("available", 0)) > 0
           and (w0["support"].get("no_applied_subtraction") is True),
           str({k: w0["support"].get(k) for k in
                ("no_applied_subtraction", "set_used")}))


def check_coherence_energy(art):
    """A-R02: same-dimension coherent/centered energy decomposition."""
    bad = []
    bad_cons = []
    bad_eta = []
    bad_old = []
    for w in art["windows"]:
        for view in ("coherence", "coherence_full_preamble"):
            for tag in ("sic", "raw"):
                for roi in ("target", "background", "all"):
                    c = ((w.get(view) or {}).get(tag) or {}).get(roi) or {}
                    if c.get("eta") is None:
                        if c.get("coherent_fraction") is not None:
                            bad.append((w["cell_id"], w["window_id"], view,
                                        tag, roi, "na"))
                        continue
                    n = int(c["N"])
                    mu = float(c["mu_energy"])
                    ec = float(c["e_coherent"])
                    ez = float(c["e_centered"])
                    et = float(c["e_total"])
                    if abs(ec - n * mu) > 1e-9 * max(1.0, et):
                        bad.append((w["cell_id"], w["window_id"], view, tag,
                                    roi, "e_coherent != N*mu"))
                    if abs(et - (ec + ez)) > 1e-9 * max(1.0, et):
                        bad_cons.append((w["cell_id"], w["window_id"], view,
                                         tag, roi))
                    if abs(float(c["coherent_fraction"]) - ec / et) > 1e-9:
                        bad_eta.append((w["cell_id"], w["window_id"], view,
                                        tag, roi))
                    if c.get("mean_fraction") is not None:
                        bad_old.append((w["cell_id"], w["window_id"], view,
                                        tag, roi))
                    if c.get("energy_conservation_rel") is not None and \
                            float(c["energy_conservation_rel"]) > 1e-6:
                        bad_cons.append((w["cell_id"], w["window_id"], view,
                                         tag, roi, "rel"))
    ok(("energy", "e_coherent == N * ||mu||^2 in every window"), not bad,
       str(bad[:3]))
    ok(("energy", "e_total == e_coherent + e_centered"), not bad_cons,
       str(bad_cons[:3]))
    ok(("energy", "coherent_fraction == e_coherent/e_total"), not bad_eta,
       str(bad_eta[:3]))
    ok(("energy", "dimensionally-wrong mean_fraction retired (None)" +
       " with supersession", not bad_old), str(bad_old[:3]))


def check_archived_windows(art, verbose=False):
    """Recompute H, the accumulation table, target, and both coherence views."""
    run_dir = art["dir"]
    o = art["operator"]["operator"]
    r = art["operator"]["roi"]
    sps = int(o["sps"])
    origin = int(o["origin"])
    reps_n = int(o["sync_reps"])
    tap_lo = int(o["tap_lo"])
    tap_hi = int(o["tap_hi"])
    code = np.asarray(art["operator_arrays"]["sampled_code"],
                      dtype=np.complex128)
    ecode = float(np.real(np.vdot(code, code)))
    tgt_lo = int(r["target_offsets"][0]) - tap_lo
    tgt_hi = int(r["target_offsets"][1]) - tap_lo
    bg_lo = int(r["background_offsets"][0]) - tap_lo
    bg_hi = int(r["background_offsets"][1]) - tap_lo
    tgt_tap = int(r["target_tap_offset"]) - tap_lo
    recs = {(w["cell_id"], w["window_id"]): w for w in art["windows"]}
    files = [("archive", f) for f in _archive_files(run_dir, "archive")]
    files += [("archive_worst", f)
              for f in _archive_files(run_dir, "archive_worst")]
    if not files:
        fail(("archive", "recompute: no archived windows"))
        return
    h_bad, tab_bad, tgt_bad, coh_bad, miss, iq_bad = [], [], [], [], [], []
    for subdir, f in files:
        cell, _, rest = f.rpartition("_")
        wid = int(rest.split(".")[0])
        z = np.load(os.path.join(run_dir, subdir, f), allow_pickle=False)
        clean = np.asarray(z["clean"], dtype=np.complex128)
        raw = np.asarray(z["raw"], dtype=np.complex128)
        sic = np.asarray(z["sic"], dtype=np.complex128)
        Hc = cir_tensor(matched_filter(clean, code, ecode), origin, sps, reps_n,
                        tap_lo, tap_hi)
        Hr = cir_tensor(matched_filter(raw, code, ecode), origin, sps, reps_n,
                        tap_lo, tap_hi)
        Hs = cir_tensor(matched_filter(sic, code, ecode), origin, sps, reps_n,
                        tap_lo, tap_hi)
        for tag, H, key in (("clean", Hc, "H_clean"), ("raw", Hr, "H_raw"),
                            ("sic", Hs, "H_sic")):
            arch = np.asarray(z[key], dtype=np.complex128)
            denom = max(1e-30, float(np.max(np.abs(H))))
            if arch.shape != H.shape or float(np.max(np.abs(arch - H))) / denom \
                    > 1e-9:
                h_bad.append((cell, wid, tag))
        rec = recs.get((cell, wid))
        if rec is None:
            miss.append((cell, wid))
            continue
        sets = rec["accum"]["sets"]
        for set_name, meta in sets.items():
            first = int(meta["first"])
            last = int(meta["last"])
            n_list = [int(n) for n in (meta.get("n_list") or [])]
            if not n_list or first < 0:
                continue
            avail = last - first + 1
            e1r = e1s = None
            for N in n_list:
                n_eff = min(int(N), avail)
                idx = list(range(first, first + n_eff))
                hc = accum(Hc, first, n_eff)
                hr = accum(Hr, first, n_eff)
                hs = accum(Hs, first, n_eff)
                er = energy(hr - hc)
                es = energy(hs - hc)
                c, flag = ratio_db(er, es)
                got = rec["accum"][set_name].get(str(N))
                if got is None:
                    tab_bad.append((cell, wid, set_name, N, "missing"))
                    continue
                for name, want, have in (
                        ("e_raw", er, got["e_raw"]["full"]),
                        ("e_sic", es, got["e_sic"]["full"])):
                    if abs(want - float(have)) > 1e-9 * max(1.0, abs(want)):
                        tab_bad.append((cell, wid, set_name, N, name))
                if got["c_cir_flag"]["full"] != flag:
                    tab_bad.append((cell, wid, set_name, N, "c_cir_flag"))
                elif flag == "numeric" and abs(
                        float(got["c_cir_db"]["full"]) - c) > 1e-6:
                    tab_bad.append((cell, wid, set_name, N, "c_cir_db"))
                nm = er / energy(hc) if energy(hc) > 0 else None
                if nm is not None and abs(nm - float(
                        got["nmse_raw"]["full"])) > 1e-6 * max(1.0, nm):
                    tab_bad.append((cell, wid, set_name, N, "nmse_raw"))
                if N == n_list[0]:
                    e1r, e1s = er, es
                gr, _ = ratio_db(e1r, er)
                gs, _ = ratio_db(e1s, es)
                for name, want, have in (("g_raw_db", gr, got["g_raw_db"]),
                                         ("g_sic_db", gs, got["g_sic_db"])):
                    if want is None or have is None:
                        if want is not None or have is not None:
                            tab_bad.append((cell, wid, set_name, N, name))
                    elif abs(float(have) - want) > 1e-6:
                        tab_bad.append((cell, wid, set_name, N, name))
                tkey = "%s/%d" % (set_name, N)
                if tkey in (rec.get("target") or {}):
                    t = rec["target"][tkey]["sic"]
                    tr = rec["target"][tkey]["raw"]
                    pk = int(np.argmax(np.abs(hs[tgt_lo:tgt_hi]))) + tgt_lo
                    pkc = int(np.argmax(np.abs(hc[tgt_lo:tgt_hi]))) + tgt_lo
                    if pk - pkc != int(t["peak_shift"]):
                        tgt_bad.append((cell, wid, tkey, "peak_shift"))
                    amp = abs(hs[tgt_tap]) / abs(hc[tgt_tap])
                    want = 20.0 * math.log10(amp) if amp > 0 else None
                    if want is not None and abs(want - float(
                            t["amp_err_db"])) > 1e-6 * max(1.0, abs(want)):
                        tgt_bad.append((cell, wid, tkey, "amp_err"))
                    amp_r = abs(hr[tgt_tap]) / abs(hc[tgt_tap])
                    want_r = 20.0 * math.log10(amp_r) if amp_r > 0 else None
                    if want_r is not None and abs(want_r - float(
                            tr["amp_err_db"])) > 1e-6 * max(1.0, abs(want_r)):
                        tgt_bad.append((cell, wid, tkey, "raw_amp_err"))
        # IQ-level actual suppression (online metric, recomputed from arrays)
        span_lo = int(o["origin"])
        span_hi = min(len(clean), span_lo + int(o["sync_reps"]) * int(o["sps"]))
        for tag, (lo, hi) in (("span", (span_lo, span_hi)),
                              ("window", (0, len(clean)))):
            r_ = raw[lo:hi] - clean[lo:hi]
            s_ = sic[lo:hi] - clean[lo:hi]
            num = float(np.sum(np.abs(r_) ** 2))
            den = float(np.sum(np.abs(s_) ** 2))
            want, wflag = ratio_db(num, den)
            got = rec["iq"][tag]
            if got.get("flag") != wflag:
                iq_bad.append((cell, wid, tag, "flag"))
            elif wflag == "numeric" and abs(float(got["suppression_db"])
                                            - want) > 1e-6:
                iq_bad.append((cell, wid, tag, "db"))
        # coherence on the exact set that was used
        sup = rec["support"]
        used = (sup["cell_set"] if sup.get("set_used") == "cell"
                else sup["window_set"])
        main_idx = list(range(int(used["first"]), int(used["last"]) + 1))
        views = (("coherence", main_idx),
                 ("coherence_full_preamble", list(range(reps_n))))
        for view, idx in views:
            E = Hs - Hc
            for roi, (lo, hi) in (("target", (tgt_lo, tgt_hi)),
                                  ("background", (bg_lo, bg_hi)),
                                  ("all", (0, Hs.shape[1]))):
                c = coherence(E[np.asarray(idx)], lo, hi)
                got = rec[view]["sic"][roi]
                if c["eta"] is None:
                    if got["eta"] is not None:
                        coh_bad.append((cell, wid, view, roi, "eta_na"))
                    continue
                n = len(idx)
                ec = n * float(np.sum(np.abs(
                    E[np.asarray(idx)][:, lo:hi].mean(axis=0)) ** 2))
                ez = float(np.sum(np.abs(
                    E[np.asarray(idx)][:, lo:hi]
                    - E[np.asarray(idx)][:, lo:hi].mean(axis=0)) ** 2))
                if abs(float(got["e_coherent"]) - ec) > 1e-9 * max(1.0, ec):
                    coh_bad.append((cell, wid, view, roi, "e_coherent"))
                if abs(float(got["e_centered"]) - ez) > 1e-9 * max(1.0, ez + ec):
                    coh_bad.append((cell, wid, view, roi, "e_centered"))
                if abs(float(got["coherent_fraction"]) - c["eta"]) > 1e-9:
                    coh_bad.append((cell, wid, view, roi, "coherent_fraction"))
                if abs(float(got["energy"]) - (ec + ez)) > 1e-9 * max(
                        1.0, ec + ez):
                    coh_bad.append((cell, wid, view, roi, "energy_total"))
    ok(("recompute", "archived H_clean/H_raw/H_sic reproduced (%d windows)"
        % len(files)), not h_bad, str(h_bad[:4]))
    ok(("recompute", "archived accumulation tables reproduced (all sets, "
        "honest N)"), not tab_bad, str(tab_bad[:4]))
    ok(("recompute", "archived target metrics reproduced"), not tgt_bad,
       str(tgt_bad[:4]))
    ok(("recompute", "coherent/centered energy fields reproduced"), not coh_bad,
       str(coh_bad[:4]))
    ok(("recompute", "IQ-level actual suppression reproduced from arrays"),
       not iq_bad, str(iq_bad[:4]))
    ok(("recompute", "every archived window has a jsonl record"), not miss,
       str(miss[:4]))


def check_summary(art):
    s = art["summary"]
    wins = art["windows"]
    if int(s.get("schema_version", 0)) != 2:
        fail(("summary", "schema_version is 2 (rectification)"),
             str(s.get("schema_version")))
    bad = []
    per_cell = {}
    for w in wins:
        per_cell.setdefault(w["cell_id"], []).append(w)
    for cell, recs in per_cell.items():
        cs = (s.get("per_cell") or {}).get(cell)
        if cs is None:
            bad.append((cell, "missing cell summary"))
            continue
        if cs["windows"] != len(recs):
            bad.append((cell, "windows", cs["windows"], len(recs)))
        sets = (recs[0].get("accum") or {}).get("sets") or {}
        if cs.get("sets") != sets:
            bad.append((cell, "sets", "summary != window record"))
        for set_name, meta in sets.items():
            n_list = [int(n) for n in (meta.get("n_list") or [])]
            for N in n_list:
                key = "%s/%d" % (set_name, N)
                for roi in ("full", "target", "background", "direct"):
                    vals = [r["accum"][set_name][str(N)]["c_cir_db"][roi]
                            for r in recs]
                    finite = [float(v) for v in vals if v is not None
                              and isinstance(v, (int, float))
                              and math.isfinite(float(v))]
                    na = sum(1 for r in recs if r["accum"][set_name][str(N)]
                             ["c_cir_flag"][roi] == "na")
                    g = cs["accum"][key][roi]
                    if int(g["n_defined"]) != len(finite):
                        bad.append((cell, key, roi, "n_defined"))
                    if int(g["n_na"]) != na:
                        bad.append((cell, key, roi, "n_na"))
                    if (mean_of(finite) is None) != (g["mean"] is None):
                        bad.append((cell, key, roi, "mean_none"))
                    elif mean_of(finite) is not None and abs(
                            mean_of(finite) - float(g["mean"])) > 1e-6:
                        bad.append((cell, key, roi, "mean"))
                    if mean_of(finite) is not None and abs(
                            percentile(finite, 50)
                            - float(g["median"])) > 1e-6:
                        bad.append((cell, key, roi, "median"))
                for gname in ("g_sic_db", "g_raw_db"):
                    vals = [r["accum"][set_name][str(N)][gname] for r in recs]
                    m = mean_of(vals)
                    if (m is None) != (cs["accum"][key][gname]["mean"] is None):
                        bad.append((cell, key, gname, "none"))
                    elif m is not None and abs(
                            m - float(cs["accum"][key][gname]["mean"])) > 1e-6:
                        bad.append((cell, key, gname, "mean"))
                amp = [abs(r["target"][key]["sic"]["amp_err_db"]) for r in recs
                       if r["target"][key]["sic"]["amp_err_db"] is not None]
                shift = [abs(r["target"][key]["sic"]["peak_shift"])
                         for r in recs]
                t = cs["target"][key]
                if int(t["n_defined"]) != len(amp):
                    bad.append((cell, key, "target", "n_defined"))
                if amp and abs(percentile(amp, 95)
                               - float(t["amp_err_abs_p95_db"])) > 1e-6:
                    bad.append((cell, key, "target", "amp_p95"))
                if shift and abs(max(shift)
                                 - float(t["peak_shift_abs_max"])) > 1e-9:
                    bad.append((cell, key, "target", "peak_shift"))
                if t["pass_amp_p95_le_1db"] != bool(
                        amp and percentile(amp, 95) <= 1.0):
                    bad.append((cell, key, "target", "pass_amp"))
                if t["pass_peak_shift_le_1"] != bool(shift and max(shift) <= 1):
                    bad.append((cell, key, "target", "pass_shift"))
        # support bookkeeping aggregated from the window records
        sup = cs.get("support") or {}
        want_mism = [r["window_id"] for r in recs
                     if not (r.get("support") or {}).get("cell_set_contained",
                                                         True)]
        if intra_list_eq(sup.get("cell_set_mismatch_windows"), want_mism) is False:
            bad.append((cell, "support", "mismatch list"))
        if sup.get("cell_set") != (recs[0].get("support") or {}).get("cell_set"):
            bad.append((cell, "support", "cell_set"))
        used = sup.get("set_used_counts") or {}
        if int(used.get("cell", -1)) + int(used.get("window", -1)) != len(recs):
            bad.append((cell, "support", "set_used_counts"))
        # worst-window evidence must be present and explicitly non-unbiased
        ww = cs.get("worst_windows") or {}
        undef = list(ww.get("selection_undefined") or [])
        if ww.get("lowest_c_cir") is None and \
                "lowest_c_cir" not in undef:
            bad.append((cell, "worst_windows", "missing_lowest"))
        if ww.get("largest_target_error") is None and \
                "largest_target_error" not in undef:
            bad.append((cell, "worst_windows", "missing_target"))
        if "NOT used" not in str(ww.get("selection_rule", "")):
            bad.append((cell, "worst_windows", "rule"))
    ok(("summary", "per-cell aggregates recomputed from windows.jsonl "
        "(all sets, honest N)"), not bad, str(bad[:6]))
    # coherence aggregation
    badc = []
    for cell, recs in per_cell.items():
        cs = s["per_cell"][cell]
        for view in ("coherence", "coherence_full_preamble"):
            for tag in ("sic", "raw"):
                for roi in ("target", "background", "all"):
                    key = "%s/%s/%s" % (view, tag, roi)
                    agg = (cs.get("coherence") or {}).get(key)
                    if not agg:
                        badc.append((cell, key, "missing"))
                        continue
                    vals = [r[view][tag][roi].get("coherent_fraction")
                            for r in recs]
                    m = mean_of(vals)
                    if (m is None) != (agg.get("coherent_fraction_mean")
                                       is None):
                        badc.append((cell, key, "none"))
                    elif m is not None and abs(
                            m - float(agg["coherent_fraction_mean"])) > 1e-6:
                        badc.append((cell, key, "mean"))
                    cons = [r[view][tag][roi].get("energy_conservation_rel")
                            for r in recs]
                    cm = max([c for c in cons if c is not None], default=None)
                    if cm is not None and float(
                            agg.get("energy_conservation_rel_max", 1e9)) \
                            < cm - 1e-12:
                        badc.append((cell, key, "conservation"))
    ok(("summary", "coherence aggregation (both views, energies) reproduced"),
       not badc, str(badc[:5]))
    # cells_summary.csv must agree with summary.json
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import csv as _csv
    rows = {}
    with open(os.path.join(art["dir"], "cells_summary.csv"), newline="") as fh:
        for row in _csv.DictReader(fh):
            rows[(row["cell_id"], row["set"], row["N"], row["roi"])] = row
    badcsv = []
    for cell, cs in (s.get("per_cell") or {}).items():
        for key, entry in cs.get("accum", {}).items():
            set_name, N = key.split("/")
            for roi in ("full", "target", "background", "direct"):
                row = rows.get((cell, set_name, N, roi))
                if row is None:
                    badcsv.append((cell, key, roi, "missing"))
                    continue
                if int(row["n_defined"]) != int(entry[roi]["n_defined"]):
                    badcsv.append((cell, key, roi, "n_defined"))
                want = entry[roi]["mean"]
                got = str(row["c_cir_db_mean"]).strip()
                if want is None:
                    if got not in ("na", "", "None"):
                        badcsv.append((cell, key, roi, "fabricated_mean"))
                elif got in ("na", "", "None") or abs(
                        float(got) - float(want)) > 1e-6:
                    badcsv.append((cell, key, roi, "mean"))
    ok(("summary", "cells_summary.csv agrees with summary.json"), not badcsv,
       str(badcsv[:5]))


def intra_list_eq(a, b):
    return list(a or []) == list(b or [])


def check_receiver(art):
    s = art["summary"]
    wins = art["windows"]
    recs = [w for w in wins if "receiver" in w]
    if not recs:
        ok(("receiver", "receiver layer not part of this run (declared)"),
           True)
        return
    bad = []
    for w in recs:
        for key, cfg in w["receiver"]["configs"].items():
            if "skip" not in cfg or "count" not in cfg:
                bad.append((w["cell_id"], w["window_id"], key, "unlabelled"))
            for b in ("clean", "raw", "sic"):
                st = cfg["branches"][b].get("status")
                if st not in ("ok", "timeout", "sfd_failed", "timing_failed",
                              "cir_failed", "invalid_input", "queue_full",
                              "internal_error"):
                    bad.append((w["cell_id"], w["window_id"], key, b, st))
    ok(("receiver", "every receiver result is labelled and accounted"),
       not bad, str(bad[:4]))
    ok(("receiver", "fixed-operator sets are not conflated with receiver sets"),
       all(c["skip"] is not None and c["count"] is not None
           for w in recs for c in w["receiver"]["configs"].values()))
    for cell, cs in (s.get("per_cell") or {}).items():
        rx = cs.get("receiver") or {}
        for key, cfg in (rx.get("configs") or {}).items():
            for b in ("clean", "raw", "sic"):
                br = cfg["branches"][b]
                if not (0 <= int(br["n_ok"]) <= int(br["n_total"])):
                    fail(("receiver", "n_ok within n_total", cell, key, b))
                    return
    ok(("receiver", "n_ok <= n_total for every branch"), True)


def check_bypass(art):
    s = art["summary"]
    if s.get("mode") != "bypass":
        ok(("bypass", "not a bypass run"), True)
        return
    tot = s["totals"]
    ok(("bypass", "every window bypassed"),
       int(tot["bypassed"]) == int(tot["windows"]), str(tot))
    ok(("bypass", "no output changed"), int(tot["output_changed"]) == 0)
    changed = [w["window_id"] for w in art["windows"] if w["iq"]["output_changed"]]
    ok(("bypass", "out == y byte-for-byte on every window"), not changed,
       str(changed[:5]))
    bad = []
    for w in art["windows"]:
        for set_name in ("full", "indep"):
            for N, entry in w["accum"][set_name].items():
                if entry["c_cir_flag"]["full"] not in ("na", "pos_inf"):
                    bad.append((w["window_id"], set_name, N))
    ok(("bypass", "no fabricated C_CIR on a bypassed window"), not bad,
       str(bad[:5]))


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------
def verify_run(run_dir, verbose=False):
    del RESULTS[:]
    for f in REQUIRED:
        p = os.path.join(run_dir, f)
        if not os.path.exists(p):
            fail(("artifacts", "required file present: %s" % f), p)
    if any(not r[0] for r in RESULTS):
        return RESULTS
    art = load_run(run_dir)
    for fn in (check_manifest, check_contracts, check_operator,
               check_archive_policy, check_windows, check_support,
               check_coherence_energy, check_archived_windows,
               check_summary, check_receiver, check_bypass):
        try:
            if fn is check_archived_windows:
                fn(art, verbose=verbose)
            else:
                fn(art)
        except Exception as exc:  # a tampered artifact must never crash us
            fail((fn.__name__, "raised %r" % (exc,)))
    return RESULTS


def print_results(results, verbose=False):
    n_fail = 0
    for okk, name, detail in results:
        if not okk:
            n_fail += 1
        if verbose or not okk:
            label = "[%s]" % (".".join(str(x) for x in name)
                              if isinstance(name, tuple) else str(name))
            print("%-6s %-70s %s" % ("PASS" if okk else "FAIL", label,
                                     (detail or "")[:120]))
    print("checks: %d, failures: %d" % (len(results), n_fail))
    return n_fail


# ---------------------------------------------------------------------------
# Reject QA: mutate a real run and require the verifier to catch it
# ---------------------------------------------------------------------------
def _mutate_copy(src, dst):
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    return dst


def _first_archive(dst, field=None):
    d = os.path.join(dst, "archive")
    f = sorted(x for x in os.listdir(d) if x.endswith(".npz"))[0]
    z = dict(np.load(os.path.join(d, f), allow_pickle=False))
    return d, f, z


def reject_qa(src_run, workdir, verbose=False):
    """Each mutation must be rejected; return the number of un-caught ones."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    mutations = []

    def mutation(name):
        def deco(fn):
            mutations.append((name, fn))
            return fn
        return deco

    import csv as _csv

    @mutation("drop a window line (wrong denominator)")
    def _m_drop(dst):
        p = os.path.join(dst, "windows.jsonl")
        lines = open(p).read().splitlines()
        open(p, "w").write("\n".join(lines[:-1]) + "\n")

    @mutation("flip one C_CIR value in windows.jsonl")
    def _m_flip(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        e = recs[0]["accum"]["full"]["8"]
        if e["c_cir_db"]["full"] is None:
            # N/A run (e.g. bypass): fabricate a residual energy instead, which
            # the verifier recomputes from the archived arrays.
            e["e_raw"]["full"] = 1.0
        else:
            e["c_cir_db"]["full"] = float(e["c_cir_db"]["full"]) + 1.0
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("tamper an archived H array")
    def _m_H(dst):
        d, f, z = _first_archive(dst)
        z["H_sic"] = z["H_sic"] * 2.0
        np.savez_compressed(os.path.join(d, f), **z)

    @mutation("scale an archived SIC IQ array")
    def _m_iq(dst):
        d, f, z = _first_archive(dst)
        z["sic"] = z["sic"] * 1.05
        np.savez_compressed(os.path.join(d, f), **z)

    @mutation("change the operator target ROI")
    def _m_roi(dst):
        p = os.path.join(dst, "operator.json")
        o = json.load(open(p))
        o["roi"]["target_offsets"] = [o["roi"]["target_offsets"][0] + 3,
                                      o["roi"]["target_offsets"][1] + 3]
        o["roi"]["target_tap_offset"] = o["roi"]["target_tap_offset"] + 3
        json.dump(o, open(p, "w"), indent=2)

    @mutation("change the N sweep in operator.json")
    def _m_N(dst):
        p = os.path.join(dst, "operator.json")
        o = json.load(open(p))
        o["repetition_sets"]["full_N"] = [1, 2, 3, 4]
        json.dump(o, open(p, "w"), indent=2)

    @mutation("tamper one summary mean (falls back to a wrong denominator)")
    def _m_sum(dst):
        p = os.path.join(dst, "summary.json")
        s = json.load(open(p))
        cell = s["cells"][0]
        entry = s["per_cell"][cell]["accum"]["full/8"]["full"]
        if entry["mean"] is None:
            s["per_cell"][cell]["windows"] = int(
                s["per_cell"][cell]["windows"]) + 1
        else:
            entry["mean"] = float(entry["mean"]) + 2.0
        json.dump(s, open(p, "w"), indent=2, allow_nan=True)

    @mutation("point the loaded library outside the worktree")
    def _m_lib(dst):
        p = os.path.join(dst, "run_manifest.json")
        m = json.load(open(p))
        m["loaded_library"]["expected_lib"] = "/usr/local/lib/" \
            "libgnuradio-uwb.so.1.0.0"
        json.dump(m, open(p, "w"), indent=2, allow_nan=True)

    @mutation("remove the MATLAB not-run declaration")
    def _m_not_run(dst):
        p = os.path.join(dst, "run_manifest.json")
        m = json.load(open(p))
        m["not_run"].pop("matlab", None)
        json.dump(m, open(p, "w"), indent=2, allow_nan=True)

    @mutation("mark one online window as truth-using")
    def _m_truth(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        recs[0]["sic"]["uses_truth"] = True
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("change one window seed to a value outside every seed set")
    def _m_seed(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        recs[0]["seed"] = 123456789
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("delete an archived window file")
    def _m_arch(dst):
        d, f, _z = _first_archive(dst)
        os.remove(os.path.join(d, f))

    @mutation("tamper cells_summary.csv (fabricate a mean / wrong denominator)")
    def _m_csv(dst):
        p = os.path.join(dst, "cells_summary.csv")
        with open(p, newline="") as fh:
            rows = list(_csv.DictReader(fh))
            fields = list(rows[0].keys())
        for r in rows:
            if r["roi"] == "full":
                if r["c_cir_db_mean"] in ("", "na"):
                    r["c_cir_db_mean"] = repr(12.0)   # fabricate on an N/A cell
                else:
                    r["c_cir_db_mean"] = repr(
                        float(r["c_cir_db_mean"]) + 1.0)
                break
        with open(p, "w", newline="") as fh:
            w = _csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)

    @mutation("claim a leaked [8,..] set as support-disjoint")
    def _m_support(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        for r in recs:
            for key in ("window_set", "cell_set"):
                if r["support"].get(key):
                    r["support"][key]["first"] = 8
            ind = r["accum"]["sets"]["indep"]
            ind["first"] = 8
            ind["available"] = int(ind["last"]) - 8 + 1
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("drop the N factor in e_coherent (missing-N energy bug)")
    def _m_energy(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        for view in ("coherence", "coherence_full_preamble"):
            for tag in ("sic", "raw"):
                for roi in ("target", "background", "all"):
                    c = recs[0][view][tag][roi]
                    if c.get("mu_energy") is not None:
                        c["e_coherent"] = float(c["mu_energy"])
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("declare the legacy index set as a strict holdout")
    def _m_legacy(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        recs[0]["accum"]["sets"]["legacy_nominal_index"]["strict_holdout"] = True
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("relabel the independent N list 54 -> 56")
    def _m_nrelabel(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        for r in recs:
            ind = r["accum"]["sets"]["indep"]
            ind["n_list"] = [1, 2, 4, 8, 16, 32, 56]
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("remove the worst-window archive declaration")
    def _m_worst(dst):
        p = os.path.join(dst, "summary.json")
        s = json.load(open(p))
        s["archive"]["worst_windows"] = []
        json.dump(s, open(p, "w"), indent=2, allow_nan=True)

    @mutation("tamper an online IQ suppression value")
    def _m_iqmetric(dst):
        p = os.path.join(dst, "windows.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        if recs[0]["iq"]["span"].get("suppression_db") is not None:
            recs[0]["iq"]["span"]["suppression_db"] = float(
                recs[0]["iq"]["span"]["suppression_db"]) + 3.0
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True) + "\n")

    @mutation("declare a different seed set in summary")
    def _m_set(dst):
        p = os.path.join(dst, "summary.json")
        s = json.load(open(p))
        s["seed_set"] = "acceptance" if s["seed_set"] != "acceptance" \
            else "calibration"
        json.dump(s, open(p, "w"), indent=2, allow_nan=True)

    uncaught = []
    for i, (name, fn) in enumerate(mutations):
        dst = os.path.join(workdir, "m%02d" % i)
        _mutate_copy(src_run, dst)
        try:
            fn(dst)
        except Exception as exc:  # a mutation that cannot be applied is a bug
            uncaught.append((name, "mutation failed: %r" % (exc,)))
            continue
        res = verify_run(dst, verbose=False)
        caught = any(not r[0] for r in res)
        if verbose:
            print("%-6s mutation: %s" % ("CAUGHT" if caught else "MISSED", name))
        if not caught:
            uncaught.append((name, "verifier accepted the mutated artifact"))
        shutil.rmtree(dst, ignore_errors=True)
    if verbose:
        print("reject-QA: %d/%d mutations caught"
              % (len(mutations) - len(uncaught), len(mutations)))
    return uncaught, len(mutations)


# ---------------------------------------------------------------------------
# A-R03 error-budget verification (independent core-operator recomputation)
# ---------------------------------------------------------------------------
def _poly_rows(preamble, phases=32, half=16):
    """Independent copy of the C++ core's 32-phase polyphase table."""
    pre = np.asarray(preamble, dtype=np.complex128)
    plen = pre.size
    M = 2 * half + 1
    rows = np.zeros((int(phases), plen), dtype=np.complex64)
    rows[0] = pre.astype(np.complex64)
    nn = np.arange(M, dtype=np.float64)
    win = 0.54 - 0.46 * np.cos(2.0 * np.pi * nn / (M - 1))
    for g in range(1, int(phases)):
        tau = float(g) / float(phases)
        taps = np.sinc(nn - float(half) - tau) * win
        ssum = taps.sum()
        if abs(ssum) > 1e-12:
            taps = taps / ssum
        conv = np.convolve(pre, taps.astype(np.complex128))
        rows[g] = conv[half:half + plen].astype(np.complex64)
    return rows


def _core_recon(model, preamble, lo, hi, rows, phases=32):
    """Independent core-operator reconstruction over [lo, hi)."""
    lo, hi = int(lo), int(hi)
    m = hi - lo
    if m <= 0:
        return np.zeros(0, dtype=np.complex128)
    pre = np.asarray(preamble, dtype=np.complex128)
    plen = pre.size
    acc = np.zeros(m, dtype=np.complex128)
    pos = np.arange(lo, hi, dtype=np.int64)
    for tau, g in zip(model["taus"], model["gains"]):
        tau = float(tau)
        ni = int(math.floor(tau))
        fr = tau - float(ni)
        if not (0.0 <= fr < 1.0):
            fr = 0.0
        beta = fr * float(phases)
        gi = min(int(beta), int(phases) - 1)
        alpha = min(max(beta - float(gi), 0.0), 1.0)
        idx = pos - ni
        okk = (idx >= 0) & (idx < plen)
        if not okk.any():
            continue
        ii = idx[okk]
        col = (1.0 - alpha) * rows[gi][ii].astype(np.complex128)
        if alpha != 0.0:
            if gi + 1 >= int(phases):
                jj = ii - 1
                okw = (jj >= 0) & (jj < plen)
                col = col.copy()
                col[okw] += alpha * rows[0][jj[okw]].astype(np.complex128)
            else:
                col = col + alpha * rows[gi + 1][ii].astype(np.complex128)
        acc[np.flatnonzero(okk)] += complex(g) * col
    w = 2.0 * math.pi * float(model["cfo_hz"]) / float(model["sample_rate"])
    p = pos.astype(np.float64) - float(model["phase_reference_sample"])
    return acc * np.exp(1j * w * p)


BUDGET_FILES = ("run_manifest.json", "error_budget_summary.json",
                "error_budget.jsonl", "budget_variants.csv")
VARIANT_TRUTH_FLAGS = {"S0_model": False}


def verify_budget(run_dir, verbose=False):
    """Verify an A-R03 error-budget run from its archived arrays."""
    del RESULTS[:]
    for f in BUDGET_FILES:
        if not os.path.exists(os.path.join(run_dir, f)):
            fail(("budget", "required file present: %s" % f))
    if any(not r[0] for r in RESULTS):
        return RESULTS
    man = load_json(os.path.join(run_dir, "run_manifest.json"))
    summ = load_json(os.path.join(run_dir, "error_budget_summary.json"))
    wins = load_jsonl(os.path.join(run_dir, "error_budget.jsonl"))
    ok(("budget", "task/stage are the A-R03 rectification run"),
       man.get("task") == "S2-A A-R03" and man.get("stage") == "rectification")
    lib = man.get("loaded_library") or {}
    ok(("budget", "library inside the worktree"),
       str(lib.get("expected_lib", "")).startswith(
           str((man.get("git") or {}).get("worktree", "")))
       and bool(lib.get("lib_exists")))
    iso = summ.get("truth_isolation") or {}
    ok(("budget", "truth isolation is declared"),
       iso.get("online_uses_truth") is False
       and iso.get("diagnostic_uses_truth") is True)
    ok(("budget", "mandated classes covered"),
       set(summ.get("mandated_classes") or []) <= set(summ.get("classes") or []))
    # per-window online/truth flags
    bad_flag = []
    for w in wins:
        for name, v in (w.get("variants") or {}).items():
            want = VARIANT_TRUTH_FLAGS.get(name, True)
            if bool(v.get("uses_truth")) != want:
                bad_flag.append((w["cell_id"], w["window_id"], name))
    ok(("budget", "only S0 is an online (truth-free) variant"), not bad_flag,
       str(bad_flag[:3]))
    # budget identity + reconstruction faithfulness
    bad_id, bad_recon = [], []
    for w in wins:
        b = w.get("budget_identity") or {}
        if b:
            rel = float(b.get("check_sum_identity_rel", 1.0))
            tot = float(b.get("e_total", 0.0))
            if b.get("e_impl_mismatch") is not None and tot > 0:
                parts = (float(b["e_impl_mismatch"]) + float(b["e_param"])
                         + float(b["e_cross"]))
                rel2 = abs(parts - tot) / tot
            else:
                rel2 = 0.0
            if rel > 1e-9 or rel2 > 1e-9:
                bad_id.append((w["cell_id"], w["window_id"], rel, rel2))
        rv = (w.get("reconstruction_validation") or {}).get(
            "nmse_diag_vs_adapter_ihat")
        if rv is not None and float(rv) > 1e-6:
            bad_recon.append((w["cell_id"], w["window_id"], rv))
    ok(("budget", "IQ budget identity e_total == ea+eb+cross"), not bad_id,
       str(bad_id[:3]))
    ok(("budget", "diagnostic reconstruction matches the adapter i_hat "
        "(<=1e-6 NMSE)"), not bad_recon, str(bad_recon[:3]))
    # independent recomputation from the archived arrays
    files = _archive_files(run_dir, "archive")
    ok(("budget", "archived windows present"), bool(files), str(len(files)))
    recs = {(w["cell_id"], w["window_id"]): w for w in wins}
    bad_va, bad_iq, bad_val = [], [], []
    rows_by_cell = {}
    for f in files:
        cell, _, rest = f.rpartition("_")
        wid = int(rest.split(".")[0])
        z = np.load(os.path.join(run_dir, "archive", f), allow_pickle=False)
        prm = json.loads(bytes(z["params_json"]).decode("utf-8"))
        preamble = None
        if cell not in rows_by_cell:
            pf = os.path.join(run_dir, "templates", "comm_preamble_code10_998p4.cf32")
            if os.path.exists(pf):
                rows_by_cell[cell] = _poly_rows(
                    np.fromfile(pf, dtype=np.complex64))
                preamble = np.fromfile(pf, dtype=np.complex64)
        if preamble is None:
            continue
        rows = rows_by_cell[cell]
        clean = np.asarray(z["clean"], dtype=np.complex128)
        raw = np.asarray(z["raw"], dtype=np.complex128)
        sic = np.asarray(z["sic"], dtype=np.complex128)
        rec = recs.get((cell, wid))
        if rec is None:
            continue
        fl, fh = [int(x) for x in prm["region"]]
        ih = np.asarray(z["ihat"], dtype=np.complex128)[fl:fh]
        model = {"taus": [float(t) for t in prm["model_taus"]],
                 "gains": [complex(a, b) for a, b in prm["model_gains"]],
                 "cfo_hz": float(prm["model_cfo_hz"]),
                 "phase_reference_sample": int(prm["model_phase_reference"]),
                 "sample_rate": 998400000.0, "n_paths": len(prm["model_taus"])}
        mine = _core_recon(model, preamble, fl, fh, rows)
        den = max(1e-30, float(np.max(np.abs(ih))))
        if float(np.max(np.abs(mine - ih))) / den > 1e-4:
            bad_val.append((cell, wid, "model_vs_ihat"))
        # recompute one truth-assisted variant's masked IQ energy
        vname = "S6_full_truth"
        vm = (prm.get("variant_models") or {}).get(vname)
        if vm:
            vmod = {"taus": [float(t) for t in vm["taus"]],
                    "gains": [complex(a, b) for a, b in vm["gains"]],
                    "cfo_hz": float(vm["cfo_hz"]),
                    "phase_reference_sample": int(vm["phase_reference_sample"]),
                    "sample_rate": 998400000.0,
                    "n_paths": len(vm["taus"])}
            recom = _core_recon(vmod, preamble, int(prm["sub_lo"]),
                                int(prm["sub_hi"]), rows)
            i_true = np.asarray(z["i_truth"], dtype=np.complex128)[
                int(prm["sub_lo"]):int(prm["sub_hi"])]
            e_iq = float(np.sum(np.abs(i_true - recom) ** 2))
            want = (rec.get("variants") or {}).get(vname, {}).get(
                "iq_residual_energy_vs_truth")
            if want is None or abs(e_iq - float(want)) > 1e-6 * max(1.0, e_iq):
                bad_iq.append((cell, wid, e_iq, want))
    ok(("budget", "published model reproduced from arrays + code table"),
       not bad_val, str(bad_val[:3]))
    ok(("budget", "S6 truth-variant IQ energy recomputed independently"),
       not bad_iq, str(bad_iq[:3]))
    return RESULTS


def budget_reject_qa(src_run, workdir, verbose=False):
    """Mutations of a real budget run; each must be rejected."""
    import shutil as _sh
    mutations = []

    def mutation(name):
        def deco(fn):
            mutations.append((name, fn))
            return fn
        return deco

    @mutation("flip a variant's uses_truth flag")
    def _m1(dst):
        p = os.path.join(dst, "error_budget.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        for r in recs:
            if "S6_full_truth" in r.get("variants", {}):
                r["variants"]["S6_full_truth"]["uses_truth"] = False
                break
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True, default=str) + "\n")

    @mutation("break the IQ budget identity")
    def _m2(dst):
        p = os.path.join(dst, "error_budget.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        recs[0]["budget_identity"]["e_param"] = (
            float(recs[0]["budget_identity"]["e_param"]) * 1.5)
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True, default=str) + "\n")

    @mutation("tamper a S6 variant IQ energy")
    def _m3(dst):
        p = os.path.join(dst, "error_budget.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        for r in recs:
            v = r.get("variants", {}).get("S6_full_truth")
            if v:
                v["iq_residual_energy_vs_truth"] = float(
                    v["iq_residual_energy_vs_truth"]) * 10.0 + 1.0
                break
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True, default=str) + "\n")

    @mutation("claim online metrics used truth")
    def _m4(dst):
        p = os.path.join(dst, "error_budget_summary.json")
        s = json.load(open(p))
        s["truth_isolation"]["online_uses_truth"] = True
        json.dump(s, open(p, "w"), indent=2, allow_nan=True)

    @mutation("inflate the recorded reconstruction NMSE")
    def _m5(dst):
        p = os.path.join(dst, "error_budget.jsonl")
        recs = [json.loads(x) for x in open(p).read().splitlines() if x]
        recs[0]["reconstruction_validation"][
            "nmse_diag_vs_adapter_ihat"] = 0.5
        with open(p, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r, allow_nan=True, default=str) + "\n")

    @mutation("tamper the archived model taus")
    def _m6(dst):
        d = os.path.join(dst, "archive")
        f = sorted(x for x in os.listdir(d) if x.endswith(".npz"))[0]
        z = dict(np.load(os.path.join(d, f), allow_pickle=False))
        prm = json.loads(bytes(z["params_json"]).decode("utf-8"))
        prm["model_taus"] = [float(t) + 3.0 for t in prm["model_taus"]]
        z["params_json"] = np.frombuffer(json.dumps(prm).encode("utf-8"),
                                         dtype=np.uint8)
        np.savez_compressed(os.path.join(d, f), **z)

    uncaught = []
    for i, (name, fn) in enumerate(mutations):
        dst = os.path.join(workdir, "b%02d" % i)
        if os.path.exists(dst):
            _sh.rmtree(dst)
        _sh.copytree(src_run, dst)
        try:
            fn(dst)
        except Exception as exc:
            uncaught.append((name, "mutation failed: %r" % (exc,)))
            continue
        res = verify_budget(dst, verbose=False)
        caught = any(not r[0] for r in res)
        if verbose:
            print("%-6s budget mutation: %s" % ("CAUGHT" if caught else "MISSED",
                                                name))
        if not caught:
            uncaught.append((name, "verifier accepted the mutated artifact"))
        _sh.rmtree(dst, ignore_errors=True)
    return uncaught, len(mutations)


# ---------------------------------------------------------------------------
# Self-test of the verifier primitives
# ---------------------------------------------------------------------------
def self_test(verbose=True):
    fails = 0

    def check(name, cond, detail=""):
        nonlocal fails
        if verbose:
            print("%-58s %s %s" % (name, "PASS" if cond else "FAIL", detail))
        if not cond:
            fails += 1

    # sampled code layout: chip j on sample 8*j
    chips = np.array([1, -1, 0, 1], dtype=np.int8)
    code = build_sampled_code(chips, sps=32)
    check("chips land on stride-8 samples",
          np.allclose(code[::8][:4], chips) and np.count_nonzero(code) == 3)
    # matched filter vs a brute-force double loop
    rng = np.random.default_rng(3)
    y = (rng.standard_normal(40) + 1j * rng.standard_normal(40))
    c = np.array([1.0 + 0j, 0, -1.0 + 0j, 0.5j, 0, 0, 0.25 + 0j])
    mf = matched_filter(y, c, float(np.real(np.vdot(c, c))))
    brute = []
    for k in range(len(y) - len(c) + 1):
        brute.append(sum(y[k + m] * np.conj(c[m]) for m in range(len(c)))
                     / float(np.real(np.vdot(c, c))))
    check("matched filter == brute force", np.allclose(mf, np.array(brute)))
    # accumulation and 1/N behaviour
    H = np.ones((16, 4), dtype=np.complex128)
    check("average convention",
          np.allclose(accum(H, 0, 4), 1.0) and np.allclose(
              accum(H, 0, 16) * 16, H.sum(axis=0)))
    check("C_CIR known scale = 6.0206 dB",
          abs(ratio_db(4.0, 1.0)[0] - 20 * math.log10(2)) < 1e-12)
    check("0/0 -> N/A", ratio_db(0.0, 0.0) == (None, "na"))
    check("positive numerator -> +inf", ratio_db(1.0, 0.0)[1] == "pos_inf")
    check("negative numerator -> -inf", ratio_db(0.0, 1.0)[1] == "neg_inf")
    # coherence references
    const = np.ones((32, 2), dtype=np.complex128)
    check("coherent constant residual: eta==1",
          abs(coherence(const, 0, 2)["eta"] - 1.0) < 1e-12,
          "%.6f" % coherence(const, 0, 2)["eta"])
    ramp = np.exp(1j * 0.4 * np.arange(32))[:, None] * np.ones((1, 2))
    check("coherent ramp: centered lag-1 ~1 (eta alone would be misleading)",
          abs(coherence(ramp, 0, 2)["centered_lag1"] - 1.0) < 1e-9,
          "eta=%.5f clag1=%.6f" % (coherence(ramp, 0, 2)["eta"],
                                   coherence(ramp, 0, 2)["centered_lag1"]))
    noise = (rng.standard_normal((64, 2))
             + 1j * rng.standard_normal((64, 2)))
    check("independent residual: eta near 1/N",
          coherence(noise, 0, 2)["eta"] < 6.0 / 64,
          "%.4f" % coherence(noise, 0, 2)["eta"])
    check("zero-energy coherence -> N/A",
          coherence(np.zeros((8, 2)), 0, 2)["eta"] is None)
    # A-R01: independent derivation of the support-disjoint run
    run, excluded = _legal_run(0, 1008, 1997, 1016, 64, -32, 128, 10126,
                               1997, 67022, 157376)
    check("support-disjoint run is [9,62] on the real geometry",
          run and run[0] == 9 and run[-1] == 62 and len(run) == 54,
          "[%s,%s] n=%d" % (run[0] if run else -1, run[-1] if run else -1,
                            len(run)))
    check("r=8 excluded as training/guard read and r=63 as tail read",
          8 in excluded and "reads_training_or_guard" in excluded[8]
          and 63 in excluded
          and "outside_applied_subtraction" in excluded[63])
    check("honest N list truncates 56 -> 54",
          _truncate_n(54, [1, 2, 4, 8, 16, 32, 56])
          == [1, 2, 4, 8, 16, 32, 54])
    check("short independent region truncates without padding",
          _truncate_n(7, [1, 2, 4, 8, 16, 32, 56]) == [1, 2, 4])
    # A-R02: same-dimension energy decomposition
    E = np.ones((16, 3), dtype=np.complex128)
    N = E.shape[0]
    mu = E.mean(axis=0)
    ec = float(N * np.sum(np.abs(mu) ** 2))
    ez = float(np.sum(np.abs(E - mu) ** 2))
    check("e_coherent = N*||mu||^2 and e_centered = 0 for repeats",
          abs(ec - float(np.sum(np.abs(E) ** 2))) < 1e-12 and ez < 1e-24,
          "%.6g %.3g" % (ec, ez))
    check("coherent_fraction == eta == e_coherent/e_total",
          abs(ec / (ec + ez) - 1.0) < 1e-12)
    print("self-test: %d failure(s)" % fails)
    return fails


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--run", action="append", default=None,
                    help="run directory to verify (repeatable)")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--reject-qa", action="store_true",
                    help="mutate --run and require every mutation to be caught")
    ap.add_argument("--budget", action="append", default=None,
                    help="A-R03 error-budget run directory to verify")
    ap.add_argument("--reject-qa-budget", action="store_true",
                    help="mutate --budget and require every mutation to be "
                         "caught")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(True)
    if not args.run and not args.budget:
        ap.error("--run and/or --budget is required (or --self-test)")
    rc = 0
    for run in (args.run or []):
        print("== verifying %s" % run)
        res = verify_run(run, verbose=args.verbose)
        nf = print_results(res, verbose=args.verbose)
        rc = rc or (1 if nf else 0)
    for bdir in (args.budget or []):
        print("== verifying budget %s" % bdir)
        res = verify_budget(bdir, verbose=args.verbose)
        nf = print_results(res, verbose=args.verbose)
        rc = rc or (1 if nf else 0)
    if args.reject_qa_budget:
        work = tempfile.mkdtemp(prefix="s2a_reject_budget_")
        try:
            for bdir in (args.budget or []):
                print("== budget reject-QA on %s" % bdir)
                uncaught, total = budget_reject_qa(bdir, work, verbose=True)
                print("budget reject-QA: %d/%d caught"
                      % (total - len(uncaught), total))
                for name, why in uncaught:
                    print("MISSED: %s (%s)" % (name, why))
                if uncaught:
                    rc = 1
        finally:
            shutil.rmtree(work, ignore_errors=True)
    if args.reject_qa:
        work = tempfile.mkdtemp(prefix="s2a_reject_")
        try:
            for run in args.run:
                print("== reject-QA on %s" % run)
                uncaught, total = reject_qa(run, work, verbose=True)
                print("reject-QA: %d/%d caught"
                      % (total - len(uncaught), total))
                for name, why in uncaught:
                    print("MISSED: %s (%s)" % (name, why))
                if uncaught:
                    rc = 1
        finally:
            shutil.rmtree(work, ignore_errors=True)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
