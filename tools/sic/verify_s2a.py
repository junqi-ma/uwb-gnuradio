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
       and sets.get("indep_start") == 8
       and sets.get("indep_N") == [1, 2, 4, 8, 16, 32, 56])
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


def _archive_files(run_dir):
    d = os.path.join(run_dir, "archive")
    if not os.path.isdir(d):
        return []
    return sorted(f for f in os.listdir(d) if f.endswith(".npz"))


def check_archive_policy(art):
    run_dir = art["dir"]
    decl = (art["summary"].get("archive") or {})
    files = _archive_files(run_dir)
    ok(("archive", "declared count == files on disk"),
       int(decl.get("archived_windows", -1)) == len(files),
       "%s vs %d" % (decl.get("archived_windows"), len(files)))
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


def check_archived_windows(art, verbose=False):
    """Recompute H, the accumulation table, target and coherence from arrays."""
    run_dir = art["dir"]
    o = art["operator"]["operator"]
    r = art["operator"]["roi"]
    sets = art["operator"]["repetition_sets"]
    sps = int(o["sps"])
    origin = int(o["origin"])
    reps = int(o["sync_reps"])
    tap_lo = int(o["tap_lo"])
    tap_hi = int(o["tap_hi"])
    code = np.asarray(art["operator_arrays"]["sampled_code"],
                      dtype=np.complex128)
    ecode = float(np.real(np.vdot(code, code)))
    # ROI indices in the archived tap grid
    tgt_lo = int(r["target_offsets"][0]) - tap_lo
    tgt_hi = int(r["target_offsets"][1]) - tap_lo
    bg_lo = int(r["background_offsets"][0]) - tap_lo
    bg_hi = int(r["background_offsets"][1]) - tap_lo
    tgt_tap = int(r["target_tap_offset"]) - tap_lo
    recs = {(w["cell_id"], w["window_id"]): w for w in art["windows"]}
    files = _archive_files(run_dir)
    if not files:
        fail(("archive", "recompute: no archived windows"))
        return
    h_bad, tab_bad, tgt_bad, coh_bad, miss = [], [], [], [], []
    for f in files:
        cell, _, rest = f.rpartition("_")
        wid = int(rest.split(".")[0])
        z = np.load(os.path.join(run_dir, "archive", f), allow_pickle=False)
        clean = np.asarray(z["clean"], dtype=np.complex128)
        raw = np.asarray(z["raw"], dtype=np.complex128)
        sic = np.asarray(z["sic"], dtype=np.complex128)
        Hc = cir_tensor(matched_filter(clean, code, ecode), origin, sps, reps,
                        tap_lo, tap_hi)
        Hr = cir_tensor(matched_filter(raw, code, ecode), origin, sps, reps,
                        tap_lo, tap_hi)
        Hs = cir_tensor(matched_filter(sic, code, ecode), origin, sps, reps,
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
        # accumulation table
        for set_name, n_list in (("full", sets["full_N"]),
                                 ("indep", sets["indep_N"])):
            start = 0 if set_name == "full" else int(sets["indep_start"])
            e1r = e1s = None
            for N in n_list:
                hc = accum(Hc, start, N)
                hr = accum(Hr, start, N)
                hs = accum(Hs, start, N)
                er = energy(hr - hc)
                es = energy(hs - hc)
                c, flag = ratio_db(er, es)
                got = rec["accum"][set_name][str(N)]
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
                if N == 1:
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
                # target protection
                t = rec["target"]["%s/%d" % (set_name, N)]["sic"]
                tr = rec["target"]["%s/%d" % (set_name, N)]["raw"]
                seg = np.abs(hs[tgt_lo:tgt_hi])
                pk = int(np.argmax(seg)) + tgt_lo
                pkc = int(np.argmax(np.abs(hc[tgt_lo:tgt_hi]))) + tgt_lo
                if pk - pkc != int(t["peak_shift"]):
                    tgt_bad.append((cell, wid, set_name, N, "peak_shift"))
                amp = abs(hs[tgt_tap]) / abs(hc[tgt_tap])
                want = 20.0 * math.log10(amp) if amp > 0 else None
                if want is not None and abs(want - float(
                        t["amp_err_db"])) > 1e-6 * max(1.0, abs(want)):
                    tgt_bad.append((cell, wid, set_name, N, "amp_err"))
                amp_r = abs(hr[tgt_tap]) / abs(hc[tgt_tap])
                want_r = 20.0 * math.log10(amp_r) if amp_r > 0 else None
                if want_r is not None and abs(want_r - float(
                        tr["amp_err_db"])) > 1e-6 * max(1.0, abs(want_r)):
                    tgt_bad.append((cell, wid, set_name, N, "raw_amp_err"))
        # coherence (largest-N single-window statistics)
        E = Hs - Hc
        for roi, (lo, hi) in (("target", (tgt_lo, tgt_hi)),
                              ("background", (bg_lo, bg_hi)),
                              ("all", (0, Hs.shape[1]))):
            c = coherence(E, lo, hi)
            got = rec["coherence"]["sic"][roi]
            if c["eta"] is None:
                if got["eta"] is not None:
                    coh_bad.append((cell, wid, roi, "eta_na"))
            else:
                if abs(float(got["eta"]) - c["eta"]) > 1e-9:
                    coh_bad.append((cell, wid, roi, "eta"))
                if c["lag1"] is not None and abs(
                        float(got["lag"].get("1")) - c["lag1"]) > 1e-9:
                    coh_bad.append((cell, wid, roi, "lag1"))
                if c["centered_lag1"] is not None and abs(
                        float(got["centered_lag"].get("1"))
                        - c["centered_lag1"]) > 1e-9:
                    coh_bad.append((cell, wid, roi, "centered_lag1"))
    ok(("recompute", "archived H_clean/H_raw/H_sic reproduced (%d windows)"
        % len(files)), not h_bad, str(h_bad[:4]))
    ok(("recompute", "archived accumulation tables reproduced"), not tab_bad,
       str(tab_bad[:4]))
    ok(("recompute", "archived target metrics reproduced"), not tgt_bad,
       str(tgt_bad[:4]))
    ok(("recompute", "archived coherence metrics reproduced"), not coh_bad,
       str(coh_bad[:4]))
    ok(("recompute", "every archived window has a jsonl record"), not miss,
       str(miss[:4]))


def check_summary(art):
    s = art["summary"]
    wins = art["windows"]
    op = art["operator"]["seeds"]
    sets = art["operator"]["repetition_sets"]
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
        for set_name, n_list in (("full", sets["full_N"]),
                                 ("indep", sets["indep_N"])):
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
                # accumulation gain
                for gname in ("g_sic_db", "g_raw_db"):
                    vals = [r["accum"][set_name][str(N)][gname] for r in recs]
                    m = mean_of(vals)
                    if (m is None) != (cs["accum"][key][gname]["mean"] is None):
                        bad.append((cell, key, gname, "none"))
                    elif m is not None and abs(
                            m - float(cs["accum"][key][gname]["mean"])) > 1e-6:
                        bad.append((cell, key, gname, "mean"))
                # target protection
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
    ok(("summary", "per-cell aggregates recomputed from windows.jsonl"),
       not bad, str(bad[:6]))
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
                    # an N/A cell must never carry a fabricated number
                    if got not in ("na", "", "None"):
                        badcsv.append((cell, key, roi, "fabricated_mean"))
                elif got in ("na", "", "None") or abs(
                        float(got) - float(want)) > 1e-6:
                    badcsv.append((cell, key, roi, "mean"))
    ok(("summary", "cells_summary.csv agrees with summary.json"), not badcsv,
       str(badcsv[:5]))


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
               check_archive_policy, check_windows, check_archived_windows,
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
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(True)
    if not args.run:
        ap.error("--run is required (or --self-test)")
    rc = 0
    for run in args.run:
        print("== verifying %s" % run)
        res = verify_run(run, verbose=args.verbose)
        nf = print_results(res, verbose=args.verbose)
        rc = rc or (1 if nf else 0)
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
