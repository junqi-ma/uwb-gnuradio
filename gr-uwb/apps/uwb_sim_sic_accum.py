#!/usr/bin/env python3
"""S2-A: accumulation-after-SIC sensing CIR evaluation driver.

Research question (``docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`` section 1):
does the ~28 dB nominal raw-IQ suppression of the sensing preamble survive
coherent accumulation over ``N`` Ipatov SYNC repetitions, does it protect the
weak target, and does the residual decay like an incoherent (1/N) term or hit a
coherent floor built from frozen-model / CFO error?

Two layers are reported separately and never mixed:

* **fixed-operator layer** (section 3.1, the causal result): one linear de-spread
  operator (code-9 sampled code, fixed origin / tap grid / gain normalisation,
  NO phase rotation, NO per-branch picking) applied identically to the paired
  ``clean = s + w``, ``raw = s + i + w`` and ``sic = raw - i_hat`` windows.
  This is the main evaluation.
* **receiver layer** (section 3.2, engineering result): the existing
  ``uwb.radar_cir_estimator`` GNU Radio block running unmodified on all three
  branches; its own sync / SFD search / L2 normalisation is allowed and the
  per-branch outcome / failure is recorded.  The receiver repetition selection
  is a block-construction parameter (``skip``/``count``), so any N sweep here is
  the receiver's own "last N" set -- NOT the fixed-operator ``[0,N)`` /
  ``[8,8+N)`` sets -- and is labelled as such.

The SIC branch always comes from the real GNU Radio PDU adapter
(``uwb.preamble_sic``, S1 ``estimate8_frozen`` default), never from a NumPy
re-implementation.  Only the offline metrics after the adapter are NumPy.

Usage
-----
    # QA / design checks (no build tree needed)
    python3 gr-uwb/apps/uwb_sim_sic_accum.py --self-test

    # A1 smoke: nominal cell, 8 windows, full arrays archived
    python3 gr-uwb/apps/uwb_sim_sic_accum.py --build-dir gr-uwb/build-sic-accum \
        --output gr-uwb/build-sic-accum/results/s2a/smoke --smoke \
        --archive-windows 8

    # A2 formal acceptance: 12 cells x 100 windows (calibration first!)
    python3 gr-uwb/apps/uwb_sim_sic_accum.py --build-dir gr-uwb/build-sic-accum \
        --output gr-uwb/build-sic-accum/results/s2a/acceptance \
        --seed-set acceptance --windows-per-cell 100 --archive-windows 8

    # A3: 1000 no-interference bypass / target-protection windows
    python3 gr-uwb/apps/uwb_sim_sic_accum.py --build-dir gr-uwb/build-sic-accum \
        --output gr-uwb/build-sic-accum/results/s2a/bypass --bypass \
        --windows-per-cell 1000 --archive-windows 8
"""

from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import math
import os
import platform
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.abspath(
    os.path.join(_HERE, os.pardir, os.pardir, "tools", "sic")))

from gnuradio import gr  # noqa: E402
import pmt  # noqa: E402

import sic_preamble_sim as S  # noqa: E402
import uwb_sim_preamble_sic as APP  # noqa: E402
import accum_metrics as AM  # noqa: E402

SCHEMA_VERSION = 2
SPS = S.SPS
SFD_MODE = S.SFD_MODE
BASE_COMMIT = "93c80b3e77c2caf53cdfab25e09683871806fad8"


# ---------------------------------------------------------------------------
# Repo paths / provenance
# ---------------------------------------------------------------------------
def repo_root() -> str:
    return os.path.abspath(os.path.join(_HERE, os.pardir, os.pardir))


def testdata_dir(*parts) -> str:
    return os.path.join(repo_root(), "testdata", "sic_accum", *parts)


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for blk in iter(lambda: fh.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def git_provenance(root: str) -> dict:
    def run(*a):
        try:
            return subprocess.run(a, cwd=root, capture_output=True, text=True,
                                  check=False).stdout.strip()
        except OSError:
            return ""
    return {"commit": run("git", "rev-parse", "HEAD"),
            "branch": run("git", "rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(run("git", "status", "--porcelain")),
            "worktree": root}


def library_provenance(build_dir: str) -> dict:
    lib = os.path.abspath(os.path.join(build_dir, "lib",
                                       "libgnuradio-uwb.so.1.0.0"))
    out = {"expected_lib": lib, "lib_exists": os.path.exists(lib),
           "build_dir": os.path.abspath(build_dir)}
    mods = sorted(glob.glob(os.path.join(build_dir, "python", "uwb",
                                         "bindings", "uwb_python*.so")))
    out["module"] = os.path.abspath(mods[-1]) if mods else None
    for tag, path in (("lib", lib), ("module", out["module"])):
        if not path:
            out["ldd_" + tag] = []
            continue
        try:
            r = subprocess.run(["ldd", path], capture_output=True, text=True,
                               check=False)
            lines = (r.stdout + r.stderr).splitlines()
            out["ldd_" + tag] = [ln.strip() for ln in lines
                                 if "gnuradio-uwb" in ln]
        except OSError:
            out["ldd_" + tag] = []
    out["in_worktree"] = out["expected_lib"].startswith(
        os.path.abspath(repo_root()))
    return out


# ---------------------------------------------------------------------------
# PMT helpers
# ---------------------------------------------------------------------------
def put_int(d, k, v):
    return pmt.dict_add(d, pmt.intern(k), pmt.from_long(int(v)))


def put_str(d, k, v):
    return pmt.dict_add(d, pmt.intern(k), pmt.intern(str(v)))


def put_float(d, k, v):
    return pmt.dict_add(d, pmt.intern(k), pmt.from_double(float(v)))


def dget(d, k, default=None):
    """Read a key from an adapter out-meta dict (PMT-valued, see ResultSink)."""
    return APP.dget(d, k, default)


def dnum(d, k, default=0.0):
    v = APP.dget(d, k, None)
    if v is None:
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


# ---------------------------------------------------------------------------
# Minimal GNU Radio runners (thin wrappers; no algorithm changes)
# ---------------------------------------------------------------------------
class SicRunner:
    """One ``uwb.preamble_sic`` PDU adapter + bounded result sinks."""

    def __init__(self, uwb, params, extras, name="s2a_sic"):
        self.tb = gr.top_block(name)
        self.sic, self.used = APP.call_sic_make_from_templates(
            uwb, params, extras)
        self.sink_out = APP.ResultSink("sic_out", 0)
        self.sink_ihat = APP.ResultSink("sic_ihat", 0)
        self.tb.msg_connect((self.sic, "out"), (self.sink_out, "in"))
        self.tb.msg_connect((self.sic, "i_hat"), (self.sink_ihat, "in"))
        self.tb.start()

    def post(self, meta, iq):
        # The expected completion count MUST be taken BEFORE posting: the
        # worker can finish (and bump the sink count) before we call wait().
        self._expect = (self.sink_out.count + 1, self.sink_ihat.count + 1)
        self.sic.to_basic_block()._post(
            pmt.intern("in"),
            pmt.cons(meta, pmt.init_c32vector(len(iq), iq.tolist())))

    def wait(self, deadline_s=180.0):
        n_out, n_ih = getattr(self, "_expect", (1, 1))
        t0 = time.time()
        while time.time() - t0 < deadline_s:
            o, om = self.sink_out.snapshot()
            h, hm = self.sink_ihat.snapshot()
            if o >= n_out and h >= n_ih:
                return om, hm
            time.sleep(0.002)
        return None

    def stop(self):
        self.tb.stop()
        self.tb.wait()


class CirRunner:
    """One ``uwb.radar_cir_estimator`` with a configurable repetition set."""

    def __init__(self, uwb, tpl_path, *, sync_reps=64, cir_pre=16, cir_post=100,
                 skip=10, count=0, name="s2a_cir"):
        self.tb = gr.top_block(name)
        self.cir = uwb.radar_cir_estimator(
            tpl_path, int(sync_reps), SFD_MODE, S.DEFAULT_SENSING_CODE, int(cir_pre),
            int(cir_post), int(skip), int(count), 128, 32, 0.12, 0.02, True, 8,
            False)
        self.skip = int(skip)
        self.count = int(count)
        self.sink = APP.ResultSink("cir_sink", 0)
        self.status = APP.ResultSink("cir_status", 0)
        self.tb.msg_connect((self.cir, "cir"), (self.sink, "in"))
        self.tb.msg_connect((self.cir, "status"), (self.status, "in"))
        self.tb.start()

    def last_status(self):
        _n, m = self.status.snapshot()
        return None if m is None else {
            k: APP.pmt_to_py(v) for k, v in m.items() if k != "__t_ns"}

    def post(self, meta, iq):
        self._expect = self.sink.count + 1
        self.cir.to_basic_block()._post(
            pmt.intern("rx"),
            pmt.cons(meta, pmt.init_c32vector(len(iq), iq.tolist())))

    def wait(self, deadline_s=180.0):
        n = getattr(self, "_expect", 1)
        t0 = time.time()
        while time.time() - t0 < deadline_s:
            c, m = self.sink.snapshot()
            if c >= n:
                return m
            time.sleep(0.002)
        return None

    def stop(self):
        self.tb.stop()
        self.tb.wait()


# ---------------------------------------------------------------------------
# Frozen operator construction
# ---------------------------------------------------------------------------
def build_operator(uwb, generator, frozen: dict):
    """Build the frozen fixed-operator from public generator data.

    Fails closed if the recovered code is not an exact ternary 64-chip code or
    if the public origin drifted from the frozen value.
    """
    taps = S.pulse_taps(uwb, generator.pulse_shape, 2.5, 200.0)
    if taps is None:
        raise SystemExit("cannot materialise the pulse-shaping taps; the fixed "
                         "operator must come from the public modulator")
    chips, diag = AM.recover_chips(generator.template, taps)
    if (diag["rank"] != AM.PREAMBLE_CODE_LEN or diag["resid_rel"] > 1e-6
            or diag["max_int_dev"] > 1e-3 or diag["nonzero_chips"] != 64
            or diag["chip_energy"] != 64):
        raise SystemExit("code recovery is not exact: %r" % (diag,))
    code = AM.sampled_code_from_chips(chips)
    if int(generator.pre) != int(frozen["operator"]["origin"]):
        raise SystemExit("public sensing origin %d != frozen %d"
                         % (generator.pre, frozen["operator"]["origin"]))
    op = AM.Operator(origin=int(generator.pre), code=code,
                     energy=AM.code_energy(code),
                     tap_lo=int(frozen["operator"]["tap_lo"]),
                     tap_hi=int(frozen["operator"]["tap_hi"]),
                     sync_reps=int(generator.sync_reps), sps=SPS)
    ev = {"recovery": diag, "origin": int(generator.pre),
          "code_energy": float(op.energy),
          "code_nonzero": int(np.count_nonzero(code)),
          "code_len": int(code.size), "pulse_taps": int(taps.size),
          "template_sha256": S.sha256_array(generator.template),
          "sampled_code_sha256": AM.sha256_f32(code),
          "chips": [int(c) for c in chips]}
    return op, ev


def verify_target_roi(generator, op, frozen, comm_present):
    """Re-verify the frozen weak-target ROI on calibration-only seeds.

    The ROI is NOT searched on SIC results: this check only confirms that the
    public sensing profile still puts the weak (delay 37, gain 0.25) peak on the
    frozen fixed-operator tap.  It fails closed on any drift.
    """
    seeds = [int(s) for s in frozen["roi"]["roi_freeze_seeds"]]
    peaks, directs, ratios = [], [], []
    for s in seeds:
        win = generator.build(seed=s, comm_paths=[[0, [1.0, 0.0]]],
                              start_offset_work=0.0, cfo_hz=0.0,
                              isr_db=10.0 if comm_present else None)
        clean = np.ascontiguousarray(win["s"] + win["w"], dtype=np.complex64)
        H = AM.fixed_operator_cir(clean, op)
        hc = AM.accumulate(H, AM.rep_indices(True, op.sync_reps))
        lo = AM.TARGET_LO - op.tap_lo
        hi = AM.TARGET_HI - op.tap_lo
        pk = int(np.argmax(np.abs(hc[lo:hi])) + AM.TARGET_LO)
        dlo = AM.DIRECT_LO - op.tap_lo
        dhi = AM.DIRECT_HI - op.tap_lo
        dp = int(np.argmax(np.abs(hc[dlo:dhi])) + AM.DIRECT_LO)
        peaks.append(pk)
        directs.append(dp)
        ratios.append(float(abs(hc[AM.TARGET_OFFSET - op.tap_lo])
                            / abs(hc[dp - op.tap_lo])))
    ev = {"seeds": seeds, "target_peak_taps": peaks,
          "direct_peak_taps": directs,
          "target_over_direct_amp": ratios,
          "frozen_target_tap": AM.TARGET_OFFSET,
          "expected_path_gain_ratio": 0.25}
    ok = (len(set(peaks)) == 1 and peaks[0] == AM.TARGET_OFFSET
          and all(abs(r - 0.25) <= 0.05 for r in ratios))
    ev["pass"] = bool(ok)
    if not ok:
        raise SystemExit("weak-target ROI verification failed: %r" % (ev,))
    return ev


# ---------------------------------------------------------------------------
# Per-window evaluation
# ---------------------------------------------------------------------------
def window_seed(base: int, cell_index: int, window_index: int,
                stride: int = 1000) -> int:
    return int(base) + int(cell_index) * int(stride) + int(window_index)


def rx_meta(wid, n, sync_reps, pre_guard):
    """PDU meta accepted by both the SIC adapter and the radar CIR estimator.

    The CIR block rejects any PDU without ``sample_rate`` / ``pulse_id`` /
    ``pre_guard_samples`` / ``calibration_delay_*``; the SIC adapter ignores the
    keys it does not use.  All values are public plan quantities.
    """
    m = pmt.make_dict()
    m = put_int(m, "window_id", int(wid))
    m = put_int(m, "pulse_id", int(wid) + 1)
    m = put_int(m, "schedule_index", int(wid))
    m = put_int(m, "sample_count", int(n))
    m = put_int(m, "code_index", S.DEFAULT_SENSING_CODE)
    m = put_int(m, "sync_repetitions", int(sync_reps))
    m = put_str(m, "sfd_mode", SFD_MODE)
    m = put_str(m, "sensing_profile_id",
                "code%d-%dSYNC-%s" % (S.DEFAULT_SENSING_CODE, int(sync_reps),
                                      SFD_MODE))
    m = put_str(m, "template_id",
                "sync_template_code%d_998p4" % S.DEFAULT_SENSING_CODE)
    m = put_float(m, "sample_rate", S.WORK_HZ)
    m = put_int(m, "window_start_sample", 0)
    m = put_int(m, "pre_guard_samples", int(pre_guard))
    m = put_float(m, "calibration_delay_native_samples", 0.0)
    return m


def _rs_summary(rs):
    """Compact JSON-safe summary of a repetition set (no numpy arrays)."""
    return {k: rs.get(k) for k in
            ("name", "kind", "strict_holdout", "tap_lo", "tap_hi", "available",
             "first", "last", "n_list", "n_nominal", "n_shortened",
             "read_lo", "read_hi", "support_source", "nz_support", "bounds",
             "excluded_before", "excluded_after", "dropped_noncontiguous",
             "superseded_by", "note")}


def evaluate_window(op, frozen, cell, wid, seed, kind, win, clean, raw, sic_iq,
                    out_meta, cir_runners=None, support=None):
    """Compute every fixed-operator S2-A metric for one window.

    Returns ``(record, H_clean, H_raw, H_sic)``; the CIR tensors are returned so
    archiving never recomputes them.
    """
    n = int(raw.size)
    origin = int(op.origin)
    span_hi = min(n, origin + int(op.sync_reps) * SPS)

    bypass_identical = APP.iq_bit_identical(raw, sic_iq)
    committed = bool(dget(out_meta, "sic_committed", False))
    cfo_est = float(dnum(out_meta, "sic_cfo_hz", 0.0))
    uses_truth = bool(dget(out_meta, "sic_uses_truth", False))

    H_clean = AM.fixed_operator_cir(clean, op)
    H_raw = AM.fixed_operator_cir(raw, op)
    H_sic = AM.fixed_operator_cir(sic_iq, op)

    # ---- A-R01: actual sample support / repetition sets -------------------
    nz = AM.code_support(op.code)
    vis_meta = int(dnum(out_meta, "sic_visible_end_sample", 0))
    visible_end = vis_meta if vis_meta > 0 else (
        int(support["bounds"]["visible_end"]) if support else n)
    sub_lo = int(dnum(out_meta, "sic_sub_lo", -1))
    sub_hi = int(dnum(out_meta, "sic_sub_hi", -1))
    train_lo = int(dnum(out_meta, "sic_training_lo", -1))
    train_hi = int(dnum(out_meta, "sic_training_hi", -1))
    model_origin = int(dnum(out_meta, "sic_detected_start", -1))
    has_sub = bool(sub_lo >= 0 and sub_hi > sub_lo)
    if has_sub:
        win_set = AM.derive_rep_set(
            op, visible_end=visible_end, sub_lo=sub_lo, sub_hi=sub_hi, n=n,
            tap_lo=op.tap_lo, tap_hi=op.tap_hi, name="support_disjoint",
            kind="support_disjoint")
    else:
        # bypass / failed window: the adapter applied nothing, so the window
        # has no actual subtraction region.  The analysis still uses the
        # PUBLIC nominal cell set and the record says so explicitly.
        win_set = dict(support) if support else AM.derive_rep_set(
            op, visible_end=visible_end if visible_end > 0 else n,
            sub_lo=op.origin, sub_hi=n, n=n, tap_lo=op.tap_lo,
            tap_hi=op.tap_hi, name="support_disjoint",
            kind="support_disjoint")
        win_set["no_applied_subtraction"] = True
    if support is None:
        support = win_set
    cell_contained = bool(not has_sub or (win_set["reps"] and support["reps"]
                                         and win_set["first"]
                                         <= support["first"]
                                         and win_set["last"]
                                         >= support["last"]))
    set_used = support if cell_contained else win_set
    sets = {"full": AM.full_preamble_set(op),
            "indep": set_used,
            "legacy_nominal_index": AM.legacy_nominal_index_set(op)}
    table = AM.residual_table(H_clean, H_raw, H_sic, op, sets=sets)

    # per-ROI independent sets: appendix only (different sample sets per ROI)
    roi_sets = {}
    roi_appendix = {}
    for rname, (rlo, rhi) in (("target", (AM.TARGET_LO, AM.TARGET_HI)),
                              ("background", (AM.BG_LO, AM.BG_HI)),
                              ("direct", (AM.DIRECT_LO, AM.DIRECT_HI))):
        rs = AM.derive_rep_set(
            op, visible_end=visible_end, sub_lo=sub_lo, sub_hi=sub_hi, n=n,
            tap_lo=rlo, tap_hi=rhi, name="support_disjoint_%s" % rname,
            kind="support_disjoint_roi")
        roi_sets[rname] = rs
        sub = AM.residual_table(H_clean, H_raw, H_sic, op,
                                sets={"indep": rs})["indep"]
        keep = {}
        for N, entry in sub.items():
            keep[N] = {
                "N": entry["N"], "n_actual": entry["n_actual"],
                "roi_shortened": entry.get("roi_shortened", {}).get(rname),
                "c_cir_db": entry["c_cir_db"].get(rname),
                "c_cir_flag": entry["c_cir_flag"].get(rname),
                "e_raw": entry["e_raw"].get(rname),
                "e_sic": entry["e_sic"].get(rname),
                "nmse_sic": entry["nmse_sic"].get(rname),
            }
        roi_appendix[rname] = {"set": _rs_summary(rs), "table": keep}

    # target protection per N on the same sets as the accumulation table
    target = {}
    for set_name in ("full", "indep", "legacy_nominal_index"):
        rs = sets[set_name]
        for N in rs["n_list"]:
            idx = np.asarray(rs["reps"][:int(N)], dtype=np.int64)
            if idx.size == 0:
                continue
            hc = AM.accumulate(H_clean, idx)
            target["%s/%d" % (set_name, N)] = {
                "raw": AM.target_metrics(AM.accumulate(H_raw, idx), hc, op),
                "sic": AM.target_metrics(AM.accumulate(H_sic, idx), hc, op),
            }

    E_sic = H_sic - H_clean
    E_raw = H_raw - H_clean
    # main coherence: the support-disjoint (legal) repetitions; the full
    # preamble is kept as a separately named legacy view.
    legal = np.asarray(set_used["reps"], dtype=np.int64)
    coh = {}
    for tag, E in (("sic", E_sic), ("raw", E_raw)):
        coh[tag] = {roi: AM.coherence_stats(E[legal], op, roi=roi)
                    for roi in ("target", "background", "all")}
    coh_full = {}
    for tag, E in (("sic", E_sic), ("raw", E_raw)):
        coh_full[tag] = {roi: AM.coherence_stats(E, op, roi=roi)
                         for roi in ("target", "background", "all")}
    per_symbol = {
        "residual_sic_power": [float(v) for v in np.sum(np.abs(E_sic) ** 2,
                                                         axis=1)],
        "residual_raw_power": [float(v) for v in np.sum(np.abs(E_raw) ** 2,
                                                        axis=1)],
        "clean_power": [float(v) for v in np.sum(np.abs(H_clean) ** 2, axis=1)],
    }

    iq = {
        "span": AM.iq_suppression_db(raw, clean, sic_iq, origin, span_hi),
        "window": AM.iq_suppression_db(raw, clean, sic_iq, 0, n),
        "bypass_bit_identical": bypass_identical,
        "output_changed": not bypass_identical,
        "raw_sha256": AM.sha256_f32(raw),
        "clean_sha256": AM.sha256_f32(clean),
        "sic_sha256": AM.sha256_f32(sic_iq),
        "raw_minus_clean_energy": AM.energy(
            raw.astype(np.complex128) - clean.astype(np.complex128)),
        "sic_minus_clean_energy": AM.energy(
            sic_iq.astype(np.complex128) - clean.astype(np.complex128)),
    }

    rec = {
        "window_id": int(wid), "cell_id": cell["id"], "kind": kind,
        "seed": int(seed), "n_samples": n,
        "comm_present": bool(win["truth"]["comm_present"]),
        "isr_db_target": cell.get("isr_db"),
        "isr_db_realized": win.get("isr_db"),
        "cfo_hz_injected": float(cell.get("cfo_hz", 0.0)),
        "start_offset_work": float(cell.get("start_offset_work", 0.0)),
        "comm_channel": cell.get("comm_paths", "single"),
        "overlap": win["overlap"],
        "sic": {
            "committed": committed,
            "detected": bool(dget(out_meta, "sic_detected", False)),
            "partial": bool(dget(out_meta, "sic_partial", False)),
            "bypass_reason": dget(out_meta, "sic_bypass_reason", None),
            "state": dget(out_meta, "sic_state", None),
            "uses_truth": uses_truth,
            "cfo_hz_est": cfo_est,
            "n_paths": int(dnum(out_meta, "sic_n_paths", 0)),
            "support_lo": int(dnum(out_meta, "sic_support_lo", -1)),
            "support_hi": int(dnum(out_meta, "sic_support_hi", -1)),
            "sub_lo": int(dnum(out_meta, "sic_sub_lo", -1)),
            "sub_hi": int(dnum(out_meta, "sic_sub_hi", -1)),
            "fit_residual": dnum(out_meta, "sic_fit_residual", None),
            "cond2": dnum(out_meta, "sic_cond2", None),
            "model_id": dget(out_meta, "sic_model_id", None),
            "model_fingerprint": dget(out_meta, "sic_model_fingerprint", None),
            "support_source": dget(out_meta, "sic_support_source", None),
            "apply_refit_count": dnum(out_meta, "sic_apply_refit_count", -1),
            "residual_corr": dnum(out_meta, "sic_residual_corr", None),
            "core_total_us": dnum(out_meta, "core_total_us", 0.0),
        },
        "iq": iq, "accum": table, "target": target,
        "coherence": coh, "coherence_full_preamble": coh_full,
        "roi_sets_appendix": roi_appendix,
        "support": {
            "sensing_origin": int(op.origin),
            "model_origin": model_origin,
            "visible_end": int(visible_end),
            "training_lo": train_lo, "training_hi": train_hi,
            "extra_guard": (int(visible_end) - train_hi
                            if train_hi > 0 else None),
            "sub_lo": sub_lo, "sub_hi": sub_hi,
            "support_lo": int(dnum(out_meta, "sic_support_lo", -1)),
            "support_hi": int(dnum(out_meta, "sic_support_hi", -1)),
            "nominal_support_hi": int(dnum(out_meta,
                                          "sic_predicted_support_hi", -1)),
            "tap_grid": [int(op.tap_lo), int(op.tap_hi)],
            "code_nz": {"nz_min": int(nz["nz_min"]),
                        "nz_max": int(nz["nz_max"]),
                        "nz_count": int(nz["nz_count"]),
                        "conservative_len": int(nz["conservative_len"])},
            "window_set": _rs_summary(win_set),
            "cell_set": _rs_summary(support),
            "cell_set_contained": cell_contained,
            "set_used": "cell" if cell_contained else "window",
            "no_applied_subtraction": (not has_sub),
            "no_applied_subtraction_source": (
                None if has_sub else
                "adapter published no applied region (bypass/failure); the "
                "analysis uses the public nominal cell set"),
            "excluded": win_set["excluded"],
            "coverage": AM.coverage_report(
                op, set_used, n=n,
                truth_span=(win["truth"]["comm"]["start_sample_exact"],
                            win["truth"]["comm"]["start_sample_exact"]
                            + win["truth"]["comm"]["length_samples"])
                if win["truth"]["comm_present"] else None),
        },
        "per_symbol": per_symbol,
    }
    if committed and cfo_est != 0.0:
        rec["accum_cfo_compensated_diagnostic"] = {
            "note": "common derotation by the ONLINE SIC cfo estimate; "
                    "diagnostic only, never a default metric",
            "table": AM.residual_table(AM.rotate_by_cfo(H_clean, cfo_est),
                                       AM.rotate_by_cfo(H_raw, cfo_est),
                                       AM.rotate_by_cfo(H_sic, cfo_est), op),
        }

    if cir_runners:
        m = rx_meta(wid, n, op.sync_reps, op.origin)
        rec["receiver"] = {"configs": {}, "windows": {}}
        for key, runner in cir_runners.items():
            branch = {}
            for bname, iq_b in (("clean", clean), ("raw", raw),
                                ("sic", sic_iq)):
                runner.post(m, np.ascontiguousarray(iq_b, dtype=np.complex64))
                mm = runner.wait(deadline_s=120.0)
                if mm is None:
                    branch[bname] = {"status": "timeout",
                                     "last_status": runner.last_status()}
                    continue
                branch[bname] = {
                    "status": dget(mm, "status", None),
                    "status_code": dnum(mm, "status_code", -1),
                    "tap_count": int(dnum(mm, "tap_count", 0)),
                    "peak_tap": int(dnum(mm, "peak_tap", 0)),
                    "cir_origin_sample": int(dnum(mm, "cir_origin_sample", -1)),
                    "preamble_start_sample": int(
                        dnum(mm, "preamble_start_sample", -1)),
                    "zero_delay_tap": int(dnum(mm, "zero_delay_tap", -1)),
                    "raw_l2_norm": dnum(mm, "raw_l2_norm", 0.0),
                    "valid_repetitions": int(dnum(mm, "valid_repetitions", 0)),
                    "taps": [str(complex(v)) for v in
                             np.asarray(mm.get("__c32", []),
                                        dtype=np.complex128)],
                }
            rec["receiver"]["configs"][key] = {
                "skip": runner.skip, "count": runner.count,
                "branches": branch,
            }
    return rec, H_clean, H_raw, H_sic


# ---------------------------------------------------------------------------
# Aggregation
# ---------------------------------------------------------------------------
def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(float(v))


def _mean(vals):
    v = [float(x) for x in vals if _finite(x)]
    return float(np.mean(v)) if v else None


def _pct(vals, p):
    v = [float(x) for x in vals if _finite(x)]
    return float(np.percentile(v, p)) if v else None


def _set_names(recs):
    names = []
    for r in recs:
        for k in (r.get("accum") or {}):
            if k in ("sets", "rois", "tap_domain"):
                continue
            if k not in names:
                names.append(k)
    return names


def _n_lists(recs):
    out = {}
    for r in recs:
        for name, meta in ((r.get("accum") or {}).get("sets") or {}).items():
            if name not in out:
                out[name] = list(meta.get("n_list") or [])
    return out


def _worst_windows(recs):
    """Per-cell lowest C_CIR and largest target-error windows (evidence only)."""
    if not recs:
        return {}
    set_names = _set_names(recs)
    main = "indep" if "indep" in set_names else (set_names[0] if set_names
                                                 else None)
    if main is None:
        return {}
    n_list = _n_lists(recs).get(main, [])
    if not n_list:
        return {}
    n_key = str(n_list[-1])
    scored = []
    for r in recs:
        e = (r.get("accum") or {}).get(main, {}).get(n_key)
        if not e:
            continue
        c = e["c_cir_db"]["full"]
        amp = ((r.get("target") or {}).get("%s/%s" % (main, n_key))
               or {}).get("sic", {}).get("amp_err_db")
        scored.append({"window_id": r["window_id"], "seed": r["seed"],
                       "c_cir_db": c, "amp_err_db": amp})
    finite_c = [s for s in scored if _finite(s["c_cir_db"])]
    finite_a = [s for s in scored if _finite(s["amp_err_db"])]
    low = min(finite_c, key=lambda x: x["c_cir_db"]) if finite_c else None
    high = (max(finite_a, key=lambda x: abs(x["amp_err_db"]))
            if finite_a else None)
    undef = []
    if low is None:
        undef.append("lowest_c_cir")
    if high is None:
        undef.append("largest_target_error")
    return {"set": main, "N": int(n_key),
            "selection_undefined": undef,
            "selection_undefined_note": (
                "an undefined entry means every window in this cell had no "
                "finite value for that metric (e.g. a fully bypassed run); it "
                "is an explicit declaration, not a missing file"),
            "selection_rule": "min C_CIR and max |amp err| over the cell at "
                              "the largest legal N; these two windows are "
                              "evidence only and are NOT used for any "
                              "unbiased distribution statistic",
            "lowest_c_cir": low, "largest_target_error": high,
            "unbiased_windows": len(recs)}


def summarize_cell(recs):
    """Per-cell aggregation over ALL windows (failures stay in the denominator)."""
    n_win = len(recs)
    if n_win == 0:
        return {}
    out = {
        "windows": n_win,
        "committed": sum(1 for r in recs if r["sic"]["committed"]),
        "bypassed": sum(1 for r in recs if not r["sic"]["committed"]),
        "uses_truth": sum(1 for r in recs if r["sic"]["uses_truth"]),
        "output_changed": sum(1 for r in recs if r["iq"]["output_changed"]),
        "bypass_bit_identical": sum(
            1 for r in recs if r["iq"]["bypass_bit_identical"]),
        "isr_db_realized_mean": _mean([r["isr_db_realized"] for r in recs]),
        "iq_suppression_span_db_mean": _mean(
            [r["iq"]["span"]["suppression_db"] for r in recs]),
        "iq_suppression_span_db_p05": _pct(
            [r["iq"]["span"]["suppression_db"] for r in recs], 5),
        "iq_suppression_window_db_mean": _mean(
            [r["iq"]["window"]["suppression_db"] for r in recs]),
        "accum": {}, "target": {}, "coherence": {}, "receiver": {},
        "sets": {}, "support": {},
    }
    set_names = _set_names(recs)
    n_lists = _n_lists(recs)
    for set_name in set_names:
        out["sets"][set_name] = (recs[0]["accum"].get("sets") or {}).get(
            set_name, {})
    for set_name in set_names:
        for N in n_lists.get(set_name, []):
            key = str(N)
            entry = {}
            for roi in ("full", "target", "background", "direct"):
                cc = [r["accum"][set_name][key]["c_cir_db"][roi] for r in recs]
                cc_f = [float(v) for v in cc if _finite(v)]
                entry[roi] = {
                    "n_defined": len(cc_f),
                    "n_na": sum(1 for r in recs
                                if r["accum"][set_name][key]
                                ["c_cir_flag"][roi] == "na"),
                    "n_pos_inf": sum(1 for r in recs
                                     if r["accum"][set_name][key]
                                     ["c_cir_flag"][roi] == "pos_inf"),
                    "n_neg_inf": sum(1 for r in recs
                                     if r["accum"][set_name][key]
                                     ["c_cir_flag"][roi] == "neg_inf"),
                    "mean": _mean(cc_f), "median": _pct(cc_f, 50),
                    "p05": _pct(cc_f, 5), "p95": _pct(cc_f, 95),
                    "ci": AM.bootstrap_ci(cc_f) if cc_f else None,
                    "nmse_raw_mean": _mean(
                        [r["accum"][set_name][key]["nmse_raw"][roi]
                         for r in recs]),
                    "nmse_sic_mean": _mean(
                        [r["accum"][set_name][key]["nmse_sic"][roi]
                         for r in recs]),
                }
            g_sic = [r["accum"][set_name][key]["g_sic_db"] for r in recs]
            g_raw = [r["accum"][set_name][key]["g_raw_db"] for r in recs]
            entry["g_sic_db"] = {
                "n": len([v for v in g_sic if _finite(v)]),
                "mean": _mean(g_sic),
                "ci": AM.bootstrap_ci([v for v in g_sic if _finite(v)])
                if [v for v in g_sic if _finite(v)] else None}
            entry["g_raw_db"] = {
                "n": len([v for v in g_raw if _finite(v)]),
                "mean": _mean(g_raw),
                "ci": AM.bootstrap_ci([v for v in g_raw if _finite(v)])
                if [v for v in g_raw if _finite(v)] else None}
            entry["g_is_sic_gain"] = False
            out["accum"]["%s/%s" % (set_name, N)] = entry
    for set_name in set_names:
        for N in n_lists.get(set_name, []):
            key = "%s/%d" % (set_name, N)
            amp = [abs(r["target"][key]["sic"]["amp_err_db"]) for r in recs
                   if _finite(r["target"][key]["sic"]["amp_err_db"])]
            amp_raw = [abs(r["target"][key]["raw"]["amp_err_db"]) for r in recs
                       if _finite(r["target"][key]["raw"]["amp_err_db"])]
            phase = [abs(r["target"][key]["sic"]["phase_err_deg"]) for r in recs
                     if _finite(r["target"][key]["sic"]["phase_err_deg"])]
            shift = [abs(r["target"][key]["sic"]["peak_shift"]) for r in recs]
            bg = [r["target"][key]["sic"]["bg_false_peak_ratio"] for r in recs]
            proxy = [r["target"][key]["sic"]["target_to_background_proxy"]
                     for r in recs]
            out["target"][key] = {
                "n_defined": len(amp),
                "amp_err_abs_p95_db": _pct(amp, 95),
                "amp_err_abs_max_db": max(amp) if amp else None,
                "amp_err_abs_mean_db": _mean(amp),
                "raw_amp_err_abs_p95_db": _pct(amp_raw, 95),
                "phase_err_abs_p95_deg": _pct(phase, 95),
                "peak_shift_abs_max": max(shift) if shift else None,
                "peak_shift_p95": _pct(shift, 95),
                "bg_false_peak_ratio_mean": _mean(bg),
                "target_to_background_proxy_mean": _mean(proxy),
                "pass_amp_p95_le_1db": bool(len(amp) > 0
                                            and _pct(amp, 95) <= 1.0),
                "pass_peak_shift_le_1": bool(shift and max(shift) <= 1),
                "threshold_amp_p95_db": 1.0, "threshold_peak_shift": 1,
            }
    for view in ("coherence", "coherence_full_preamble"):
        for tag in ("sic", "raw"):
            for roi in ("target", "background", "all"):
                cf = [r[view][tag][roi].get("coherent_fraction") for r in recs
                      if _finite(r[view][tag][roi].get("coherent_fraction"))]
                slopes = [r[view][tag][roi].get("phase_slope_hz") for r in recs
                          if _finite(r[view][tag][roi].get("phase_slope_hz"))]
                cons = [r[view][tag][roi].get("energy_conservation_rel")
                        for r in recs
                        if _finite(r[view][tag][roi].get(
                            "energy_conservation_rel"))]
                out["coherence"]["%s/%s/%s" % (view, tag, roi)] = {
                    "view": view,
                    "n": len(cf), "eta_mean": _mean(cf),
                    "coherent_fraction_mean": _mean(cf),
                    "eta_ci": AM.bootstrap_ci(cf) if cf else None,
                    "eta_random_reference": 1.0 / max(1, len(
                        (recs[0]["accum"].get("sets") or {}).get(
                            "indep", {}).get("n_list") or [64])),
                    "e_coherent_mean": _mean(
                        [r[view][tag][roi].get("e_coherent") for r in recs]),
                    "e_centered_mean": _mean(
                        [r[view][tag][roi].get("e_centered") for r in recs]),
                    "e_total_mean": _mean(
                        [r[view][tag][roi].get("e_total") for r in recs]),
                    "mu_energy_mean": _mean(
                        [r[view][tag][roi].get("mu_energy") for r in recs]),
                    "energy_conservation_rel_max": (max(cons) if cons else None),
                    "lag1_mean": _mean(
                        [r[view][tag][roi]["lag"].get("1") for r in recs]),
                    "centered_lag1_mean": _mean(
                        [r[view][tag][roi]["centered_lag"].get("1")
                         for r in recs]),
                    "phase_slope_hz_mean": _mean(slopes),
                    "phase_slope_hz_p95": _pct([abs(s) for s in slopes], 95),
                    "mean_fraction_retired": True,
                }
    # ---- A-R01: support bookkeeping / coverage ---------------------------
    cell_sets = [(r.get("support") or {}).get("cell_set") for r in recs]
    cell_sets = [c for c in cell_sets if c]
    mism = [r["window_id"] for r in recs
            if not (r.get("support") or {}).get("cell_set_contained", True)]
    cov_sub = [((r.get("support") or {}).get("coverage") or {}).get(
        "cir_overlap_sub_ratio") for r in recs]
    cov_truth = [((r.get("support") or {}).get("coverage") or {}).get(
        "cir_overlap_truth_ratio") for r in recs]
    firsts = [((r.get("support") or {}).get("window_set") or {}).get("first")
              for r in recs]
    lasts = [((r.get("support") or {}).get("window_set") or {}).get("last")
             for r in recs]
    out["support"] = {
        "cell_set": cell_sets[0] if cell_sets else None,
        "set_used_counts": {
            "cell": sum(1 for r in recs
                        if (r.get("support") or {}).get("set_used") == "cell"),
            "window": sum(1 for r in recs
                          if (r.get("support") or {}).get("set_used")
                          == "window")},
        "cell_set_mismatch_windows": mism,
        "window_set_first_min": min([f for f in firsts if f is not None],
                                    default=None),
        "window_set_last_max": max([l for l in lasts if l is not None],
                                   default=None),
        "cir_overlap_sub_ratio_mean": _mean(cov_sub),
        "cir_overlap_truth_ratio_mean": _mean(cov_truth),
        "coverage_note": "truth support is an OFFLINE diagnostic only; it "
                         "never feeds a gate or geometry choice",
    }
    # explicit worst-window evidence (separate from unbiased distributions)
    worst = _worst_windows(recs)
    out["worst_windows"] = worst
    recs_rx = [r for r in recs if "receiver" in r]
    if recs_rx:
        out["receiver"] = {"windows": len(recs_rx), "configs": {}}
        for key in recs_rx[0]["receiver"]["configs"]:
            cfg0 = recs_rx[0]["receiver"]["configs"][key]
            per = {"skip": cfg0["skip"], "count": cfg0["count"], "branches": {}}
            for bname in ("clean", "raw", "sic"):
                ok = [r["receiver"]["configs"][key]["branches"][bname]
                      for r in recs_rx
                      if r["receiver"]["configs"][key]["branches"][bname]
                      .get("status") == "ok"]
                per["branches"][bname] = {
                    "n_ok": len(ok), "n_total": len(recs_rx),
                    "peak_tap_median": _pct([b["peak_tap"] for b in ok], 50),
                    "raw_l2_norm_mean": _mean([b["raw_l2_norm"] for b in ok]),
                    "valid_repetitions_mean": _mean(
                        [b["valid_repetitions"] for b in ok]),
                    "preamble_start_mean": _mean(
                        [b["preamble_start_sample"] for b in ok]),
                    "target_to_background_proxy_mean": _mean(
                        [_rx_target_proxy(r["receiver"]["configs"][key], bname)
                         for r in recs_rx]),
                }
            out["receiver"]["configs"][key] = per
    return out


def _rx_target_proxy(cfg, branch):
    b = cfg["branches"][branch]
    taps = b.get("taps")
    if not taps or b.get("status") != "ok":
        return None
    t = np.asarray([complex(v) for v in taps], dtype=np.complex128)
    tgt_lo, tgt_hi = AM.RX_TARGET_ROI
    bg_lo, bg_hi = AM.RX_BACKGROUND_TAPS
    if tgt_hi > t.size or bg_hi > t.size or tgt_lo < 0:
        return None
    a = float(np.max(np.abs(t[tgt_lo:tgt_hi])))
    bg = float(np.mean(np.abs(t[bg_lo:bg_hi]) ** 2))
    return (a ** 2) / bg if bg > 0 else None



def _make_window(generator, sic_runner, *, cell, cp, seed, snr_db, sensing_roi,
                 observed_end_public, wid):
    """Generate one paired window and run the real SIC PDU adapter on raw."""
    win = generator.build(
        seed=int(seed), comm_paths=cp,
        start_offset_work=float(cell.get("start_offset_work", 0.0)),
        cfo_hz=float(cell.get("cfo_hz", 0.0)),
        isr_db=cell.get("isr_db"), snr_db=snr_db)
    clean = np.ascontiguousarray(win["s"] + win["w"], dtype=np.complex64)
    raw = np.ascontiguousarray(win["y"], dtype=np.complex64)
    meta = rx_meta(wid, int(raw.size), S.DEFAULT_SYNC_REPS, generator.pre)
    meta = put_int(meta, "sensing_roi_lo", sensing_roi[0])
    meta = put_int(meta, "sensing_roi_hi", sensing_roi[1])
    meta = put_int(meta, "observed_end_sample", observed_end_public)
    sic_runner.post(meta, raw)
    res = sic_runner.wait()
    if res is None:
        raise SystemExit("SIC adapter did not terminate window_id=%d" % wid)
    out_meta, ihat_meta = res
    ihat = np.ascontiguousarray(ihat_meta.get("__c32", np.zeros_like(raw)),
                                dtype=np.complex64)
    sic_iq = np.ascontiguousarray(out_meta.get("__c32", raw),
                                  dtype=np.complex64)
    return win, clean, raw, sic_iq, ihat, out_meta


def _save_window(path, *, clean, raw, sic_iq, ihat, H_clean, H_raw, H_sic,
                 win, rec, reason=None):
    """Archive one window (IQ + CIR + truth sidecar + record) and return path."""
    meta = dict(rec)
    meta["archive_reason"] = reason
    np.savez_compressed(
        path, clean=clean, raw=raw, sic=sic_iq, ihat=ihat,
        i_truth=np.asarray(win["i"], dtype=np.complex64),
        H_clean=H_clean, H_raw=H_raw, H_sic=H_sic,
        truth_json=np.frombuffer(json.dumps(win["truth"]).encode("utf-8"),
                                 dtype=np.uint8),
        meta_json=np.frombuffer(json.dumps(meta, default=str).encode("utf-8"),
                                dtype=np.uint8))
    return path


# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------
def run(args) -> int:
    frozen_op = load_json(testdata_dir("operator.json"))
    cells_file = load_json(testdata_dir("cells.json"))
    root = repo_root()
    t_start = time.time()
    outdir = os.path.abspath(args.output)
    for sub in ("templates", "archive", "archive_worst", "figures", "logs",
                "operator"):
        os.makedirs(os.path.join(outdir, sub), exist_ok=True)

    if args.bypass:
        cells = [{"id": "no_interference", "isr_db": None, "cfo_hz": 0.0,
                  "comm_paths": "single", "start_offset_work": 0.0}]
        comm_present = False
    else:
        cells = [c for c in cells_file["cells"]
                 if not args.cell or c["id"] in args.cell]
        comm_present = True
    if not cells:
        raise SystemExit("no cells selected")
    windows_per_cell = int(args.windows_per_cell)
    seed_base = int(frozen_op["seeds"][args.seed_set])

    SIC_CFG = frozen_op["sic"]
    start = time.time()
    uwb = S.load_uwb(args.build_dir)
    loaded = library_provenance(args.build_dir)
    if not loaded["lib_exists"]:
        raise SystemExit("built library missing: %s" % loaded["expected_lib"])
    if not loaded["in_worktree"]:
        raise SystemExit("build dir is outside this worktree: %s"
                         % loaded["build_dir"])

    snr_db = float(frozen_op["evaluation"]["snr_db"])
    generator = S.SicSignalGenerator(
        uwb, sensing_code=S.DEFAULT_SENSING_CODE, comm_code=S.DEFAULT_COMM_CODE,
        sync_reps=S.DEFAULT_SYNC_REPS, sfd_mode=SFD_MODE,
        pulse_shape=S.DEFAULT_PULSE_SHAPE, comm_present=comm_present,
        build_comm=True, snr_db=snr_db,
        isr_db=10.0 if comm_present else None, verbose=True)
    op, op_ev = build_operator(uwb, generator, frozen_op)
    roi_ev = verify_target_roi(generator, op, frozen_op, comm_present)
    print("weak-target ROI verified: peak=%s direct=%s ratio=%s"
          % (roi_ev["target_peak_taps"], roi_ev["direct_peak_taps"],
             ["%.4f" % r for r in roi_ev["target_over_direct_amp"]]),
          flush=True)

    tpl_path = os.path.join(outdir, "templates", "sync_template_code9_998p4.cf32")
    generator.template.tofile(tpl_path)
    np.savez(os.path.join(outdir, "operator", "operator_arrays.npz"),
             sampled_code=np.asarray(op.code, dtype=np.complex64),
             chips=np.asarray(op_ev["chips"], dtype=np.int8),
             pulse_taps=np.asarray(S.pulse_taps(uwb, generator.pulse_shape,
                                                2.5, 200.0), dtype=np.float64),
             sensing_template=np.asarray(generator.template,
                                         dtype=np.complex64))

    visibility_guard = int(math.ceil(max(
        [float(c["start_offset_work"]) for c in cells
         if 0.0 < float(c.get("start_offset_work", 0.0)) < SPS] or [0.0])))
    if not comm_present:
        visibility_guard = 0
    generator.overlap_tail_guard = visibility_guard
    nominal_origin = int(generator.pre)
    fit_reps = int(SIC_CFG["fit_reps"])
    validation_reps = int(SIC_CFG["validation_reps"])
    observed_end_public = int(min(generator.n, nominal_origin
                                  + visibility_guard + fit_reps * SPS))
    search_roi = (max(0, nominal_origin - visibility_guard),
                  min(generator.n, nominal_origin + S.DEFAULT_SYNC_REPS * SPS))
    core_roi = (int(search_roi[0]),
                int(min(generator.n, search_roi[1] + S.DEFAULT_SYNC_REPS * SPS
                        + int(SIC_CFG["post_tail"]))))
    sensing_roi = (int(nominal_origin),
                   int(min(generator.n, nominal_origin + S.DEFAULT_SYNC_REPS * SPS
                           + visibility_guard)))
    # ---- A-R01: cell-common support-disjoint repetition set ----------------
    # Derived from the PUBLIC plan only (visibility limit, nominal support and
    # the actual sensing processing ROI), so it is fixed before any window is
    # evaluated.  Each window then re-derives its own actual set from the
    # adapter's published sub_lo/sub_hi and must CONTAIN this set; a mismatch is
    # recorded and handled explicitly (never silently ignored).
    nominal_support_hi = int(min(
        generator.n, nominal_origin + S.DEFAULT_SYNC_REPS * SPS
        + int(SIC_CFG["post_tail"])))
    sub_lo_nom = int(max(nominal_origin, sensing_roi[0]))
    sub_hi_nom = int(min(nominal_support_hi, sensing_roi[1]))
    cell_support = AM.derive_rep_set(
        op, visible_end=observed_end_public, sub_lo=sub_lo_nom,
        sub_hi=sub_hi_nom, n=generator.n, tap_lo=op.tap_lo, tap_hi=op.tap_hi,
        name="support_disjoint", kind="support_disjoint")
    print("support-disjoint set: reps=[%d,%d] available=%d n_list=%s "
          "(visible_end=%d sub=[%d,%d))"
          % (cell_support["first"], cell_support["last"],
             cell_support["available"], cell_support["n_list"],
             observed_end_public, sub_lo_nom, sub_hi_nom), flush=True)

    sic_params = {
        "sync": generator.comm_sync, "preamble": generator.comm_wave,
        "comm_code": S.DEFAULT_COMM_CODE, "sync_reps": S.DEFAULT_SYNC_REPS,
        "samples_per_symbol": SPS, "sample_rate": S.WORK_HZ,
        "k_paths": int(SIC_CFG["k_paths"]), "fit_reps": fit_reps,
        "validation_reps": validation_reps,
        "roi_lo": core_roi[0], "roi_hi": core_roi[1],
        "coarse_stride": int(SIC_CFG["coarse_stride"]), "mode": "estimate",
        "detect_threshold": float(SIC_CFG["detect_threshold"]),
        "validate_threshold": float(SIC_CFG["validate_threshold"]),
    }
    sic_extras = {
        "in_flight_max": 8, "iq_budget_bytes": 64 << 20, "queue_capacity": 8,
        "publish_reconstruction": True,
        "post_tail": int(SIC_CFG["post_tail"]),
        "enable_fractional": True, "enable_cfo": True, "path_min_spacing": 0,
        "max_window_samples": 0, "support_mode": "configured_nominal",
        "online_validation_reps": 0,
        "fit_residual_max": float(SIC_CFG["fit_residual_max"]),
    }
    sic_runner = SicRunner(uwb, sic_params, sic_extras)

    cir_runners = None
    if args.receiver:
        cir_runners = {}
        for spec in frozen_op["receiver_configs"]:
            cir_runners[spec["id"]] = CirRunner(
                uwb, tpl_path, sync_reps=S.DEFAULT_SYNC_REPS, skip=int(spec["skip"]),
                count=int(spec["count"]), name=spec["id"])

    windows_jsonl = open(os.path.join(outdir, "windows.jsonl"), "w",
                         encoding="utf-8")
    table_rows = []
    all_recs = {}
    archived = []
    worst_archived = []
    try:
        for ci, cell in enumerate(cells):
            recs = []
            cp = (cells_file["comm_paths"][cell["comm_paths"]]
                  if comm_present else [[0, [1.0, 0.0]]])
            for wi in range(windows_per_cell):
                seed = window_seed(seed_base, ci, wi)
                win, clean, raw, sic_iq, ihat, out_meta = _make_window(
                    generator, sic_runner, cell=cell, cp=cp, seed=seed,
                    snr_db=snr_db, sensing_roi=sensing_roi,
                    observed_end_public=observed_end_public, wid=wi)
                use_rx = (cir_runners is not None
                          and wi < int(args.receiver_windows))
                rec, H_clean, H_raw, H_sic = evaluate_window(
                    op, frozen_op, cell, wi, seed, args.seed_set, win, clean,
                    raw, sic_iq, out_meta,
                    cir_runners=cir_runners if use_rx else None,
                    support=cell_support)
                recs.append(rec)
                windows_jsonl.write(json.dumps(rec, allow_nan=True) + "\n")
                table_rows.extend(_table_rows(rec))
                if wi < int(args.archive_windows):
                    path = _save_window(
                        os.path.join(outdir, "archive", "%s_%05d.npz"
                                     % (cell["id"], wi)),
                        clean=clean, raw=raw, sic_iq=sic_iq, ihat=ihat,
                        H_clean=H_clean, H_raw=H_raw, H_sic=H_sic, win=win,
                        rec=rec, reason="first_n_by_index")
                    archived.append(os.path.relpath(path, outdir))
                if (wi + 1) % 20 == 0 or wi + 1 == windows_per_cell:
                    print("[%s] %d/%d windows (%.1fs)"
                          % (cell["id"], wi + 1, windows_per_cell,
                             time.time() - start), flush=True)
            # A-R04: worst-window evidence, regenerated and hash-verified.
            # One window can be selected by both rules; it is archived ONCE with
            # the combined reason and both declarations point at it.
            worst = _worst_windows(recs)
            picks = {}
            for label in ("lowest_c_cir", "largest_target_error"):
                wrec = worst.get(label)
                if not wrec:
                    continue
                picks.setdefault(int(wrec["window_id"]), []).append(label)
            for wid in sorted(picks):
                reasons = sorted(picks[wid])
                rel = os.path.join("archive_worst", "%s_%05d.npz"
                                   % (cell["id"], wid))
                if wid < int(args.archive_windows):
                    for label in reasons:
                        worst_archived.append({
                            "cell_id": cell["id"], "window_id": wid,
                            "reason": label, "reasons": reasons,
                            "path": os.path.join(
                                "archive", "%s_%05d.npz" % (cell["id"], wid)),
                            "already_archived": True})
                    continue
                rec_old = recs[wid]
                win2, clean2, raw2, sic2, ihat2, out2 = _make_window(
                    generator, sic_runner, cell=cell, cp=cp,
                    seed=int(rec_old["seed"]), snr_db=snr_db,
                    sensing_roi=sensing_roi,
                    observed_end_public=observed_end_public, wid=wid)
                same = (AM.sha256_f32(raw2) == rec_old["iq"]["raw_sha256"]
                        and AM.sha256_f32(clean2)
                        == rec_old["iq"]["clean_sha256"]
                        and AM.sha256_f32(sic2) == rec_old["iq"]["sic_sha256"])
                if not same:
                    for label in reasons:
                        worst_archived.append({
                            "cell_id": cell["id"], "window_id": wid,
                            "reason": label, "reasons": reasons, "path": None,
                            "already_archived": False,
                            "error": "regeneration hash mismatch; NOT "
                                     "archived (no empty marker)"})
                    print("WARNING: worst-window regeneration hash mismatch "
                          "%s/%d - not archived" % (cell["id"], wid),
                          flush=True)
                    continue
                path = _save_window(
                    os.path.join(outdir, rel), clean=clean2, raw=raw2,
                    sic_iq=sic2, ihat=ihat2,
                    H_clean=AM.fixed_operator_cir(clean2, op),
                    H_raw=AM.fixed_operator_cir(raw2, op),
                    H_sic=AM.fixed_operator_cir(sic2, op), win=win2,
                    rec=rec_old, reason="+".join(reasons))
                archived.append(rel)
                for label in reasons:
                    worst_archived.append({
                        "cell_id": cell["id"], "window_id": wid,
                        "reason": label, "reasons": reasons, "path": rel,
                        "already_archived": False, "verified_hashes": True})
            all_recs[cell["id"]] = recs
    finally:
        windows_jsonl.close()
        sic_runner.stop()
        if cir_runners:
            for r in cir_runners.values():
                r.stop()

    with open(os.path.join(outdir, "accum_table.csv"), "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["cell_id", "window_id", "set", "N", "roi", "c_cir_db",
                    "c_cir_flag", "e_raw", "e_sic", "nmse_raw", "nmse_sic",
                    "g_sic_db", "g_raw_db", "residual_raw_db",
                    "residual_sic_db", "n_actual", "roi_shortened"])
        w.writerows(table_rows)

    per_cell = {cid: summarize_cell(recs) for cid, recs in all_recs.items()}
    total_windows = sum(len(v) for v in all_recs.values())
    summary = {
        "schema_version": SCHEMA_VERSION, "task": "S2-A",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "base_commit": BASE_COMMIT,
        "mode": "bypass" if args.bypass else "interference",
        "seed_set": args.seed_set, "windows_per_cell": windows_per_cell,
        "cells": list(all_recs.keys()), "per_cell": per_cell,
        "totals": {
            "windows": total_windows,
            "committed": sum(1 for v in all_recs.values() for r in v
                             if r["sic"]["committed"]),
            "bypassed": sum(1 for v in all_recs.values() for r in v
                            if not r["sic"]["committed"]),
            "uses_truth": sum(1 for v in all_recs.values() for r in v
                              if r["sic"]["uses_truth"]),
            "output_changed": sum(1 for v in all_recs.values() for r in v
                                  if r["iq"]["output_changed"]),
            "partitions": sum(len(v) for v in all_recs.values()),
        },
        "archive": {"archived_windows": len(archived),
                    "total_windows": total_windows,
                    "rule": "first --archive-windows windows of each cell by "
                            "window index (deterministic) plus the per-cell "
                            "worst C_CIR / worst target-error windows, "
                            "regenerated and hash-verified before archiving; "
                            "the remaining windows cannot be recomputed "
                            "independently from the archive",
                    "fixed_by_index": [p for p in archived
                                       if "archive_worst" not in p],
                    "worst_windows": worst_archived,
                    "worst_windows_rule": "selection is metric-driven "
                                          "evidence only and is NOT used for "
                                          "any unbiased distribution statistic",
                    "paths": archived},
        "stage": args.stage,
        "rectification": {
            "schema_version": SCHEMA_VERSION,
            "items": ["A-R01 support-disjoint independent region",
                      "A-R02 same-dimension coherent/centered decomposition"],
            "support_disjoint_cell_set": {
                "first": cell_support["first"],
                "last": cell_support["last"],
                "available": cell_support["available"],
                "n_list": list(cell_support["n_list"]),
                "read_lo": cell_support["read_lo"],
                "read_hi": cell_support["read_hi"]}}, 
        "operator": {"origin": int(op.origin), "energy": float(op.energy),
                     "sha256": AM.sha256_f32(op.code),
                     "recovery": op_ev["recovery"],
                     "target_roi_offsets": [AM.TARGET_LO, AM.TARGET_HI],
                     "target_tap": AM.TARGET_OFFSET,
                     "background_offsets": [AM.BG_LO, AM.BG_HI],
                     "receiver_target_roi": list(AM.RX_TARGET_ROI),
                     "receiver_target_tap": AM.RX_TARGET_TAP,
                     "receiver_background_taps": list(AM.RX_BACKGROUND_TAPS),
                     "reciprocal_mapping": AM.RX_TAP_OFFSET,
                     "full_N": list(AM.FULL_N),
                     "indep_start": cell_support["first"],
                     "indep_N": list(cell_support["n_list"]),
                     "indep_available": cell_support["available"],
                     "indep_n_nominal": list(cell_support["n_nominal"]),
                     "indep_n_shortened": cell_support["n_shortened"],
                     "support_source": cell_support["support_source"],
                     "legacy_indep_start": AM.LEGACY_INDEP_START,
                     "legacy_indep_N": list(AM.INDEP_N_NOMINAL),
                     "convention": "average", "phase_rotation": "identity",
                     "code_energy_normalization": True,
                     "roi_verification": roi_ev,
                     "support_disjoint_cell_set": {
                         "first": cell_support["first"],
                         "last": cell_support["last"],
                         "available": cell_support["available"],
                         "n_list": list(cell_support["n_list"]),
                         "read_lo": cell_support["read_lo"],
                         "read_hi": cell_support["read_hi"],
                         "bounds": cell_support["bounds"]}},
        "not_run": {
            "matlab": "MATLAB cross-check script provided in "
                      "testdata/sic_accum/matlab/ but NOT executed",
            "native": "native 491.52 MS/s / UHD / FPGA grid not exercised "
                      "(out of S2-A scope)",
        },
        "timing": {"wall_s": time.time() - start,
                   "mode": "diagnostic (no benchmark lock held)"},
    }
    with open(os.path.join(outdir, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True, allow_nan=True)

    with open(os.path.join(outdir, "cells_summary.csv"), "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["cell_id", "set", "N", "roi", "n_defined", "n_na",
                    "c_cir_db_mean", "c_cir_db_median", "nmse_sic_mean",
                    "g_sic_db_mean", "amp_err_p95_db", "raw_amp_err_p95_db",
                    "peak_shift_max", "pass_amp", "pass_shift"])
        for cid, cs in per_cell.items():
            for key, entry in sorted(cs.get("accum", {}).items()):
                set_name, N = key.split("/")
                for roi in ("full", "target", "background", "direct"):
                    e = entry.get(roi, {})
                    tgt = cs.get("target", {}).get(key, {})
                    w.writerow([cid, set_name, N, roi, e.get("n_defined"),
                                e.get("n_na"), e.get("mean"), e.get("median"),
                                e.get("nmse_sic_mean"),
                                entry.get("g_sic_db", {}).get("mean"),
                                tgt.get("amp_err_abs_p95_db"),
                                tgt.get("raw_amp_err_abs_p95_db"),
                                tgt.get("peak_shift_abs_max"),
                                tgt.get("pass_amp_p95_le_1db"),
                                tgt.get("pass_peak_shift_le_1")])

    manifest = {
        "schema_version": SCHEMA_VERSION, "task": "S2-A",
        "run_id": os.path.basename(outdir),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git": git_provenance(root), "build_dir": loaded["build_dir"],
        "loaded_library": loaded,
        "python": sys.version.split()[0], "numpy": np.__version__,
        "platform": platform.platform(),
        "frozen_config": {
            "sic_model_mode": "estimate8_frozen",
            "support_mode": "configured_nominal", "fit_reps": fit_reps,
            "validation_reps": validation_reps, "online_validation_reps": 0,
            "k_paths": int(SIC_CFG["k_paths"]),
            "coarse_stride": int(SIC_CFG["coarse_stride"]),
            "post_tail": int(SIC_CFG["post_tail"]),
            "detect_threshold": float(SIC_CFG["detect_threshold"]),
            "validate_threshold": float(SIC_CFG["validate_threshold"]),
            "fit_residual_max": float(SIC_CFG["fit_residual_max"]),
            "snr_db": snr_db,
        },
        "sic_adapter_params_used": sic_runner.used,
        "geometry": {"pre_guard": int(generator.pre), "n": int(generator.n),
                     "geom": dict(generator.geom), "sps": SPS,
                     "sync_reps": S.DEFAULT_SYNC_REPS},
        "rois": {"search_roi": list(search_roi), "core_roi": list(core_roi),
                 "sensing_roi": list(sensing_roi),
                 "visibility_guard": visibility_guard,
                 "observed_end_public": observed_end_public},
        "operator": summary["operator"], "operator_evidence": op_ev,
        "roi_verification": roi_ev,
        "seeds": {"seed_set": args.seed_set, "base": seed_base,
                  "stride_per_cell": 1000, "cell_order": list(all_recs.keys()),
                  "noise_substream": "np.random.default_rng(seed) per window",
                  "payload_substream": "fixed PSDU (deterministic)",
                  "path_substream": "deterministic profile (no RNG)"},
        "inputs_hashed": _input_hashes(),
        "stage": args.stage,
        "rectification": summary["rectification"],
        "archive": summary["archive"], "cli": " ".join(sys.argv),
        "not_run": summary["not_run"],
    }
    with open(os.path.join(outdir, "run_manifest.json"), "w",
              encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2, sort_keys=True, allow_nan=True)
    shutil.copyfile(testdata_dir("operator.json"),
                    os.path.join(outdir, "operator.json"))
    shutil.copyfile(testdata_dir("cells.json"),
                    os.path.join(outdir, "cells.json"))

    if args.plots:
        try:
            make_plots(outdir, all_recs, per_cell)
        except Exception as exc:  # figures are best-effort, never a gate
            print("plots skipped: %r" % (exc,), flush=True)

    print(json.dumps({"output": outdir,
                      "windows": summary["totals"]["windows"],
                      "committed": summary["totals"]["committed"],
                      "bypassed": summary["totals"]["bypassed"],
                      "wall_s": summary["timing"]["wall_s"],
                      "archived": summary["archive"]["archived_windows"]},
                     indent=2))
    print("total wall %.1fs" % (time.time() - t_start))
    return 0


def _input_hashes():
    files = {
        "operator_contract": testdata_dir("operator.json"),
        "cells_contract": testdata_dir("cells.json"),
        "target_roi_contract": os.path.join(
            repo_root(), "testdata", "sic_preamble", "cases",
            "target_roi.json"),
        "accum_metrics": os.path.join(repo_root(), "tools", "sic",
                                      "accum_metrics.py"),
        "app": os.path.abspath(__file__),
        "sic_generator": os.path.join(_HERE, "sic_preamble_sim.py"),
        "sic_app": os.path.join(_HERE, "uwb_sim_preamble_sic.py"),
        "sic_core_h": os.path.join(repo_root(), "gr-uwb", "include",
                                   "gnuradio", "uwb",
                                   "uwb_preamble_sic_core.h"),
        "sic_adapter_cc": os.path.join(repo_root(), "gr-uwb", "lib",
                                       "uwb_preamble_sic.cc"),
    }
    return {k: {"path": v, "sha256": sha256_file(v)}
            for k, v in files.items() if os.path.exists(v)}


def _table_rows(rec):
    rows = []
    for set_name, table in (rec.get("accum") or {}).items():
        if set_name in ("sets", "rois", "tap_domain"):
            continue
        for N, entry in table.items():
            for roi in ("full", "target", "background", "direct"):
                rows.append([
                    rec["cell_id"], rec["window_id"], set_name, N, roi,
                    AM.csv_cell(entry["c_cir_db"][roi]),
                    entry["c_cir_flag"][roi], entry["e_raw"][roi],
                    entry["e_sic"][roi], AM.csv_cell(entry["nmse_raw"][roi]),
                    AM.csv_cell(entry["nmse_sic"][roi]),
                    AM.csv_cell(entry["g_sic_db"]),
                    AM.csv_cell(entry["g_raw_db"]),
                    AM.csv_cell(entry["residual_raw_db"][roi]),
                    AM.csv_cell(entry["residual_sic_db"][roi]),
                    entry.get("n_actual"),
                    entry.get("roi_shortened", {}).get(roi)])
    return rows


# ---------------------------------------------------------------------------
# Plots (best-effort; cost is reported separately from the run)
# ---------------------------------------------------------------------------
def make_plots(outdir, all_recs, per_cell):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig_dir = os.path.join(outdir, "figures")
    os.makedirs(fig_dir, exist_ok=True)

    for roi in ("target", "full"):
        fig, ax = plt.subplots(figsize=(9, 5))
        for cid, cs in per_cell.items():
            xs, ys = [], []
            for N in AM.FULL_N:
                e = cs["accum"].get("full/%d" % N, {}).get(roi, {})
                if e.get("mean") is not None:
                    xs.append(N)
                    ys.append(e["mean"])
            if xs:
                ax.plot(xs, ys, marker="o", label=cid)
        ax.axhline(0, color="k", lw=0.6)
        ax.set_xscale("log", base=2)
        ax.set_xlabel("N (accumulated SYNC symbols)")
        ax.set_ylabel("C_CIR(N) [dB]")
        ax.set_title("fixed-operator CIR SIC gain, ROI=%s" % roi)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        fig.savefig(os.path.join(fig_dir, "ccir_vs_N_%s.png" % roi), dpi=120)
        plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    for cid, recs in all_recs.items():
        for tag, key in (("raw", "e_raw"), ("sic", "e_sic")):
            ys = [_mean([r["accum"]["full"][str(N)][key]["full"]
                         for r in recs]) for N in AM.FULL_N]
            ax.plot(AM.FULL_N, ys, marker=".", label="%s %s" % (cid, tag))
    first = all_recs[list(all_recs)[0]]
    base = _mean([r["accum"]["full"]["1"]["e_raw"]["full"] for r in first])
    if base:
        ax.plot(AM.FULL_N, [base / N for N in AM.FULL_N], "k--",
                label="1/N reference")
    ax.set_xscale("log", base=2)
    ax.set_yscale("log")
    ax.set_xlabel("N")
    ax.set_ylabel("residual energy (full tap grid)")
    ax.set_title("raw/SIC residual vs N and the 1/N reference")
    ax.grid(True, alpha=0.3, which="both")
    ax.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "residual_vs_N.png"), dpi=120)
    plt.close(fig)

    cid0 = list(all_recs)[0]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for cid, recs in all_recs.items():
        ps = np.mean(np.array([r["per_symbol"]["residual_sic_power"]
                               for r in recs]), axis=0)
        ax[0].plot(range(len(ps)), ps, label=cid)
    ax[0].set_xlabel("SYNC repetition r")
    ax[0].set_ylabel("mean |E_sic|^2 (all taps)")
    ax[0].set_title("per-symbol SIC residual")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend(fontsize=6)
    etas = [r["coherence"]["sic"]["all"]["eta"] for r in all_recs[cid0]]
    ax[1].hist([e for e in etas if e is not None], bins=20)
    ax[1].axvline(1.0 / AM.SYNC_REPS, color="r", ls="--",
                  label="1/N random reference")
    ax[1].set_title("eta (single window, %s)" % cid0)
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "residual_symbols_and_eta.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for cid, cs in per_cell.items():
        xs, amp, ph = [], [], []
        for N in AM.FULL_N:
            t = cs["target"].get("full/%d" % N, {})
            xs.append(N)
            amp.append(t.get("amp_err_abs_p95_db")
                       if t.get("amp_err_abs_p95_db") is not None else np.nan)
            ph.append(t.get("peak_shift_abs_max"))
        ax[0].plot(xs, amp, marker="o", label=cid)
        ax[1].plot(xs, ph, marker="o", label=cid)
    ax[0].axhline(1.0, color="r", ls="--", label="1 dB threshold")
    ax[1].axhline(1.0, color="r", ls="--", label="1 sample threshold")
    ax[0].set_title("weak-target |amp err| P95 [dB]")
    ax[1].set_title("weak-target |peak shift| max [samples]")
    for a in ax:
        a.set_xscale("log", base=2)
        a.grid(True, alpha=0.3)
        a.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "target_vs_N.png"), dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Self-test (no build tree, no GNU Radio run)
# ---------------------------------------------------------------------------
def self_test(verbose=True) -> int:
    fails = 0

    def check(name, cond, detail=""):
        nonlocal fails
        if verbose:
            print("%-56s %s %s" % (name, "PASS" if cond else "FAIL", detail))
        if not cond:
            fails += 1

    fails += AM.self_test(verbose=verbose)

    op = load_json(testdata_dir("operator.json"))
    cal = set(window_seed(op["seeds"]["calibration"], ci, wi)
              for ci in range(12) for wi in range(1000))
    acc = set(window_seed(op["seeds"]["acceptance"], ci, wi)
              for ci in range(12) for wi in range(1000))
    check("calibration/acceptance seed windows disjoint", not (cal & acc),
          "|cal|=%d |acc|=%d" % (len(cal), len(acc)))
    check("acceptance seed windows unique", len(acc) == 12 * 1000)
    check("64 SYNC support = 65024 samples", AM.SYNC_REPS * SPS == 65024)
    cells = load_json(testdata_dir("cells.json"))
    ids = [c["id"] for c in cells["cells"]]
    check("12 acceptance cells declared", len(ids) == 12, str(len(ids)))
    check("cell ids unique", len(set(ids)) == len(ids))
    check("SNR fixed at 30 dB",
          float(op["evaluation"]["snr_db"]) == 30.0)
    check("A2 grid = 9 ISRxCFO + 3 extra", _grid_ok(cells))
    check("target ROI frozen on calibration profile",
          op["roi"]["target_offsets"] == [AM.TARGET_LO, AM.TARGET_HI]
          and op["roi"]["target_tap_offset"] == AM.TARGET_OFFSET)
    check("receiver tap mapping documented",
          op["roi"]["receiver_target_roi"] == list(AM.RX_TARGET_ROI)
          and op["roi"]["receiver_tap_offset"] == AM.RX_TAP_OFFSET)
    check("fixed-operator geometry matches accum_metrics",
          op["operator"]["tap_lo"] == AM.TAP_LO
          and op["operator"]["tap_hi"] == AM.TAP_HI
          and op["operator"]["origin"] == 1997
          and op["repetition_sets"]["full_N"] == list(AM.FULL_N)
          and op["repetition_sets"]["indep_nominal_N"]
          == list(AM.INDEP_N_NOMINAL)
          and op["repetition_sets"]["indep_policy"] == "support_disjoint")
    check("summary aggregation smoke", _summarize_smoke() == 0)
    check("independent verifier present",
          os.path.exists(os.path.join(repo_root(), "tools", "sic",
                                      "verify_s2a.py")))
    if verbose:
        print("app self-test: %d failure(s)" % fails)
    return fails


def _grid_ok(cells) -> bool:
    got = set((c["isr_db"], c["cfo_hz"], c["comm_paths"],
               float(c["start_offset_work"])) for c in cells["cells"])
    want = {(isr, cfo, "single", 0.0) for isr in (0, 10, 20)
            for cfo in (0, -50000, 50000)}
    want |= {(0, 50000, "four", 0.0), (10, 0, "four", 0.0),
             (10, 0, "single", 0.4)}
    return got == want


def _summarize_smoke() -> int:
    def entry(N):
        return {"c_cir_db": {"full": 10.0, "target": None, "background": 5.0,
                             "direct": 1.0},
                "c_cir_flag": {"full": "numeric", "target": "na",
                               "background": "numeric", "direct": "numeric"},
                "e_raw": {"full": 1.0, "target": 0.0, "background": 1.0,
                          "direct": 1.0},
                "e_sic": {"full": 0.1, "target": 0.0, "background": 0.1,
                          "direct": 0.1},
                "nmse_raw": {"full": 1.0, "target": None, "background": 1.0,
                             "direct": 1.0},
                "nmse_sic": {"full": 0.1, "target": None, "background": 0.1,
                             "direct": 0.1},
                "g_sic_db": 3.0, "g_raw_db": 3.0,
                "residual_raw_db": {"full": 0.0, "target": None,
                                    "background": 0.0, "direct": 0.0},
                "residual_sic_db": {"full": -10.0, "target": None,
                                    "background": -10.0, "direct": -10.0},
                "N": int(N), "start": 0, "stop": int(N)}

    full = {str(N): entry(N) for N in AM.FULL_N}
    ind = {str(N): entry(N) for N in (1, 2, 4, 8, 16, 32, 54)}
    tgt = {"%s/%d" % (s, N): {"raw": {"amp_err_db": 1.0, "peak_shift": 0,
                                      "phase_err_deg": 1.0,
                                      "bg_false_peak_ratio": 0.1,
                                      "target_to_background_proxy": 10.0},
                             "sic": {"amp_err_db": 0.2, "peak_shift": 0,
                                     "phase_err_deg": 0.2,
                                     "bg_false_peak_ratio": 0.05,
                                     "target_to_background_proxy": 20.0}}
           for s, lst in (("full", AM.FULL_N),
                          ("indep", (1, 2, 4, 8, 16, 32, 54)))
           for N in lst}
    recs = []
    for i in range(3):
        recs.append({
            "window_id": i, "cell_id": "c", "kind": "calibration", "seed": i,
            "n_samples": 100, "comm_present": True, "isr_db_target": 10,
            "isr_db_realized": 10.0, "cfo_hz_injected": 0.0,
            "start_offset_work": 0.0, "comm_channel": "single",
            "overlap": {"lo": 0, "hi": 1, "samples": 1},
            "sic": {"committed": True, "detected": True, "partial": False,
                    "bypass_reason": None, "state": "committed",
                    "uses_truth": False, "cfo_hz_est": 0.0, "n_paths": 1,
                    "support_lo": 0, "support_hi": 1, "sub_lo": 0, "sub_hi": 1,
                    "fit_residual": 0.1, "cond2": 1.0, "model_id": 1,
                    "model_fingerprint": "0x1", "residual_corr": None,
                    "support_source": "configured_nominal_prediction",
                    "apply_refit_count": 0, "core_total_us": 1.0},
            "iq": {"span": {"suppression_db": 20.0},
                   "window": {"suppression_db": 20.0},
                   "bypass_bit_identical": False, "output_changed": True,
                   "raw_minus_clean_energy": 1.0,
                   "sic_minus_clean_energy": 0.01},
            "accum": {"full": full, "indep": ind,
                      "sets": {"full": {"n_list": list(AM.FULL_N)},
                               "indep": {"n_list": [1, 2, 4, 8, 16, 32, 54]}}},
            "target": tgt,
            "support": {"cell_set": {"first": 9, "last": 62, "available": 54,
                                     "n_list": [1, 2, 4, 8, 16, 32, 54]},
                        "cell_set_contained": True, "set_used": "cell",
                        "coverage": {"cir_overlap_sub_ratio": 1.0,
                                     "cir_overlap_truth_ratio": 0.9},
                        "window_set": {"first": 9, "last": 62},
                        "excluded": []},
            "coherence": {t: {roi: {"eta": 0.5, "coherent_fraction": 0.5,
                                    "lag": {"1": 0.5},
                                    "centered_lag": {"1": 0.5},
                                    "phase_slope_hz": 0.0,
                                    "e_coherent": 1.0, "e_centered": 1.0,
                                    "e_total": 2.0, "mu_energy": 0.1,
                                    "energy_conservation_rel": 0.0,
                                    "eta_flag": "numeric"}
                              for roi in ("target", "background", "all")}
                          for t in ("sic", "raw")},
            "coherence_full_preamble": {t: {roi: {"eta": 0.5,
                                                  "coherent_fraction": 0.5,
                                                  "lag": {"1": 0.5},
                                                  "centered_lag": {"1": 0.5},
                                                  "phase_slope_hz": 0.0,
                                                  "e_coherent": 1.0,
                                                  "e_centered": 1.0,
                                                  "e_total": 2.0,
                                                  "mu_energy": 0.1,
                                                  "energy_conservation_rel":
                                                      0.0,
                                                  "eta_flag": "numeric"}
                                          for roi in ("target", "background",
                                                      "all")}
                                      for t in ("sic", "raw")},
            "per_symbol": {"residual_sic_power": [1.0] * 64,
                           "residual_raw_power": [1.0] * 64,
                           "clean_power": [1.0] * 64},
        })
    s = summarize_cell(recs)
    return 0 if s and s["windows"] == 3 and s["target"] else 1


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def build_parser():
    ap = argparse.ArgumentParser(
        description="S2-A accumulation-after-SIC sensing CIR evaluation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--build-dir", help="gr-uwb build tree (build-sic-accum)")
    ap.add_argument("--output", help="run directory under build-sic-accum/"
                                     "results")
    ap.add_argument("--cell", action="append", default=None,
                    help="cell id to run (repeatable; default all 12)")
    ap.add_argument("--windows-per-cell", type=int, default=100)
    ap.add_argument("--seed-set", choices=("calibration", "acceptance"),
                    default="acceptance")
    ap.add_argument("--i-know-this-is-calibration", action="store_true",
                    help="required to use the calibration seed set")
    ap.add_argument("--archive-windows", type=int, default=8,
                    help="first N windows per cell archived in full")
    ap.add_argument("--receiver", action="store_true",
                    help="also run the existing radar CIR block per branch")
    ap.add_argument("--receiver-windows", type=int, default=20,
                    help="maximum windows per cell that get the receiver layer")
    ap.add_argument("--stage", default="rectification",
                    help="run stage tag; the run directory name must contain it")
    ap.add_argument("--plots", action="store_true")
    ap.add_argument("--bypass", action="store_true",
                    help="A3: no interference; bypass/target-protection mode")
    ap.add_argument("--smoke", action="store_true",
                    help="A1: nominal cell, 8 calibration windows, archived")
    ap.add_argument("--self-test", action="store_true")
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(verbose=True)
    if args.smoke:
        args.cell = args.cell or ["isr10_cfo0_single"]
        if args.windows_per_cell == 100:
            args.windows_per_cell = 8
        args.archive_windows = max(8, args.archive_windows)
        args.seed_set = "calibration"
        args.i_know_this_is_calibration = True
    if not args.build_dir or not args.output:
        ap.error("--build-dir and --output are required for a run")
    base = os.path.basename(os.path.abspath(args.output))
    if args.stage and args.stage not in base:
        new_out = os.path.abspath(args.output) + "_" + args.stage
        print("run stage %r not in the output name; using %s"
              % (args.stage, new_out), flush=True)
        args.output = new_out
    if not args.i_know_this_is_calibration and args.seed_set == "calibration":
        raise SystemExit("--seed-set calibration requires "
                         "--i-know-this-is-calibration")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
