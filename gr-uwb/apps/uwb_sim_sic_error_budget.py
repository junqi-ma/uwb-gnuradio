#!/usr/bin/env python3
"""A-R03: SIC residual-error source budget (offline diagnostics, uses_truth).

Scientific question (``docs/sic/S2A_整改意见_2026-10-18.md`` section 3): the
initial S2-A report measured a *coherent* residual floor after the S1
``estimate8_frozen`` cancellation.  This driver quantifies the contributions of

* generator/template/interpolation/finite-edge mismatch,
* integer/fractional delay, complex-gain and path-selection error,
* CFO estimation error and the phase-reference/gain coupling,
* the training-region structural contamination by the unknown sensing signal,
* training-AWGN-induced parameter uncertainty,
* CF32 quantisation / reconstruction-numerical floor / support over-reach,

without modifying the shared SIC core and without ever feeding truth into the
online path.

Design
------
* The **online** result is produced by the real ``uwb.preamble_sic`` PDU
  adapter exactly as in ``uwb_sim_sic_accum.py`` (reused, not re-implemented).
* Every **diagnostic** reconstruction uses the frozen NumPy reference
  ``tools/sic/sic_reference.py`` (imported, never modified) and is validated
  against the adapter's own published ``i_hat`` before any conclusion.
* The truth-assisted substitutions live in ``tools/sic/residual_budget.py`` in
  a clearly separated ``uses_truth=True`` namespace; the production adapter
  never sees truth.
* Minimum coverage: the four mandated classes (nominal, hard CFO, fractional
  offset, four-path) plus the ISR 0/20 nominal cells for the absolute-residual
  ISR sweep; 8 windows for the smoke, >= 100 acceptance windows for the main
  comparison.

CLI
---
    # QA (no build tree)
    python3 gr-uwb/apps/uwb_sim_sic_error_budget.py --self-test

    # smoke: 4 classes x 8 calibration windows
    python3 gr-uwb/apps/uwb_sim_sic_error_budget.py \
        --build-dir gr-uwb/build-sic-accum \
        --output gr-uwb/build-sic-accum/results/s2a/budget_smoke \
        --windows 8 --seed-set calibration --i-know-this-is-calibration

    # main: all classes x 100 acceptance windows
    python3 gr-uwb/apps/uwb_sim_sic_error_budget.py \
        --build-dir gr-uwb/build-sic-accum \
        --output gr-uwb/build-sic-accum/results/s2a/budget \
        --windows 100 --seed-set acceptance --plots
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import sys
import time
from datetime import datetime, timezone

import numpy as np
import pmt  # noqa: F401  (re-exported through uwb_sim_sic_accum)

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.abspath(
    os.path.join(_HERE, os.pardir, os.pardir, "tools", "sic")))

import sic_preamble_sim as S  # noqa: E402
import uwb_sim_preamble_sic as APP  # noqa: E402
import accum_metrics as AM  # noqa: E402
import residual_budget as RB  # noqa: E402
import uwb_sim_sic_accum as ACC  # noqa: E402

SCHEMA_VERSION = 2
STAGE = "rectification"

#: mandated classes + the ISR sweep cells
DEFAULT_CLASSES = ["isr10_cfo0_single", "isr0_cfo+50k_single",
                   "isr10_cfo0_off0.4_single", "isr10_cfo0_four",
                   "isr0_cfo0_single", "isr20_cfo0_single"]
MANDATED_CLASSES = ["isr10_cfo0_single", "isr0_cfo+50k_single",
                    "isr10_cfo0_off0.4_single", "isr10_cfo0_four"]
SENS_CFO_LIST = (-400.0, -100.0, 100.0, 400.0)
SENS_TAU_LIST = (-0.2, -0.05, 0.05, 0.2)
VARIANT_ORDER = ["S0_model", "S1_cfo_truth", "S2_tau_truth", "S3_gain_truth",
                 "S4_pathset_truth", "S5_tau_cfo_truth", "S6_full_truth",
                 "C1_cfo_truth_gainrefit", "C2_tau_cfo_truth_gainrefit"]


def _vec(msg_meta, key):
    """Read a pmt f64vector from an adapter out-meta dict."""
    v = msg_meta.get(key)
    if v is None:
        return []
    try:
        return [float(x) for x in pmt.f64vector_elements(v)]
    except (TypeError, ValueError):
        try:
            return [float(x) for x in v]
        except TypeError:
            return []


def published_model_from_meta(meta):
    """Model dict (canonical RB form) from the adapter's published evidence."""
    taus = _vec(meta, "sic_tau")
    gre = _vec(meta, "sic_gain_re")
    gim = _vec(meta, "sic_gain_im")
    if not taus or len(gre) != len(taus) or len(gim) != len(taus):
        return None
    return RB.model_of(taus, [complex(a, b) for a, b in zip(gre, gim)],
                       cfo_hz=ACC.dnum(meta, "sic_cfo_hz", 0.0),
                       phase_reference_sample=int(ACC.dnum(
                           meta, "sic_phase_reference", 0)),
                       sample_rate=float(ACC.dnum(meta, "sic_sample_rate",
                                                  S.WORK_HZ)))


def _nmse(a, b):
    a = np.asarray(a, dtype=np.complex128)
    b = np.asarray(b, dtype=np.complex128)
    d = float(np.sum(np.abs(a - b) ** 2))
    c = float(np.sum(np.abs(b) ** 2))
    return (d / c) if c > 0 else None


def _ratio_db(num, den):
    if num <= 0.0 and den <= 0.0:
        return None
    if den <= 0.0:
        return float("inf")
    if num <= 0.0:
        return float("-inf")
    return 10.0 * math.log10(num / den)


def evaluate_budget(uwb, generator, op, cell, cp, seed, snr_db, sic_runner,
                    support, sensing_roi, observed_end_public, wid,
                    *, preamble, do_sensitivity=False):
    """One window: online model + truth-assisted budget + training controls."""
    win, clean, raw, sic_iq, ihat, out_meta = ACC._make_window(
        generator, sic_runner, cell=cell, cp=cp, seed=seed, snr_db=snr_db,
        sensing_roi=sensing_roi, observed_end_public=observed_end_public,
        wid=wid)
    n = int(raw.size)
    sub_lo = int(ACC.dnum(out_meta, "sic_sub_lo", -1))
    sub_hi = int(ACC.dnum(out_meta, "sic_sub_hi", -1))
    support_lo = int(ACC.dnum(out_meta, "sic_support_lo", -1))
    support_hi = int(ACC.dnum(out_meta, "sic_support_hi", -1))
    train_lo = int(ACC.dnum(out_meta, "sic_training_lo", -1))
    train_hi = int(ACC.dnum(out_meta, "sic_training_hi", -1))
    model = published_model_from_meta(out_meta)
    committed = bool(ACC.dget(out_meta, "sic_committed", False))
    rec = {"window_id": int(wid), "cell_id": cell["id"], "seed": int(seed),
           "n_samples": n, "committed": committed,
           "isr_db_realized": win.get("isr_db"),
           "committed_model": model is not None,
           "sub_lo": sub_lo, "sub_hi": sub_hi, "support_lo": support_lo,
           "support_hi": support_hi, "training_lo": train_lo,
           "training_hi": train_hi,
           "online_iq_suppression_db": ACC.AM.iq_suppression_db(
               raw, clean, sic_iq, 0, n)["suppression_db"]}
    if model is None:
        rec["error"] = "no published model"
        return rec, None
    i_true_wave = np.asarray(win["i"], dtype=np.complex128)
    truth = RB.truth_model(generator.pre, win["truth"]["comm"],
                           sample_rate=S.WORK_HZ)
    plen = int(preamble.size)
    rows = RB.build_polyphase_rows(preamble)
    # full natural support of the published model (what the adapter publishes)
    flo, fhi = RB.support_span(model, n, plen)
    # applied subtraction region intersected with that support
    slo = max(int(sub_lo), flo)
    shi = min(int(sub_hi), fhi)
    i_true_full = RB.reconstruct_in(truth, preamble, flo, fhi, rows=rows)
    i_model_full = RB.reconstruct_in(model, preamble, flo, fhi, rows=rows)
    i_true_sub = RB.reconstruct_in(truth, preamble, slo, shi, rows=rows)
    i_model_sub = RB.reconstruct_in(model, preamble, slo, shi, rows=rows)
    i_true = np.zeros(n, dtype=np.complex128)
    i_true[slo:shi] = i_true_sub
    i_model = np.zeros(n, dtype=np.complex128)
    i_model[slo:shi] = i_model_sub
    rec["reconstruction_validation"] = {
        "nmse_diag_vs_adapter_ihat": _nmse(
            i_model_full, np.asarray(ihat, dtype=np.complex128)[flo:fhi]),
        "region": [flo, fhi],
        "core_operator": "the C++ core's 32-phase polyphase blend operator",
        "note": "diagnostic reconstruction vs the adapter's own i_hat over the "
                "model's full support; a large value invalidates the "
                "truth-assisted conclusions"}
    rec["impl_mismatch_full"] = _nmse(i_true_full,
                                     i_true_wave[flo:fhi])
    # parameter error table
    pairing = RB.pair_paths(model, truth)
    ef = RB.effective_gains(model)
    tg = [complex(g) for g in truth["gains"]]
    rec["params"] = {
        "model_origin": int(ACC.dnum(out_meta, "sic_detected_start", -1)),
        "truth_origin_exact": float(win["truth"]["comm"]["start_sample_exact"]),
        "origin_err": (float(ACC.dnum(out_meta, "sic_detected_start", -1))
                       - float(win["truth"]["comm"]["start_sample_exact"])),
        "model_cfo_hz": float(model["cfo_hz"]),
        "truth_cfo_hz": float(truth["cfo_hz"]),
        "cfo_err_hz": float(model["cfo_hz"]) - float(truth["cfo_hz"]),
        "n_paths_model": int(model["n_paths"]),
        "n_paths_truth": int(truth["n_paths"]),
        "pairing": pairing,
        "model_gains_truth_frame": [[g.real, g.imag] for g in ef],
        "truth_gains": [[g.real, g.imag] for g in tg],
        "gain_mag_err_db_mean": ACC._mean(
            [p["gain_mag_err_db"] for p in pairing["pairs"]]),
        "gain_phase_err_deg_mean": ACC._mean(
            [p["gain_phase_err_deg"] for p in pairing["pairs"]]),
    }
    # IQ budget with the published model
    rec["iq_budget_model"] = RB.iq_budget(
        i_true_wave, i_true, i_model, lo=slo, hi=shi)
    # truth-assisted variants
    svars = RB.truth_variants(model, truth, np.asarray(raw, dtype=np.complex128),
                             train_lo=train_lo, train_hi=train_hi,
                             preamble=preamble, sample_rate=S.WORK_HZ)
    legal = np.asarray(support["reps"], dtype=np.int64)
    N_main = int(support["n_list"][-1])
    idx = legal[:N_main]
    H_clean = AM.fixed_operator_cir(clean, op)
    hc = AM.accumulate(H_clean, idx)
    tlo, thi = AM.TARGET_LO - op.tap_lo, AM.TARGET_HI - op.tap_lo
    variants = {}
    res_vecs = {}
    for name in VARIANT_ORDER:
        if name not in svars:
            continue
        vm = svars[name]
        vlo, vhi = RB.support_span(vm, n, plen)
        vlo = max(int(slo), int(vlo))
        vhi = min(int(shi), int(vhi))
        ihat_v = np.zeros(n, dtype=np.complex128)
        if vhi > vlo:
            ihat_v[vlo:vhi] = RB.reconstruct_in(vm, preamble, vlo, vhi,
                                                rows=rows)
        sic_v = np.asarray(raw, dtype=np.complex64) - np.asarray(
            ihat_v, dtype=np.complex64)
        H_v = AM.fixed_operator_cir(sic_v, op)
        E = H_v - H_clean
        variants[name] = {
            "description": RB.VARIANT_SPECS.get(name),
            "uses_truth": name != "S0_model",
            "cir_residual_full_db": _ratio_db(
                AM.energy(AM.accumulate(E, idx)), AM.energy(hc)),
            "cir_residual_target_db": _ratio_db(
                AM.energy(AM.accumulate(E, idx), tlo, thi),
                AM.energy(hc, tlo, thi)),
            "cir_residual_far_db": _ratio_db(
                AM.energy(AM.accumulate(E, idx), int(
                    AM.BG_LO - op.tap_lo), int(AM.BG_HI - op.tap_lo)),
                AM.energy(hc, int(AM.BG_LO - op.tap_lo),
                          int(AM.BG_HI - op.tap_lo))),
            "cir_energy_full": AM.energy(AM.accumulate(E, idx)),
            "iq_masked_energy": float(np.sum(np.abs(
                np.asarray(raw, dtype=np.complex128)[sub_lo:sub_hi]
                - np.asarray(ihat_v, dtype=np.complex128)[sub_lo:sub_hi])
                ** 2)),
            "iq_residual_energy_vs_truth": float(np.sum(np.abs(
                i_true_wave[sub_lo:sub_hi]
                - np.asarray(ihat_v, dtype=np.complex128)[sub_lo:sub_hi])
                ** 2)),
        }
        res_vecs[name] = E[legal].astype(np.complex64)
    rec["variants"] = variants
    rec["budget_identity"] = RB.iq_budget(i_true_wave, i_true, i_model,
                                          lo=slo, hi=shi)
    rec["impl_mismatch_db_vs_clean"] = _ratio_db(
        float(np.sum(np.abs(i_true_wave[flo:fhi] - i_true_full) ** 2)),
        float(np.sum(np.abs(i_true_wave[flo:fhi]) ** 2)))
    if do_sensitivity:
        rec["sensitivity"] = RB.sensitivity(
            truth, y=np.asarray(raw, dtype=np.complex128), preamble=preamble,
            lo=sub_lo, hi=sub_hi, dcfo_list=SENS_CFO_LIST,
            dtau_list=SENS_TAU_LIST, sample_rate=S.WORK_HZ)
    # training-input controls: same i / s / w and the same scale, no
    # renormalisation.  Each control re-estimates a model OFFLINE (diagnostic).
    controls = {}
    for cname, sig in (("i_only", np.asarray(win["i"], dtype=np.complex64)),
                       ("s_plus_i", np.asarray(win["s"] + win["i"],
                                               dtype=np.complex64)),
                       ("i_plus_w", np.asarray(win["i"] + win["w"],
                                               dtype=np.complex64))):
        cmeta = ACC.rx_meta(wid, n, S.DEFAULT_SYNC_REPS, generator.pre)
        cmeta = ACC.put_int(cmeta, "sensing_roi_lo", sensing_roi[0])
        cmeta = ACC.put_int(cmeta, "sensing_roi_hi", sensing_roi[1])
        cmeta = ACC.put_int(cmeta, "observed_end_sample", observed_end_public)
        sic_runner.post(cmeta, np.ascontiguousarray(sig))
        res = sic_runner.wait()
        if res is None:
            controls[cname] = {"error": "timeout"}
            continue
        cm, cih_meta = res
        cih = np.asarray(cih_meta.get("__c32", np.zeros_like(sig)),
                         dtype=np.complex128)
        cm_model = published_model_from_meta(cm)
        csic = np.asarray(cm.get("__c32", sig), dtype=np.complex64)
        # Diagnostics for a CONTROL input.  ``cir_residual vs clean`` is NOT
        # meaningful here (the control may contain no sensing signal at all), so
        # the meaningful quantities are computed in the IQ domain over the exact
        # applied mask:
        #   e_residual  = ||input - i_hat||^2        (what is left uncancelled)
        #   e_fit_truth = ||i_true - i_hat||^2        (model error vs the true
        #                                             interference waveform)
        #   e_input     = ||input||^2                (normaliser)
        y_c = np.asarray(sig, dtype=np.complex128)[sub_lo:sub_hi]
        ih_c = cih[sub_lo:sub_hi].copy()
        # a control model is only applied inside its own published region
        mlo = int(ACC.dnum(cm, "sic_sub_lo", sub_lo))
        mhi = int(ACC.dnum(cm, "sic_sub_hi", sub_hi))
        if mlo > sub_lo or mhi < sub_hi:
            mask = np.zeros(sub_hi - sub_lo, dtype=bool)
            mask[max(0, mlo - sub_lo):max(0, mhi - sub_lo)] = True
            ih_c = np.where(mask, ih_c, 0.0)
        e_input = float(np.sum(np.abs(y_c) ** 2))
        e_res = float(np.sum(np.abs(y_c - ih_c) ** 2))
        e_fit = float(np.sum(np.abs(
            i_true_wave[sub_lo:sub_hi] - ih_c) ** 2))
        ctrl = {
            "committed": bool(ACC.dget(cm, "sic_committed", False)),
            "bypass_reason": ACC.dget(cm, "sic_bypass_reason", None),
            "cfo_hz_est": ACC.dnum(cm, "sic_cfo_hz", None),
            "n_paths": int(ACC.dnum(cm, "sic_n_paths", -1)),
            "fit_residual": ACC.dnum(cm, "sic_fit_residual", None),
            "e_input": e_input,
            "e_residual_masked": e_res,
            "e_fit_vs_truth": e_fit,
            "cancel_db": _ratio_db(e_res, e_input),
            "fit_vs_truth_db": _ratio_db(e_fit, e_input),
            "aci_energy_masked": float(np.sum(np.abs(
                np.asarray(sig, dtype=np.complex128)[sub_lo:sub_hi]) ** 2)),
        }
        if cm_model is not None:
            ctrl["cfo_err_hz"] = float(cm_model["cfo_hz"]) \
                - float(truth["cfo_hz"])
            ctrl["params"] = {
                "taus": [float(t) for t in cm_model["taus"]],
                "n_paths": int(cm_model["n_paths"]),
            }
        controls[cname] = ctrl
    rec["training_controls"] = controls
    rec["_arrays"] = {"clean": clean, "raw": raw, "sic": sic_iq, "ihat": ihat,
                      "i_truth": np.asarray(win["i"], dtype=np.complex64),
                      "model_taus": [float(t) for t in model["taus"]],
                      "model_gains": [[float(g.real), float(g.imag)]
                                      for g in model["gains"]],
                      "model_cfo_hz": float(model["cfo_hz"]),
                      "model_phase_reference": int(
                          model["phase_reference_sample"]),
                      "truth_taus": [float(t) for t in truth["taus"]],
                      "truth_gains": [[float(g.real), float(g.imag)]
                                      for g in truth["gains"]],
                      "truth_cfo_hz": float(truth["cfo_hz"]),
                      "sub_lo": int(sub_lo), "sub_hi": int(sub_hi),
                      "support_lo": int(support_lo),
                      "support_hi": int(support_hi),
                      "region": [int(flo), int(fhi)],
                      "variant_models": {k: {"taus": [float(t) for t in v["taus"]],
                                             "gains": [[float(g.real), float(g.imag)]
                                                       for g in v["gains"]],
                                             "cfo_hz": float(v["cfo_hz"]),
                                             "phase_reference_sample": int(
                                                 v["phase_reference_sample"])}
                                         for k, v in svars.items()}}
    rec["support_disjoint"] = {"first": int(support["first"]),
                               "last": int(support["last"]),
                               "N_main": N_main, "n_list": list(
                                   support["n_list"])}
    return rec, res_vecs


def summarize_class(recs, res_vecs_by_win):
    """Per-class aggregates: parameter bias, variant ratios, bias/fluctuation."""
    n = len(recs)
    out = {"windows": n,
           "committed": sum(1 for r in recs if r.get("committed")),
           "model_windows": sum(1 for r in recs if r.get("committed_model")),
           "reconstruction_nmse_max": max(
               [r["reconstruction_validation"]["nmse_diag_vs_adapter_ihat"]
                for r in recs
                if r.get("reconstruction_validation")
                and r["reconstruction_validation"][
                    "nmse_diag_vs_adapter_ihat"] is not None] or [None]),
           "origin_err_mean": ACC._mean([r["params"]["origin_err"]
                                         for r in recs if r.get("params")]),
           "cfo_err_hz_mean": ACC._mean([r["params"]["cfo_err_hz"]
                                         for r in recs if r.get("params")]),
           "cfo_err_hz_std": (float(np.std([r["params"]["cfo_err_hz"]
                                            for r in recs
                                            if r.get("params")]))
                              if any(r.get("params") for r in recs) else None),
           "cfo_err_hz_p95_abs": ACC._pct(
               [abs(r["params"]["cfo_err_hz"]) for r in recs
                if r.get("params")], 95),
           "delay_err_abs_mean": ACC._mean(
               [r["params"]["pairing"]["delay_err_abs_mean"] for r in recs
                if r.get("params")]),
           "delay_err_abs_max": max([r["params"]["pairing"]["delay_err_abs_max"]
                                     for r in recs if r.get("params")
                                     and r["params"]["pairing"][
                                         "delay_err_abs_max"] is not None]
                                    or [None]),
           "gain_mag_err_db_mean": ACC._mean(
               [r["params"]["gain_mag_err_db_mean"] for r in recs
                if r.get("params")]),
           "gain_phase_err_deg_mean": ACC._mean(
               [r["params"]["gain_phase_err_deg_mean"] for r in recs
                if r.get("params")]),
           "n_paths_model_mean": ACC._mean(
               [r["params"]["n_paths_model"] for r in recs
                if r.get("params")]),
           "n_paths_truth": ACC._mean(
               [r["params"]["n_paths_truth"] for r in recs
                if r.get("params")]),
           "unpaired_truth_mean": ACC._mean(
               [len(r["params"]["pairing"]["unpaired_truth"]) for r in recs
                if r.get("params")]),
           "iq_suppression_db_mean": ACC._mean(
               [r["online_iq_suppression_db"] for r in recs]),
           "variants": {}, "training_controls": {}, "budget": {},
           "bias_vs_fluctuation": {},
           "impl_mismatch_db_mean": ACC._mean(
               [r.get("impl_mismatch_db_vs_clean") for r in recs]),
           "impl_mismatch_db_std": (float(np.std([
               r["impl_mismatch_db_vs_clean"] for r in recs
               if r.get("impl_mismatch_db_vs_clean") is not None]))
               if any(r.get("impl_mismatch_db_vs_clean") is not None
                      for r in recs) else None),
           "recon_validation_nmse_max": max(
               [r["reconstruction_validation"]["nmse_diag_vs_adapter_ihat"]
                for r in recs if r.get("reconstruction_validation")] or [None])}
    if n == 0:
        return out
    r0 = recs[0]
    for name in VARIANT_ORDER:
        vals = [r["variants"][name] for r in recs if name in r.get("variants", {})]
        if not vals:
            continue
        out["variants"][name] = {
            "description": vals[0]["description"],
            "uses_truth": vals[0]["uses_truth"],
            "cir_residual_full_db_mean": ACC._mean(
                [v["cir_residual_full_db"] for v in vals]),
            "cir_residual_full_db_ci": AM.bootstrap_ci(
                [v["cir_residual_full_db"] for v in vals
                 if v["cir_residual_full_db"] is not None]),
            "cir_residual_target_db_mean": ACC._mean(
                [v["cir_residual_target_db"] for v in vals]),
            "cir_residual_far_db_mean": ACC._mean(
                [v["cir_residual_far_db"] for v in vals]),
            "iq_masked_energy_mean": ACC._mean(
                [v["iq_masked_energy"] for v in vals]),
            "iq_residual_vs_truth_mean": ACC._mean(
                [v["iq_residual_energy_vs_truth"] for v in vals]),
            "cir_energy_full_mean": ACC._mean(
                [v["cir_energy_full"] for v in vals]),
        }
    for cname in ("i_only", "s_plus_i", "i_plus_w"):
        vals = [r["training_controls"][cname] for r in recs
                if r.get("training_controls", {}).get(cname)]
        if not vals:
            continue
        out["training_controls"][cname] = {
            "committed": sum(1 for v in vals if v.get("committed")),
            "e_input_mean": ACC._mean([v.get("e_input") for v in vals]),
            "e_residual_masked_mean": ACC._mean(
                [v.get("e_residual_masked") for v in vals]),
            "e_fit_vs_truth_mean": ACC._mean(
                [v.get("e_fit_vs_truth") for v in vals]),
            "cancel_db_mean": ACC._mean([v.get("cancel_db") for v in vals]),
            "fit_vs_truth_db_mean": ACC._mean(
                [v.get("fit_vs_truth_db") for v in vals]),
            "cfo_err_hz_mean": ACC._mean([v.get("cfo_err_hz")
                                          for v in vals]),
            "n_paths_mean": ACC._mean([v.get("n_paths") for v in vals]),
            "fit_residual_mean": ACC._mean([v.get("fit_residual")
                                            for v in vals]),
        }
    b = [r["budget_identity"] for r in recs if r.get("budget_identity")]
    if b:
        out["budget"] = {
            "e_impl_mismatch_mean": ACC._mean([x["e_impl_mismatch"]
                                               for x in b]),
            "e_param_mean": ACC._mean([x["e_param"] for x in b]),
            "e_cross_mean": ACC._mean([x["e_cross"] for x in b]),
            "e_total_mean": ACC._mean([x["e_total"] for x in b]),
            "identity_rel_max": max([x["check_sum_identity_rel"] for x in b]),
            "ratio_impl_over_total_mean": ACC._mean(
                [x["e_impl_mismatch"] / x["e_total"] for x in b
                 if x["e_total"] > 0]),
            "ratio_param_over_total_mean": ACC._mean(
                [x["e_param"] / x["e_total"] for x in b if x["e_total"] > 0]),
            "ratio_cross_over_total_mean": ACC._mean(
                [x["e_cross"] / x["e_total"] for x in b if x["e_total"] > 0]),
        }
    # deterministic bias vs cross-window fluctuation (CIR domain, model variant)
    mv = [v for k, v in (res_vecs_by_win or {}).items() if v is not None]
    if mv:
        A = np.stack([np.asarray(v, dtype=np.complex128) for v in mv], axis=0)
        mean_vec = A.mean(axis=0)
        e_mean = float(np.sum(np.abs(mean_vec) ** 2))
        e_fluct = float(np.mean(np.sum(np.abs(A - mean_vec) ** 2, axis=(1, 2))))
        per_win = np.sum(np.abs(A) ** 2, axis=(1, 2))
        out["bias_vs_fluctuation"] = {
            "n_windows": int(A.shape[0]),
            "mean_vector_energy": e_mean,
            "cross_window_fluctuation_energy": e_fluct,
            "fluctuation_over_mean": (e_fluct / e_mean if e_mean > 0 else None),
            "per_window_energy_mean": float(per_win.mean()),
            "per_window_energy_std": float(per_win.std()),
            "note": "the ensemble-mean residual vector is the deterministic "
                    "(window-repeatable) part; the centered part is the "
                    "cross-window fluctuation.  A phase ramp is deterministic "
                    "within a window but may rotate across seeds."}
    return out


def run(args) -> int:
    frozen_op = ACC.load_json(ACC.testdata_dir("operator.json"))
    cells_file = ACC.load_json(ACC.testdata_dir("cells.json"))
    cells = [c for c in cells_file["cells"] if c["id"] in args.classes]
    if not cells:
        raise SystemExit("no classes selected")
    outdir = os.path.abspath(args.output)
    for sub in ("templates", "figures", "logs", "operator", "archive"):
        os.makedirs(os.path.join(outdir, sub), exist_ok=True)
    seed_base = int(frozen_op["seeds"][args.seed_set])
    snr_db = float(frozen_op["evaluation"]["snr_db"])
    uwb = S.load_uwb(args.build_dir)
    loaded = ACC.library_provenance(args.build_dir)
    if not loaded["lib_exists"] or not loaded["in_worktree"]:
        raise SystemExit("bad build dir: %r" % (loaded,))
    generator = S.SicSignalGenerator(
        uwb, sensing_code=S.DEFAULT_SENSING_CODE, comm_code=S.DEFAULT_COMM_CODE,
        sync_reps=S.DEFAULT_SYNC_REPS, sfd_mode=ACC.SFD_MODE,
        pulse_shape=S.DEFAULT_PULSE_SHAPE, comm_present=True, build_comm=True,
        snr_db=snr_db, isr_db=10.0, verbose=True)
    # the A2 acceptance cells use the public visibility guard (offset 0.4), so
    # the SAME generator configuration is required for the budget windows to be
    # bit-identical to the acceptance run's windows.
    visibility_guard = 1
    generator.overlap_tail_guard = visibility_guard
    op, op_ev = ACC.build_operator(uwb, generator, frozen_op)
    roi_ev = ACC.verify_target_roi(generator, op, frozen_op, True)
    preamble = np.asarray(generator.comm_wave, dtype=np.complex128)
    generator.comm_wave.tofile(os.path.join(
        outdir, "templates", "comm_preamble_code10_998p4.cf32"))
    generator.comm_sync.tofile(os.path.join(
        outdir, "templates", "comm_sync_code10_998p4.cf32"))
    generator.template.tofile(os.path.join(
        outdir, "templates", "sync_template_code9_998p4.cf32"))
    np.savez(os.path.join(outdir, "operator", "operator_arrays.npz"),
             sampled_code=np.asarray(op.code, dtype=np.complex64),
             chips=np.asarray(op_ev["chips"], dtype=np.int8),
             pulse_taps=np.asarray(S.pulse_taps(uwb, generator.pulse_shape,
                                                2.5, 200.0), dtype=np.float64),
             sensing_template=np.asarray(generator.template,
                                         dtype=np.complex64))
    nominal_origin = int(generator.pre)
    fit_reps = int(frozen_op["sic"]["fit_reps"])
    validation_reps = int(frozen_op["sic"]["validation_reps"])
    observed_end_public = int(min(generator.n, nominal_origin
                                  + visibility_guard + fit_reps * S.SPS))
    search_roi = (max(0, nominal_origin - visibility_guard),
                  min(generator.n, nominal_origin + S.DEFAULT_SYNC_REPS * S.SPS))
    core_roi = (int(search_roi[0]),
                int(min(generator.n, search_roi[1] + S.DEFAULT_SYNC_REPS * S.SPS
                        + int(frozen_op["sic"]["post_tail"]))))
    sensing_roi = (int(nominal_origin),
                   int(min(generator.n, nominal_origin + S.DEFAULT_SYNC_REPS
                           * S.SPS + visibility_guard)))
    nominal_support_hi = int(min(generator.n, nominal_origin
                                 + S.DEFAULT_SYNC_REPS * S.SPS
                                 + int(frozen_op["sic"]["post_tail"])))
    support = AM.derive_rep_set(
        op, visible_end=observed_end_public,
        sub_lo=int(max(nominal_origin, sensing_roi[0])),
        sub_hi=int(min(nominal_support_hi, sensing_roi[1])),
        n=generator.n, tap_lo=op.tap_lo, tap_hi=op.tap_hi,
        name="support_disjoint", kind="support_disjoint")
    print("support-disjoint set: [%d,%d] n=%d n_list=%s"
          % (support["first"], support["last"], support["available"],
             support["n_list"]), flush=True)
    sic_params = {
        "sync": generator.comm_sync, "preamble": generator.comm_wave,
        "comm_code": S.DEFAULT_COMM_CODE, "sync_reps": S.DEFAULT_SYNC_REPS,
        "samples_per_symbol": S.SPS, "sample_rate": S.WORK_HZ,
        "k_paths": int(frozen_op["sic"]["k_paths"]), "fit_reps": fit_reps,
        "validation_reps": validation_reps, "roi_lo": core_roi[0],
        "roi_hi": core_roi[1],
        "coarse_stride": int(frozen_op["sic"]["coarse_stride"]),
        "mode": "estimate",
        "detect_threshold": float(frozen_op["sic"]["detect_threshold"]),
        "validate_threshold": float(frozen_op["sic"]["validate_threshold"]),
    }
    sic_extras = {
        "in_flight_max": 8, "iq_budget_bytes": 64 << 20, "queue_capacity": 8,
        "publish_reconstruction": True,
        "post_tail": int(frozen_op["sic"]["post_tail"]),
        "enable_fractional": True, "enable_cfo": True, "path_min_spacing": 0,
        "max_window_samples": 0, "support_mode": "configured_nominal",
        "online_validation_reps": 0,
        "fit_residual_max": float(frozen_op["sic"]["fit_residual_max"]),
    }
    runner = ACC.SicRunner(uwb, sic_params, sic_extras)
    start = time.time()
    jsonl = open(os.path.join(outdir, "error_budget.jsonl"), "w",
                 encoding="utf-8")
    per_class = {}
    archived = []
    cross_check = []
    try:
        for cell in cells:
            cp = cells_file["comm_paths"][cell["comm_paths"]]
            recs, vecs = [], {}
            for wi in range(int(args.windows)):
                ci = next(i for i, c in enumerate(cells_file["cells"])
                          if c["id"] == cell["id"])
                seed = ACC.window_seed(seed_base, ci, wi)
                rec, res_vecs = evaluate_budget(
                    uwb, generator, op, cell, cp, seed, snr_db, runner,
                    support, sensing_roi, observed_end_public, wi,
                    preamble=preamble,
                    do_sensitivity=(cell["id"] == "isr10_cfo0_single"))
                arrays = rec.pop("_arrays", None)
                recs.append(rec)
                jsonl.write(json.dumps(rec, allow_nan=True, default=str)
                            + "\n")
                if arrays is not None and wi < int(args.archive_windows):
                    if args.cross_check_run:
                        ref = os.path.join(
                            args.cross_check_run, "archive",
                            "%s_%05d.npz" % (cell["id"], wi))
                        if os.path.exists(ref):
                            zr = np.load(ref, allow_pickle=False)
                            ident = bool(np.array_equal(
                                arrays["raw"], np.asarray(zr["raw"]))
                                and np.array_equal(
                                    arrays["clean"],
                                    np.asarray(zr["clean"])))
                            cross_check.append({
                                "cell_id": cell["id"], "window_id": wi,
                                "identical": ident})
                            if not ident:
                                raise SystemExit(
                                    "budget window %s/%d is NOT bit-identical "
                                    "to the reference run; the reuse claim is "
                                    "invalid" % (cell["id"], wi))
                    np.savez_compressed(
                        os.path.join(outdir, "archive",
                                     "%s_%05d.npz" % (cell["id"], wi)),
                        clean=arrays["clean"], raw=arrays["raw"],
                        sic=arrays["sic"], ihat=arrays["ihat"],
                        i_truth=arrays["i_truth"],
                        params_json=np.frombuffer(json.dumps({
                            k: v for k, v in arrays.items()
                            if k not in ("clean", "raw", "sic", "ihat",
                                         "i_truth")},
                            default=str).encode("utf-8"), dtype=np.uint8))
                    archived.append("%s_%05d.npz" % (cell["id"], wi))
                if res_vecs is not None:
                    vecs[wi] = res_vecs.get("S0_model")
                if (wi + 1) % 10 == 0 or wi + 1 == int(args.windows):
                    print("[%s] %d/%d (%.1fs)"
                          % (cell["id"], wi + 1, args.windows,
                             time.time() - start), flush=True)
            per_class[cell["id"]] = summarize_class(recs, vecs)
    finally:
        jsonl.close()
        runner.stop()

    summary = {
        "schema_version": SCHEMA_VERSION, "task": "S2-A A-R03",
        "stage": STAGE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "classes": list(per_class.keys()),
        "mandated_classes": MANDATED_CLASSES,
        "windows": int(args.windows), "seed_set": args.seed_set,
        "seed_base": seed_base,
        "support_disjoint": {"first": support["first"],
                             "last": support["last"],
                             "available": support["available"],
                             "n_list": list(support["n_list"])},
        "operator": {"origin": int(op.origin), "sha256": AM.sha256_f32(
            op.code), "roi_verification": roi_ev},
        "per_class": per_class,
        "truth_isolation": {
            "online_uses_truth": False,
            "diagnostic_uses_truth": True,
            "note": "every S1..S6/C1/C2 variant and the training controls are "
                    "offline diagnostics (uses_truth=True).  The production "
                    "adapter never receives truth and the shared core is "
                    "unmodified.",
            "allowed_conclusion": "multi-factor / not uniquely separable is an "
                                  "acceptable result; no exact percentages are "
                                  "invented",
        },
        "cross_check_run": args.cross_check_run,
        "cross_check": {
            "compared": len(cross_check),
            "identical": sum(1 for c in cross_check if c["identical"]),
            "mismatches": [c for c in cross_check if not c["identical"]],
            "note": "the budget windows are regenerated deterministically; every "
                    "compiled window that also exists in the reference run is "
                    "compared bit-for-bit (raw and clean) and a mismatch aborts "
                    "the run"},
        "archive": {"windows_per_cell": int(args.archive_windows),
                    "archived": archived,
                    "rule": "first --archive-windows windows of each class, by "
                            "window index only; the remaining windows cannot "
                            "be recomputed from the archive"},
        "not_run": {"matlab": "MATLAB not executed",
                    "native": "native grid not exercised",
                    "throughput": "no throughput acceptance in this round"},
        "timing": {"wall_s": time.time() - start,
                   "mode": "diagnostic (no benchmark lock held)"},
    }
    with open(os.path.join(outdir, "error_budget_summary.json"), "w",
              encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True, allow_nan=True)
    with open(os.path.join(outdir, "budget_variants.csv"), "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["class_id", "variant", "uses_truth",
                    "cir_residual_full_db_mean", "cir_residual_target_db_mean",
                    "cir_residual_far_db_mean", "iq_masked_energy_mean",
                    "cir_energy_full_mean"])
        for cid, cs in per_class.items():
            for name, v in (cs.get("variants") or {}).items():
                w.writerow([cid, name, v.get("uses_truth"),
                            v.get("cir_residual_full_db_mean"),
                            v.get("cir_residual_target_db_mean"),
                            v.get("cir_residual_far_db_mean"),
                            v.get("iq_masked_energy_mean"),
                            v.get("cir_energy_full_mean")])
    manifest = {
        "schema_version": SCHEMA_VERSION, "task": "S2-A A-R03",
        "stage": STAGE, "created_utc": summary["created_utc"],
        "git": ACC.git_provenance(ACC.repo_root()),
        "build_dir": loaded["build_dir"], "loaded_library": loaded,
        "python": sys.version.split()[0], "numpy": np.__version__,
        "platform": platform.platform(),
        "frozen_config": frozen_op["sic"],
        "geometry": {"pre_guard": nominal_origin, "n": generator.n},
        "rois": {"search_roi": list(search_roi), "core_roi": list(core_roi),
                 "sensing_roi": list(sensing_roi),
                 "observed_end_public": observed_end_public},
        "inputs_hashed": ACC._input_hashes(),
        "cli": " ".join(sys.argv),
        "not_run": summary["not_run"],
    }
    with open(os.path.join(outdir, "run_manifest.json"), "w",
              encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2, sort_keys=True, allow_nan=True)
    if args.plots:
        try:
            make_plots(outdir, summary)
        except Exception as exc:
            print("plots skipped: %r" % (exc,), flush=True)
    print(json.dumps({"output": outdir, "classes": list(per_class.keys()),
                      "wall_s": summary["timing"]["wall_s"]}, indent=2))
    return 0


def make_plots(outdir, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig_dir = os.path.join(outdir, "figures")
    os.makedirs(fig_dir, exist_ok=True)
    classes = summary["classes"]
    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(VARIANT_ORDER))
    for cid in classes:
        cs = summary["per_class"][cid]
        ys = [cs["variants"].get(n, {}).get("cir_residual_full_db_mean")
              for n in VARIANT_ORDER]
        ax.plot(x, ys, marker="o", label=cid)
    ax.set_xticks(x)
    ax.set_xticklabels(VARIANT_ORDER, rotation=45, ha="right", fontsize=7)
    ax.set_ylabel("CIR residual / clean [dB] (support-disjoint N=54)")
    ax.set_title("A-R03 truth-assisted substitution budget")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "variant_budget.png"), dpi=120)
    plt.close(fig)
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    names = [c for c in classes if c in MANDATED_CLASSES]
    ax[0].bar([c for c in names],
              [summary["per_class"][c]["cfo_err_hz_mean"] for c in names])
    ax[0].set_title("model CFO error (Hz)")
    ax[0].tick_params(axis="x", rotation=45, labelsize=7)
    ax[1].bar([c for c in names],
              [summary["per_class"][c]["delay_err_abs_mean"] or 0.0
               for c in names])
    ax[1].set_title("mean |delta tau| (samples)")
    ax[1].tick_params(axis="x", rotation=45, labelsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "parameter_errors.png"), dpi=120)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 4))
    w = 0.35
    idx = np.arange(len(classes))
    ax.bar(idx - w / 2, [summary["per_class"][c]["variants"].get(
        "S0_model", {}).get("cir_residual_full_db_mean") or 0.0
        for c in classes], w, label="S0 model (online)")
    ax.bar(idx + w / 2, [summary["per_class"][c]["variants"].get(
        "S6_full_truth", {}).get("cir_residual_full_db_mean") or 0.0
        for c in classes], w, label="S6 full truth (floor)")
    ax.set_xticks(idx)
    ax.set_xticklabels(classes, rotation=45, ha="right", fontsize=7)
    ax.set_ylabel("CIR residual / clean [dB]")
    ax.set_title("online model vs reachable truth floor")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "model_vs_floor.png"), dpi=120)
    plt.close(fig)


def self_test(verbose=True) -> int:
    fails = RB.self_test(verbose=verbose)
    if verbose:
        print("error-budget app self-test: %d failure(s)" % fails)
    return fails


def build_parser():
    ap = argparse.ArgumentParser(
        description="A-R03 SIC residual error budget (offline diagnostics)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--build-dir")
    ap.add_argument("--output")
    ap.add_argument("--classes", nargs="+", default=DEFAULT_CLASSES)
    ap.add_argument("--windows", type=int, default=8)
    ap.add_argument("--seed-set", choices=("calibration", "acceptance"),
                    default="calibration")
    ap.add_argument("--i-know-this-is-calibration", action="store_true")
    ap.add_argument("--plots", action="store_true")
    ap.add_argument("--archive-windows", type=int, default=4)
    ap.add_argument("--cross-check-run", default=None,
                    help="reference run dir; archived windows are compared "
                         "bit-for-bit against it (fail closed on a mismatch)")
    ap.add_argument("--stage", default=STAGE)
    ap.add_argument("--self-test", action="store_true")
    return ap


def main(argv=None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(True)
    if not args.build_dir or not args.output:
        ap.error("--build-dir and --output are required for a run")
    if args.seed_set == "calibration" and not args.i_know_this_is_calibration:
        raise SystemExit("--seed-set calibration requires "
                         "--i-know-this-is-calibration")
    base = os.path.basename(os.path.abspath(args.output))
    if args.stage and args.stage not in base:
        args.output = os.path.abspath(args.output) + "_" + args.stage
        print("using %s" % args.output, flush=True)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
