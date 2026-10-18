#!/usr/bin/env python3
"""S2-A rectification analysis: extract the report tables from the new runs.

Reads only the rectification artifacts (schema 2) and prints the tables used in
``docs/sic/S2A_整改开发报告_2026-10-18.md``.  Not part of the shipped pipeline;
kept in the worktree so every number in the report can be re-derived.
"""

import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RES = os.path.join(ROOT, "gr-uwb", "build-sic-accum", "results", "s2a")


def load_budget(name):
    p = os.path.join(RES, name, "error_budget_summary.json")
    if not os.path.exists(p):
        return None
    with open(p) as fh:
        return json.load(fh)


def load(name, f="summary.json"):
    p = os.path.join(RES, name, f)
    if not os.path.exists(p):
        return None
    with open(p) as fh:
        return json.load(fh)


def jsonl(name):
    p = os.path.join(RES, name, "windows.jsonl")
    out = []
    with open(p) as fh:
        for line in fh:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def fmt(v, n=2):
    if v is None:
        return "  na"
    if isinstance(v, float):
        return ("%%.%df" % n) % v
    return str(v)


def main():
    acc = load("acceptance_rectification")
    old = load("acceptance")
    if acc is None:
        print("no acceptance_rectification run yet")
        return 1
    print("== A-R01: support-disjoint vs legacy independent label ==")
    print("%-26s %-26s %-8s %s" % ("cell", "cell_set[first,last,avail]",
                                   "n_list", "coverage cir/sub cir/truth"))
    for cid in acc["cells"]:
        cs = acc["per_cell"][cid]
        st = cs["support"]["cell_set"]
        sup = cs["support"]
        print("%-26s [%d,%d] n=%d %-20s %s %s"
              % (cid, st["first"], st["last"], st["available"],
                 str(st["n_list"]), fmt(sup["cir_overlap_sub_ratio_mean"], 3),
                 fmt(sup["cir_overlap_truth_ratio_mean"], 3)))
    print()
    print("%-26s %8s %8s %8s | %8s %8s %8s | %s"
          % ("cell", "C1_sd", "C8_sd", "C54_sd", "C1_leg", "C8_leg", "C56_leg",
             "dC(8) sd-leg"))
    for cid in acc["cells"]:
        cs = acc["per_cell"][cid]["accum"]
        o = old["per_cell"][cid]["accum"] if old else {}
        def g(d, k, roi="full"):
            e = d.get(k, {}).get(roi, {}).get("mean")
            return e
        print("%-26s %8s %8s %8s | %8s %8s %8s | %s"
              % (cid, fmt(g(cs, "indep/1")), fmt(g(cs, "indep/8")),
                 fmt(g(cs, "indep/54")), fmt(g(cs, "indep/1")),
                 fmt(g(cs, "indep/8")), fmt(g(o, "indep/56")),
                 fmt((g(cs, "indep/8") or 0) - (g(o, "indep/8") or 0))))
    print()
    print("== A-R02: coherence / energy decomposition (support-disjoint N=54) ==")
    print("%-26s %8s %10s %10s %10s %10s"
          % ("cell", "eta", "e_coherent", "e_centered", "e_total", "cons_rel"))
    for cid in acc["cells"]:
        c = acc["per_cell"][cid]["coherence"]["coherence/sic/all"]
        print("%-26s %8s %10s %10s %10s %10s"
              % (cid, fmt(c["eta_mean"], 4), "%.4g" % c["e_coherent_mean"],
                 "%.4g" % c["e_centered_mean"], "%.4g" % c["e_total_mean"],
                 "%.2g" % (c["energy_conservation_rel_max"] or 0)))
    print()
    print("== A-R01: legacy diagnostic (must not be used as the main result) ==")
    for cid in acc["cells"][:3]:
        cs = acc["per_cell"][cid]
        leg = cs["sets"]["legacy_nominal_index"]
        print("  %-24s legacy strict_holdout=%s first=%s last=%s"
              % (cid, leg["strict_holdout"], leg["first"], leg["last"]))
    print()
    b = load_budget("budget_rectification")
    if b is None:
        print("no budget run yet")
        return 0
    print("== A-R03: parameter error table (mean over windows) ==")
    print("%-26s %10s %10s %8s %9s %9s %8s %6s/%s"
          % ("class", "cfo_err", "cfo_std", "|dtau|", "gain_mag", "gain_ph",
             "np_mod", "n_truth", "unpaired_truth"))
    for cid in b["classes"]:
        cs = b["per_class"][cid]
        print("%-26s %10s %10s %8s %9s %9s %8s %6s/%s"
              % (cid, fmt(cs["cfo_err_hz_mean"], 1), fmt(cs["cfo_err_hz_std"], 1),
                 fmt(cs["delay_err_abs_mean"], 3), fmt(cs["gain_mag_err_db_mean"], 3),
                 fmt(cs["gain_phase_err_deg_mean"], 2),
                 fmt(cs["n_paths_model_mean"], 2), fmt(cs["n_paths_truth"], 0),
                 fmt(cs["unpaired_truth_mean"], 2)))
    print()
    print("== A-R03: variant residual (CIR residual / clean, dB, N=54) ==")
    V = ["S0_model", "S1_cfo_truth", "S2_tau_truth", "S3_gain_truth",
         "S4_pathset_truth", "S5_tau_cfo_truth", "S6_full_truth",
         "C1_cfo_truth_gainrefit", "C2_tau_cfo_truth_gainrefit"]
    print("%-26s %s" % ("class", " ".join("%8s" % v[:8] for v in V)))
    for cid in b["classes"]:
        cs = b["per_class"][cid]
        print("%-26s %s" % (cid, " ".join(
            "%8s" % fmt((cs["variants"].get(v) or {}).get(
                "cir_residual_full_db_mean")) for v in V)))
    print()
    print("== A-R03: training-input controls (IQ, dB relative to the input) ==")
    print("%-26s %10s %10s %10s %10s %10s"
          % ("class", "i_only", "i+w", "s+i", "impl(s0)", "recon_nmse"))
    for cid in b["classes"]:
        cs = b["per_class"][cid]
        tc = cs["training_controls"]
        print("%-26s %10s %10s %10s %10s %10s"
              % (cid, fmt(tc["i_only"]["cancel_db_mean"]),
                 fmt(tc["i_plus_w"]["cancel_db_mean"]),
                 fmt(tc["s_plus_i"]["cancel_db_mean"]),
                 fmt(cs["impl_mismatch_db_mean"]),
                 "%.2g" % (cs["recon_validation_nmse_max"] or 0)))
    print()
    print("== A-R03: budget components (share of the total residual energy) ==")
    print("%-26s %12s %12s %12s %12s"
          % ("class", "e_impl_frac", "e_param_frac", "e_cross_frac", "ident_rel"))
    for cid in b["classes"]:
        cs = b["per_class"][cid]["budget"]
        print("%-26s %12s %12s %12s %12s"
              % (cid, fmt(cs["ratio_impl_over_total_mean"], 3),
                 fmt(cs["ratio_param_over_total_mean"], 3),
                 fmt(cs["ratio_cross_over_total_mean"], 3),
                 "%.1g" % cs["identity_rel_max"]))
    print()
    print("== A-R03: deterministic bias vs cross-window fluctuation ==")
    print("%-26s %14s %14s %10s"
          % ("class", "mean_vec_energy", "fluct_energy", "ratio"))
    for cid in b["classes"]:
        bf = b["per_class"][cid]["bias_vs_fluctuation"]
        print("%-26s %14.5g %14.5g %10.3g"
              % (cid, bf["mean_vector_energy"],
                 bf["cross_window_fluctuation_energy"],
                 bf["fluctuation_over_mean"] or 0))
    print()
    print("== A-R01: bypass/receiver recap (rectification acceptance) ==")
    print("acceptance totals:", acc["totals"])
    byp = load("bypass_rectification")
    if byp:
        print("bypass totals:", byp["totals"])
        print("bypass archive:", {k: v for k, v in byp["archive"].items()
                                  if k not in ("paths", "fixed_by_index")})
    print()
    print("== worst-window evidence examples ==")
    for cid in acc["cells"][:4]:
        w = acc["per_cell"][cid]["worst_windows"]
        print("  %-26s lowest_c_cir=%s largest_target_err=%s"
              % (cid, (w.get("lowest_c_cir") or {}).get("c_cir_db"),
                 (w.get("largest_target_error") or {}).get("amp_err_db")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
