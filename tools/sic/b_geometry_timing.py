#!/usr/bin/env python3
"""Paired B-geometry SIC timing on the payload path.

Calls ``uwb_sim_sic_payload.py`` in the named worktree so the measurement
uses that tree's library. Generator time, post time and ``run_wall_us`` stay
outside ``core_total_us``. Cache hit/miss counters already ride on the SIC
``out`` message; the stock recorder does not keep them, so this process copies
them onto ``record["sic"]["diag"]`` before the jsonl line is written. It does
not edit the simulator source and does not load two ``libgnuradio-uwb``
copies in one process.

One invocation measures one worktree. Run baseline and candidate serially.
``compare`` then checks that the two sides saw the same ``input_sha256`` list
and reports core-time deltas. A non-commit control that actually commits, or
a missing diag field, exits non-zero.
"""
from __future__ import annotations

import importlib.util
import json
import os
import statistics
import sys

CASES = (
    ("center_overlap", [
        "--phase", "b1", "--position", "center", "--snr-db", "30",
        "--isr-db", "10", "--cfo-hz", "0", "--comm-channel", "single",
        "--interferer-paths", "single", "--interferer", "on"]),
    ("early_boundary", [
        "--phase", "b3", "--position", "early", "--snr-db", "-3.5",
        "--isr-db", "10", "--cfo-hz", "0", "--comm-channel", "single",
        "--interferer-paths", "single", "--interferer", "on"]),
    ("late_boundary", [
        "--phase", "b3", "--position", "late", "--snr-db", "-3.5",
        "--isr-db", "10", "--cfo-hz", "0", "--comm-channel", "single",
        "--interferer-paths", "single", "--interferer", "on"]),
    ("multipath_m50", [
        "--phase", "b3", "--position", "center", "--snr-db", "30",
        "--isr-db", "10", "--cfo-hz", "-50000", "--comm-channel", "single",
        "--interferer-paths", "four", "--interferer", "on"]),
    ("multipath_p50", [
        "--phase", "b3", "--position", "center", "--snr-db", "30",
        "--isr-db", "10", "--cfo-hz", "50000", "--comm-channel", "single",
        "--interferer-paths", "four", "--interferer", "on"]),
    ("bypass_no_overlap", [
        "--phase", "b4", "--position", "no_overlap", "--snr-db", "30",
        "--isr-db", "10", "--cfo-hz", "0", "--comm-channel", "single",
        "--interferer-paths", "single", "--interferer", "on"]),
)

DIAG_KEYS = (
    "sic_refine_self_energy_hits",
    "sic_refine_self_energy_misses",
    "sic_refine_cache_hits",
    "sic_refine_cache_misses",
    "sic_refine_k",
    "sic_refine_m",
    "sic_refine_candidates",
    "sic_fractional_refine_us",
    "sic_estimate_us",
    "sic_apply_us",
    "sic_samples_per_symbol",
)


def _pct(xs, p):
    if not xs:
        return None
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def _dist(xs):
    if not xs:
        return None
    return {
        "n": len(xs),
        "mean_us": statistics.fmean(xs),
        "p50_us": _pct(xs, 0.50),
        "p95_us": _pct(xs, 0.95),
        "p99_us": _pct(xs, 0.99),
        "min_us": min(xs),
        "max_us": max(xs),
    }


def _load_payload(repo):
    path = os.path.join(repo, "gr-uwb", "apps", "uwb_sim_sic_payload.py")
    if not os.path.isfile(path):
        raise SystemExit("missing payload sim: %s" % path)
    # A unique name so a second invocation in this process cannot happen to
    # bind the other worktree's already-imported module.
    spec = importlib.util.spec_from_file_location(
        "uwb_sim_sic_payload_geom", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    orig = mod.run_window

    def wrapped(*args, **kwargs):
        rec = orig(*args, **kwargs)
        rt = args[0]
        if rt.sic is None:
            return rec
        pid = rec["packet_ids"][2]
        raw = rt.sink_out.get(pid) if rt.sink_out is not None else None
        diag = {}
        if isinstance(raw, dict):
            for key in DIAG_KEYS:
                if key in raw and raw[key] is not None:
                    diag[key] = int(raw[key])
        rec.setdefault("sic", {})["diag"] = diag
        return rec

    mod.run_window = wrapped
    return mod


def _summarize(case_dir):
    rows = []
    with open(os.path.join(case_dir, "windows.jsonl")) as fh:
        for line in fh:
            rows.append(json.loads(line))
    with open(os.path.join(case_dir, "summary.json")) as fh:
        summary = json.load(fh)
    with open(os.path.join(case_dir, "plan.json")) as fh:
        plan = json.load(fh)
    with open(os.path.join(case_dir, "run_manifest.json")) as fh:
        manifest = json.load(fh)
    cores = []
    gens = []
    posts = []
    demod_sic = []
    hits = misses = chits = cmisses = 0
    counters_seen = 0
    missing_diag = 0
    states = {}
    for row in rows:
        sic = row.get("sic") or {}
        st = sic.get("state")
        states[st] = states.get(st, 0) + 1
        if sic.get("core_total_us") is not None:
            cores.append(int(sic["core_total_us"]))
        timing = row.get("timing") or {}
        gens.append(int(timing.get("gen_us") or 0))
        posts.append(int(timing.get("post_us") or 0))
        demod = ((row.get("demod") or {}).get("sic") or {})
        if demod.get("stage_total_us") is not None:
            demod_sic.append(int(demod["stage_total_us"]))
        diag = sic.get("diag")
        if not isinstance(diag, dict):
            missing_diag += 1
            continue
        if "sic_refine_self_energy_hits" not in diag:
            continue
        hits += int(diag["sic_refine_self_energy_hits"])
        misses += int(diag["sic_refine_self_energy_misses"])
        chits += int(diag.get("sic_refine_cache_hits") or 0)
        cmisses += int(diag.get("sic_refine_cache_misses") or 0)
        counters_seen += 1
    cfg = plan.get("case_config") or {}
    gen = (manifest.get("generator") or {})
    prov = manifest.get("provenance") or {}
    git = prov.get("git") or {}
    cold = cores[:1]
    steady = cores[1:]
    extract = {
        "schema": "sic-perf-b-geometry/1",
        "windows": len(rows),
        "states": states,
        "committed": sum(1 for r in rows if (r.get("sic") or {}).get("committed")),
        "input_sha256": [r.get("input_sha256") for r in rows],
        "psdu_sic_match": [
            ((r.get("demod") or {}).get("sic") or {}).get("psdu_match")
            for r in rows],
        "psdu_raw_match": [
            ((r.get("demod") or {}).get("raw") or {}).get("psdu_match")
            for r in rows],
        "k_paths": cfg.get("sic_k_paths"),
        "fit_reps": cfg.get("sic_fit_reps"),
        "window_n": gen.get("n"),
        "refine_m_steady": [
            int(((r.get("sic") or {}).get("diag") or {}).get("sic_refine_m") or 0)
            for r in rows[1:]],
        "core_cold": _dist(cold),
        "core_steady": _dist(steady),
        "core_all": _dist(cores),
        "generator_us": _dist(gens),
        "post_us": _dist(posts),
        "demod_sic_stage_us": _dist(demod_sic),
        "run_wall_us": (summary.get("runtime") or {}).get("run_wall_us"),
        "cache_counters_published": counters_seen == len(rows) and len(rows) > 0,
        "self_energy_hits": hits if counters_seen == len(rows) else None,
        "self_energy_misses": misses if counters_seen == len(rows) else None,
        "column_cache_hits": chits if counters_seen == len(rows) else None,
        "column_cache_misses": cmisses if counters_seen == len(rows) else None,
        "missing_diag": missing_diag,
        "library_sha256": prov.get("loaded_library_sha256"),
        "git_head": git.get("head"),
        "git_dirty": git.get("dirty"),
        "case_config_hash": plan.get("case_config_hash"),
    }
    with open(os.path.join(case_dir, "timing_extract.json"), "w") as fh:
        json.dump(extract, fh, indent=2, sort_keys=True)
        fh.write("\n")
    return extract


def _run_case(mod, repo, build_dir, out_root, name, extra, windows,
              require_counters):
    dest = os.path.join(out_root, name)
    if os.path.exists(dest):
        raise SystemExit("refuse overwrite: %s" % dest)
    argv = [
        "--build-dir", build_dir, "--output", dest,
        "--seed-set", "acceptance", "--windows", str(windows),
        "--archive-windows", "4", "--progress", "8", "--sic", "estimate",
    ] + extra
    print("== %s %s" % (os.path.basename(repo.rstrip("/")), name), flush=True)
    rc = mod.main(argv)
    print("== %s %s rc=%s" % (os.path.basename(repo.rstrip("/")), name, rc),
          flush=True)
    if rc != 0:
        raise SystemExit("case %s failed rc=%s" % (name, rc))
    extract = _summarize(dest)
    if extract["missing_diag"]:
        raise SystemExit("%s missing diag on %d windows" % (
            name, extract["missing_diag"]))
    if require_counters and not extract["cache_counters_published"]:
        raise SystemExit("%s did not publish self-energy cache counters" % name)
    if name == "bypass_no_overlap" and extract["committed"] != 0:
        raise SystemExit("bypass control committed %d windows" %
                         extract["committed"])
    if name != "bypass_no_overlap" and extract["committed"] == 0:
        raise SystemExit("%s committed nothing" % name)
    return extract


def run_library(argv):
    import argparse
    ap = argparse.ArgumentParser(description="one-library B geometry timing")
    ap.add_argument("--repo", required=True)
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--windows", type=int, default=32)
    ap.add_argument("--only", default=None)
    ap.add_argument("--require-cache-counters", action="store_true",
                    help="fail if the loaded library does not publish "
                         "sic_refine_self_energy_hits (candidate builds do)")
    args = ap.parse_args(argv)
    repo = os.path.realpath(args.repo)
    build = os.path.realpath(args.build_dir)
    out = os.path.realpath(args.output)
    if not os.path.isdir(os.path.join(build, "lib")):
        raise SystemExit("build dir has no lib/: %s" % build)
    os.makedirs(out, exist_ok=True)
    os.chdir(repo)
    mod = _load_payload(repo)
    selected = [c for c in CASES if args.only is None or c[0] == args.only]
    if not selected:
        raise SystemExit("unknown --only %s" % args.only)
    extracts = {}
    for name, extra in selected:
        extracts[name] = {
            "committed": _run_case(mod, repo, build, out, name, extra,
                                   args.windows,
                                   args.require_cache_counters)["committed"],
            "core_steady_mean_us": _summarize(os.path.join(out, name))[
                "core_steady"]["mean_us"],
        }
        # _run_case already wrote the extract; the second summarize is a
        # reread so the printed mean matches the artifact.
    print(json.dumps({"output": out, "cases": extracts}, sort_keys=True),
          flush=True)
    return 0


def compare(argv):
    import argparse
    ap = argparse.ArgumentParser(description="pair two geometry result roots")
    ap.add_argument("--base", required=True)
    ap.add_argument("--cand", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args(argv)
    if os.path.exists(args.output):
        raise SystemExit("refuse overwrite: %s" % args.output)
    report = {"schema": "sic-perf-b-geometry-pair/1", "cases": {}}
    failed = False
    for name, _extra in CASES:
        pair = {}
        for side, root in (("base", args.base), ("cand", args.cand)):
            path = os.path.join(root, name, "timing_extract.json")
            with open(path) as fh:
                pair[side] = json.load(fh)
        base, cand = pair["base"], pair["cand"]
        same_input = base["input_sha256"] == cand["input_sha256"]
        same_psdu = (base["psdu_sic_match"] == cand["psdu_sic_match"]
                     and base["psdu_raw_match"] == cand["psdu_raw_match"]
                     and base["states"] == cand["states"])
        bmean = base["core_steady"]["mean_us"]
        cmean = cand["core_steady"]["mean_us"]
        row = {
            "same_input_sha256": same_input,
            "same_demod_state": same_psdu,
            "windows": base["windows"],
            "k_paths": cand["k_paths"],
            "fit_reps": cand["fit_reps"],
            "window_n": cand["window_n"],
            "refine_m_set": sorted(set(cand["refine_m_steady"])),
            "base_core_cold_us": (base["core_cold"] or {}).get("mean_us"),
            "cand_core_cold_us": (cand["core_cold"] or {}).get("mean_us"),
            "base_core_steady_mean_us": bmean,
            "cand_core_steady_mean_us": cmean,
            "delta_mean_us": cmean - bmean,
            "speedup": (bmean / cmean) if cmean else None,
            "base_self_energy_hits": base["self_energy_hits"],
            "base_self_energy_misses": base["self_energy_misses"],
            "cand_self_energy_hits": cand["self_energy_hits"],
            "cand_self_energy_misses": cand["self_energy_misses"],
            "base_column_cache_hits": base["column_cache_hits"],
            "cand_column_cache_hits": cand["column_cache_hits"],
            "base_run_wall_us": base["run_wall_us"],
            "cand_run_wall_us": cand["run_wall_us"],
            "base_generator_mean_us": base["generator_us"]["mean_us"],
            "cand_generator_mean_us": cand["generator_us"]["mean_us"],
            "base_demod_sic_mean_us": (base["demod_sic_stage_us"] or {}).get(
                "mean_us"),
            "cand_demod_sic_mean_us": (cand["demod_sic_stage_us"] or {}).get(
                "mean_us"),
            "base_lib": base["library_sha256"],
            "cand_lib": cand["library_sha256"],
            "base_git": base["git_head"],
            "cand_git": cand["git_head"],
            "states": cand["states"],
        }
        if not same_input or not same_psdu:
            failed = True
        if name == "bypass_no_overlap" and (
                base["committed"] or cand["committed"]):
            failed = True
        report["cases"][name] = row
        print("%s steady base %.1f cand %.1f us  same_input=%s same_demod=%s"
              % (name, bmean, cmean, same_input, same_psdu), flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as fh:
        json.dump(report, fh, indent=2, sort_keys=True)
        fh.write("\n")
    return 1 if failed else 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ("run", "compare"):
        raise SystemExit("usage: b_geometry_timing.py run|compare ...")
    if argv[0] == "compare":
        return compare(argv[1:])
    return run_library(argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
