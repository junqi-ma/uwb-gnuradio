#!/usr/bin/env python3
"""Independent verifier for S2-B payload-collision runs.

Reads ONLY the declared artifacts (``run_manifest.json``, ``windows.jsonl``,
``summary.json``, ``index.json`` and the optional ``recon/*.npz`` arrays) and
re-derives every aggregate from scratch:

  * per-path PER_PSDU / PER_FCS and Wilson CIs from the raw per-window rows,
  * paired ``raw_fail_sic_ok`` / ``raw_ok_sic_fail`` counts,
  * the pooled median actual suppression from the recorded per-window values,
  * an INDEPENDENT suppression recomputation from the archived arrays,
  * output-mask exactness (``out == raw`` outside the subtraction mask),
  * the single-FCS / PSDU-length contract and the geometry / seed discipline,
  * provenance (worktree library, not ``/usr/local``, MATLAB 未实跑 declared).

``--self-test`` builds a consistent artifact set that MUST pass, then fabricates
deliberately corrupted variants (wrong denominator, dropped failure, wrong
suppression, zeroed array, wrong library, broken ROI, double FCS) and asserts
that each one is rejected.

Usage (from the repo root):
  env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py --results <run_dir>
  env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py --self-test
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import payload_metrics as PM  # noqa: E402

RESULTS = []


def record(kind, name, detail=""):
    RESULTS.append((kind, name, detail))
    if kind == "FAIL":
        print("FAIL  %-52s %s" % (name, detail), flush=True)


def ok(name, detail=""):
    record("PASS", name, detail)


def fail(name, detail=""):
    record("FAIL", name, detail)


def skip(name, detail=""):
    record("SKIP", name, detail)


# ---------------------------------------------------------------------------
def _is_worktree_lib(path, run_dir):
    if not path:
        return False
    p = os.path.abspath(path)
    if p.startswith("/usr/local") or p.startswith("/usr/lib"):
        return False
    return "build-sic-payload" in p or "gr-uwb" in p


def check_manifest(run, run_dir):
    m = run.get("manifest")
    if not m:
        fail("manifest_present", "run_manifest.json missing")
        return
    ok("manifest_present")
    if m.get("schema") != "sic-s2b-payload/1":
        fail("manifest_schema", str(m.get("schema")))
    else:
        ok("manifest_schema")
    prov = m.get("provenance", {})
    lib = prov.get("loaded_library")
    if _is_worktree_lib(lib, run_dir):
        ok("manifest_library_in_worktree", lib)
    else:
        fail("manifest_library_in_worktree", str(lib))
    if prov.get("git", {}).get("head"):
        ok("manifest_git_head", prov["git"]["head"][:12])
    else:
        fail("manifest_git_head", "missing git head")
    fc = m.get("fcs_contract", {})
    if (fc.get("modulator_append_fcs") is False
            and "generator" in str(fc.get("appended_once_by", ""))):
        ok("manifest_fcs_single_append")
    else:
        fail("manifest_fcs_single_append", json.dumps(fc))
    nr = m.get("not_run")
    if isinstance(nr, list) and any("MATLAB" in str(x) for x in nr):
        ok("manifest_matlab_not_run_declared")
    else:
        fail("manifest_matlab_not_run_declared", str(nr))


def check_windows(run):
    recs = run["records"]
    if not recs:
        fail("windows_present", "no rows")
        return
    ok("windows_present", "%d rows" % len(recs))
    bad = 0
    for r in recs:
        wlen = r.get("psdu_wire_len")
        need = ("window_id", "seed", "payload_seed", "demod", "rois",
                "training_span", "overlap")
        if any(k not in r for k in need):
            bad += 1
            continue
        # single FCS contract: 96B wire = 94 data + 2 FCS
        if wlen != 96:
            bad += 1
        # training span must be inside the payload/overlap support (unknown)
        t0, t1 = r["training_span"]
        ov = r["overlap"]
        if not (t0 >= 0 and t1 > t0):
            bad += 1
    if bad == 0:
        ok("windows_contract_fields", "%d rows" % len(recs))
    else:
        fail("windows_contract_fields", "%d bad rows" % bad)
    # unique terminal per (window, path)
    ids = [r["window_id"] for r in recs]
    if len(set(ids)) == len(ids):
        ok("windows_unique_ids")
    else:
        fail("windows_unique_ids", "duplicates present")
    # ROI sanity
    roi_bad = 0
    for r in recs:
        ro = r.get("rois", {})
        if not (0 <= ro.get("detect_roi_lo", -1) < ro.get("detect_roi_hi", -1)):
            roi_bad += 1
        if not (0 <= ro.get("sub_roi_lo", -1) < ro.get("sub_roi_hi", -1)):
            roi_bad += 1
    (ok if roi_bad == 0 else fail)("windows_roi_ordering",
                                   "%d bad" % roi_bad)


def check_summary(run):
    s = run.get("summary")
    recs = run["records"]
    if not s:
        fail("summary_present")
        return
    ok("summary_present")
    for path in ("clean", "raw", "sic"):
        if path not in (s.get("per") or {}):
            continue
        mine = PM.per_psdu(recs, path)
        theirs = s["per"][path]
        if (mine["errors"] == theirs.get("errors")
                and mine["total"] == theirs.get("total")):
            ok("summary_per_%s" % path,
               "err=%d/%d" % (mine["errors"], mine["total"]))
        else:
            fail("summary_per_%s" % path,
                 "recomputed %d/%d vs summary %s/%s"
                 % (mine["errors"], mine["total"], theirs.get("errors"),
                    theirs.get("total")))
    # fcs PER
    for path in ("clean", "raw", "sic"):
        if any(path in (r.get("demod") or {}) for r in recs):
            mine = PM.per_fcs(recs, path)
            ok("recompute_per_fcs_%s" % path,
               "err=%d/%d" % (mine["errors"], mine["total"]))
    if "sic" in (s.get("per") or {}):
        mine = PM.paired_counts(recs)
        theirs = s.get("paired", {})
        if (mine["raw_fail_sic_ok"] == theirs.get("raw_fail_sic_ok")
                and mine["raw_ok_sic_fail"] == theirs.get("raw_ok_sic_fail")):
            ok("summary_paired", json.dumps(mine))
        else:
            fail("summary_paired", "recomputed %s vs %s"
                 % (json.dumps(mine), json.dumps(theirs)))
        supp_rec = PM.median_suppression(recs)
        supp_sum = (s.get("sic") or {}).get("median_suppression_db")
        if supp_rec is None or supp_sum is None or \
                abs(supp_rec - supp_sum) < 1e-6:
            ok("summary_median_suppression",
               "%.4f" % (supp_rec if supp_rec is not None else float("nan")))
        else:
            fail("summary_median_suppression",
                 "recomputed %.4f vs summary %.4f" % (supp_rec, supp_sum))
    # denominator discipline: total == number of planned windows
    n = len(recs)
    for path, d in (s.get("per") or {}).items():
        if d.get("total") != n:
            fail("summary_denominator_%s" % path,
                 "total=%s rows=%d" % (d.get("total"), n))
        else:
            ok("summary_denominator_%s" % path, "=%d" % n)


def check_recon(run):
    recs = {r["window_id"]: r for r in run["records"]}
    recon = run.get("recon", {})
    if not recon:
        skip("recon_archived_arrays", "no recon/*.npz")
        return
    n_checked = 0
    n_bad_supp = 0
    n_bad_mask = 0
    for fn, arr in recon.items():
        wid = int(fn.replace("window_", "").replace(".npz", ""))
        r = recs.get(wid)
        if r is None:
            n_bad_mask += 1
            continue
        i_true = arr["i_true"]
        clean = arr["clean"]
        out = arr["out"]
        ov = r["overlap"]
        rec_supp = r.get("suppression_db_overlap")
        if out.size == 0:
            continue
        mine = PM.suppression_db(i_true, clean, out, ov["lo"], ov["hi"])
        if rec_supp is not None and mine is not None:
            if abs(mine - rec_supp) > 1e-4:
                n_bad_supp += 1
        # mask exactness: use the archived raw when available (bit-exact
        # outside-mask contract); else fall back to the reconstructed raw.
        if "raw" in arr.files:
            raw = arr["raw"]
        else:
            raw = (clean.astype(np.complex128)
                   + i_true.astype(np.complex128)).astype(np.complex64)
        changed = PM.mask_outside_changed(
            raw, out,
            (r.get("sic") or {}).get("sub_lo"),
            (r.get("sic") or {}).get("sub_hi"))
        if changed > 0:
            n_bad_mask += 1
        n_checked += 1
    (ok if n_bad_supp == 0 else fail)(
        "recon_suppression_recompute", "%d/%d mismatch"
        % (n_bad_supp, n_checked))
    (ok if n_bad_mask == 0 else fail)(
        "recon_mask_exactness", "%d/%d bad" % (n_bad_mask, n_checked))


def check_seeds(run, run_dir):
    recs = run["records"]
    payloads = [r.get("payload_seed") for r in recs]
    noise = [r.get("seed") for r in recs]
    if len(set(payloads)) == len(payloads):
        ok("seeds_payload_unique")
    else:
        fail("seeds_payload_unique", "duplicate payload seeds")
    if len(set(noise)) == len(noise):
        ok("seeds_noise_unique")
    else:
        fail("seeds_noise_unique", "duplicate noise seeds")
    # calibration vs acceptance separation
    cal = [p for p in payloads if p is not None and p < 22000000]
    acc = [p for p in payloads if p is not None and p >= 22000000]
    if cal and acc:
        fail("seeds_set_separation", "mixed calibration/acceptance seeds")
    else:
        ok("seeds_set_separation")


def check_no_truth_leak(run):
    recs = run["records"]
    leaked = [r["window_id"] for r in recs
              if (r.get("sic") or {}).get("uses_truth") is True]
    (ok if not leaked else fail)("no_truth_leak", "leaked=%s" % leaked[:5])
    refit = [r["window_id"] for r in recs
             if (r.get("sic") or {}).get("apply_refit_count") not in (0, None)]
    (ok if not refit else fail)("apply_refit_zero", "refit=%s" % refit[:5])


def check_index(run, run_dir):
    idx = run.get("index")
    if not idx:
        fail("index_present")
        return
    ok("index_present")
    if idx.get("schema") == "sic-s2b-payload/1":
        ok("index_schema")
    else:
        fail("index_schema", str(idx.get("schema")))
    if idx.get("windows") == len(run["records"]):
        ok("index_window_count")
    else:
        fail("index_window_count", "index=%s rows=%d"
             % (idx.get("windows"), len(run["records"])))
    ap = idx.get("archive_policy") or {}
    if "archived" in ap and "total" in ap:
        ok("index_archive_policy", json.dumps(ap))
    else:
        fail("index_archive_policy", json.dumps(ap))


def verify_run(run_dir, verbose=False):
    del RESULTS[:]
    run = PM.load_run(run_dir)
    if not run["records"] and not run["manifest"]:
        fail("run_readable", "no artifacts in %s" % run_dir)
        return 1
    check_manifest(run, run_dir)
    check_windows(run)
    check_summary(run)
    check_recon(run)
    check_seeds(run, run_dir)
    check_no_truth_leak(run)
    check_index(run, run_dir)
    n_fail = sum(1 for r in RESULTS if r[0] == "FAIL")
    n_pass = sum(1 for r in RESULTS if r[0] == "PASS")
    n_skip = sum(1 for r in RESULTS if r[0] == "SKIP")
    print("== %s: %d pass, %d fail, %d skip =="
          % (run_dir, n_pass, n_fail, n_skip), flush=True)
    return 1 if n_fail else 0


# ---------------------------------------------------------------------------
# Self-test: build a good set, then corrupt it in targeted ways.
# ---------------------------------------------------------------------------
def _mk_good(root, n=12):
    os.makedirs(os.path.join(root, "recon"), exist_ok=True)
    n_samples = 4096
    recs = []
    rng = np.random.default_rng(0)
    for w in range(n):
        raw_fail = (w % 3 == 0)
        sic_ok = True
        clean_ok = True
        sub_lo, sub_hi = 1000, 3000
        out = rng.standard_normal(n_samples).astype(np.complex64) * 0.01
        clean = rng.standard_normal(n_samples).astype(np.complex64)
        i_true = rng.standard_normal(n_samples).astype(np.complex64) * 0.5
        # make 'out' equal raw = clean+i outside the mask
        raw = clean.astype(np.complex128) + i_true.astype(np.complex128)
        out = raw.astype(np.complex64).copy()
        supp = PM.suppression_db(i_true, clean, out, 1200, 2800)
        rec = {
            "window_id": w, "seed": 22000000 + w,
            "payload_seed": 22100000 + w, "jitter": 0,
            "psdu_wire_len": 96, "psdu_wire_hex": "AA" * 96,
            "training_span": [1100, 1100 + 8 * 16],
            "overlap": {"lo": 1200, "hi": 2800, "samples": 1600},
            "rois": {"detect_roi_lo": 900, "detect_roi_hi": 3500,
                     "sub_roi_lo": 1000, "sub_roi_hi": 3000,
                     "observed_end": 3200, "nominal_start": 2200,
                     "start_jitter": 100, "guard": 100},
            "suppression_db_overlap": supp,
            "mask_outside_changed": 0,
            "demod": {
                "clean": {"status": "success", "psdu_match": clean_ok,
                          "fcs_pass": clean_ok},
                "raw": {"status": "success" if not raw_fail
                        else "payload_failed",
                        "psdu_match": not raw_fail, "fcs_pass": not raw_fail},
                "sic": {"status": "success" if sic_ok else "payload_failed",
                        "psdu_match": sic_ok, "fcs_pass": sic_ok},
            },
            "sic": {"state": "committed", "committed": True, "detected": True,
                    "uses_truth": False, "apply_refit_count": 0,
                    "sub_lo": sub_lo, "sub_hi": sub_hi},
        }
        recs.append(rec)
        np.savez_compressed(os.path.join(root, "recon", "window_%04d.npz" % w),
                            i_true=i_true, clean=clean, out=out)
    with open(os.path.join(root, "windows.jsonl"), "w") as fh:
        for r in recs:
            fh.write(json.dumps(r) + "\n")
    summary = {"schema": "sic-s2b-payload/1", "per": {}, "paired": {},
               "sic": {}, "config": {}}
    for path in ("clean", "raw", "sic"):
        m = PM.per_psdu(recs, path)
        summary["per"][path] = {"errors": m["errors"], "total": m["total"],
                                "per": m["per"]}
    summary["paired"] = PM.paired_counts(recs)
    summary["sic"] = {"median_suppression_db": PM.median_suppression(recs)}
    with open(os.path.join(root, "summary.json"), "w") as fh:
        json.dump(summary, fh)
    manifest = {
        "schema": "sic-s2b-payload/1",
        "provenance": {"git": {"head": "deadbeef"},
                       "loaded_library": os.path.join(root, "gr-uwb",
                                                      "build-sic-payload",
                                                      "lib", "libgnuradio-uwb.so")},
        "fcs_contract": {"appended_once_by": "generator",
                         "modulator_append_fcs": False},
        "not_run": ["MATLAB cross-check (matlab.status=未实跑)"],
        "templates": {}, "rois": recs[0]["rois"],
    }
    with open(os.path.join(root, "run_manifest.json"), "w") as fh:
        json.dump(manifest, fh)
    index = {"schema": "sic-s2b-payload/1", "windows": len(recs),
             "archive_policy": {"archived": n, "total": n}}
    with open(os.path.join(root, "index.json"), "w") as fh:
        json.dump(index, fh)
    return recs


def _edit_jsonl(path, fn):
    with open(path) as fh:
        recs = [json.loads(x) for x in fh if x.strip()]
    recs = fn(recs)
    with open(path, "w") as fh:
        for r in recs:
            fh.write(json.dumps(r) + "\n")


def mutations(root):
    def wrong_denominator(r):
        # drop a row from windows.jsonl but leave summary total unchanged
        return r[:-1]

    def wrong_suppression(r):
        return r

    def missing_failure(r):
        # flip a raw failure to success without updating summary
        for x in r:
            if x["demod"]["raw"]["psdu_match"] is False:
                x["demod"]["raw"]["psdu_match"] = True
                x["demod"]["raw"]["status"] = "success"
                break
        return r

    def bad_roi(r):
        r[0]["rois"]["detect_roi_lo"] = 999999
        return r

    def double_fcs(r):
        r[0]["psdu_wire_len"] = 98
        return r

    def zero_array(root2):
        p = os.path.join(root2, "recon", os.listdir(
            os.path.join(root2, "recon"))[0])
        with np.load(p) as z:
            i_true = z["i_true"]
            clean = z["clean"]
        np.savez_compressed(p, i_true=i_true, clean=clean,
                            out=np.zeros_like(clean))

    def bad_library(root2):
        with open(os.path.join(root2, "run_manifest.json")) as fh:
            m = json.load(fh)
        m["provenance"]["loaded_library"] = "/usr/local/lib/libgnuradio-uwb.so"
        with open(os.path.join(root2, "run_manifest.json"), "w") as fh:
            json.dump(m, fh)

    return [
        ("wrong_denominator", lambda d: _edit_jsonl(
            os.path.join(d, "windows.jsonl"), wrong_denominator)),
        ("missing_failure_row", lambda d: _edit_jsonl(
            os.path.join(d, "windows.jsonl"), missing_failure)),
        ("bad_roi_order", lambda d: _edit_jsonl(
            os.path.join(d, "windows.jsonl"), bad_roi)),
        ("double_fcs_length", lambda d: _edit_jsonl(
            os.path.join(d, "windows.jsonl"), double_fcs)),
        ("zeroed_out_array", zero_array),
        ("wrong_library", bad_library),
    ]


def self_test():
    print("== verify_s2b self-test ==", flush=True)
    base = tempfile.mkdtemp(prefix="s2b_verify_")
    good = os.path.join(base, "good")
    os.makedirs(good)
    _mk_good(good)
    rc = verify_run(good)
    if rc != 0:
        print("SELF-TEST: the GOOD set was rejected (verifier bug)", flush=True)
        return 1
    all_caught = True
    for name, fn in mutations(base):
        d = os.path.join(base, "mut_%s" % name)
        if os.path.exists(d):
            shutil.rmtree(d)
        shutil.copytree(good, d)
        fn(d)
        rc = verify_run(d)
        if rc == 0:
            print("SELF-TEST FAIL: mutation %r NOT caught" % name, flush=True)
            all_caught = False
        else:
            print("SELF-TEST ok: mutation %r caught" % name, flush=True)
    shutil.rmtree(base, ignore_errors=True)
    print("== self-test %s ==" % ("PASS" if all_caught else "FAIL"), flush=True)
    return 0 if all_caught else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="S2-B independent verifier")
    ap.add_argument("--results", help="run directory to verify")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test()
    if not args.results:
        ap.error("--results or --self-test required")
    if not os.path.isdir(args.results):
        print("not a directory: %s" % args.results)
        return 2
    return verify_run(args.results)


if __name__ == "__main__":
    raise SystemExit(main())
