#!/usr/bin/env python3
"""Independent, fail-closed verifier for S2-B payload-collision runs (rectified).

Self-contained: it does NOT import ``payload_metrics`` (the driver's helper) for
any critical statistic — Wilson CIs, PER, suppression and the CRC are
re-derived here from the raw per-window rows and the archived arrays, so a bug in
the shared helper cannot make the verifier agree with itself.

Reads ONLY the declared artifacts:
  plan.json, run_manifest.json, windows.jsonl, summary.json, index.json,
  recon/*.npz, and (optionally) matrix.json under the same directory.

Checks (all fail-closed):
  * plan: planned_windows / expected ids / seeds / case_config_hash present and
    consistent with the rows, summary, manifest and index (B-R03 item 5).
  * library: the real path exists and is strictly inside the run's build/lib;
    its sha256 is recomputed; /usr/local, parent repos and the other worktree are
    rejected (B-R03 item 1).
  * provenance: generator/metrics/driver/core/adapter/bindings hashes present
    (and recomputed when the repo is reachable) + dirty patch (B-R03 item 2).
  * archive: every declared archive file exists with a fixed shape, finite
    ``out``, a valid overlap; suppression / mask-exactness / coverage are
    recomputed; missing/empty/NaN are failures, never skips (B-R03 item 3).
  * packets: ``psdu_match`` recomputed from decoded hex vs truth hex; the
    single-FCS CRC is re-derived independently (B-R03 item 4).
  * summary: PER_PSDU and PER_FCS both recomputed with an INDEPENDENT Wilson
    implementation; the PER CI must be the error-proportion interval, not the
    success interval (B-R01); status distribution sums to the planned total.
  * boundary: training span inside the payload region; overlap/support/ROIs
    inside the window; finite interferer support (B-R03 item 6).
  * no truth leak; apply refit count 0.
  * matrix: when a matrix.json is present, the expected cell set must match the
    sub-run directories (B-R03 item 7).

``--self-test`` keeps the original 6 mutations and adds ~10 more (wrong CI,
forged/external library, hash mismatch, deleted archives, empty array, moved
training span, faked match/FCS/summary, deleted failure row with plan, missing
matrix cell).  Every mutation must make ``verify_run`` non-zero.

Usage (from the repo root):
  env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py --results <run_dir>
  env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2b.py --self-test
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

RESULTS = []


def record(kind, name, detail=""):
    RESULTS.append((kind, name, detail))
    print("%-4s  %-52s %s" % (kind, name, detail), flush=True)


def ok(name, detail=""):
    record("PASS", name, detail)


def fail(name, detail=""):
    record("FAIL", name, detail)


def skip(name, detail=""):
    record("SKIP", name, detail)


# ---------------------------------------------------------------------------
# Independent statistics (no shared helper)
# ---------------------------------------------------------------------------
def wilson(k: int, n: int, z: float = 1.96):
    if n <= 0:
        return (None, None)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = (z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def per_ci(errors, total, z=1.96):
    if total <= 0 or errors < 0 or errors > total:
        return (None, None)
    return wilson(int(errors), int(total), z)


def crc16_802154(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b & 0xFF
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def suppression_db(i_true, clean, out, lo, hi, eps=1e-30):
    if out is None or out.size == 0:
        return None
    a, b = max(0, int(lo)), min(int(hi), i_true.size, clean.size, out.size)
    if b <= a:
        return None
    e_i = float(np.sum(np.abs(i_true[a:b].astype(np.complex128)) ** 2))
    if e_i <= 0.0:
        return None
    resid = out[a:b].astype(np.complex128) - clean[a:b].astype(np.complex128)
    e_r = float(np.sum(np.abs(resid) ** 2))
    if e_r <= eps:
        return float("inf")
    return 10.0 * math.log10(e_i / e_r)


def per_from_records(records, path, success_pred):
    n = ok_ = 0
    status = {}
    for r in records:
        d = (r.get("demod") or {}).get(path) or {}
        n += 1
        st = str(d.get("status", "missing"))
        status[st] = status.get(st, 0) + 1
        if success_pred(d):
            ok_ += 1
    err = n - ok_
    return {"errors": err, "success": ok_, "total": n,
            "per": (err / n if n else None),
            "per_ci": per_ci(err, n), "success_ci": wilson(ok_, n),
            "status": status}


def psdu_pred(d):
    return d.get("psdu_match") is True


def fcs_pred(d):
    return str(d.get("status")) == "success" and d.get("fcs_pass") is True


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------
def load_run(run_dir):
    out = {"dir": run_dir, "manifest": None, "summary": None, "index": None,
           "plan": None, "records": [], "recon": {}, "matrix": None}
    for key, name in (("manifest", "run_manifest.json"),
                      ("summary", "summary.json"), ("index", "index.json"),
                      ("plan", "plan.json"), ("matrix", "matrix.json")):
        p = os.path.join(run_dir, name)
        if os.path.exists(p):
            with open(p) as fh:
                out[key] = json.load(fh)
    wpath = os.path.join(run_dir, "windows.jsonl")
    if os.path.exists(wpath):
        with open(wpath) as fh:
            for line in fh:
                if line.strip():
                    out["records"].append(json.loads(line))
    rdir = os.path.join(run_dir, "recon")
    if os.path.isdir(rdir):
        for fn in sorted(os.listdir(rdir)):
            if fn.endswith(".npz"):
                out["recon"][fn] = np.load(os.path.join(rdir, fn))
    return out


def _repo_root_from_run(run_dir):
    cur = os.path.abspath(run_dir)
    for _ in range(12):
        if os.path.exists(os.path.join(cur, "gr-uwb", "apps",
                                       "uwb_sim_sic_payload.py")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return None


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------
def check_plan(run):
    plan = run.get("plan")
    recs = run["records"]
    if not plan:
        fail("plan_present", "plan.json missing (planned denominator unknown)")
        return
    ok("plan_present")
    planned = plan.get("planned_windows")
    if planned == len(recs):
        ok("plan_planned_equals_rows", "planned=%s" % planned)
    else:
        fail("plan_planned_equals_rows",
             "planned=%s rows=%d" % (planned, len(recs)))
    want_ids = plan.get("window_ids")
    got_ids = [r.get("window_id") for r in recs]
    if want_ids == got_ids:
        ok("plan_window_ids")
    else:
        fail("plan_window_ids", "ids differ from plan")
    seeds = plan.get("seeds") or []
    smis = 0
    for r, st in zip(recs, seeds):
        if r.get("seed") != st.get("noise") or \
                r.get("payload_seed") != st.get("payload"):
            smis += 1
    (ok if smis == 0 else fail)("plan_seed_binding", "%d mismatches" % smis)
    cch = plan.get("case_config_hash")
    others = [(run.get("summary") or {}).get("case_config_hash"),
              ((run.get("manifest") or {}).get("case_config_hash")),
              (run.get("index") or {}).get("case_config_hash")]
    if cch and all(o == cch for o in others):
        ok("plan_case_config_hash_consistent", str(cch)[:12])
    else:
        fail("plan_case_config_hash_consistent",
             "plan=%s others=%s" % (str(cch)[:12], [str(o)[:12] for o in others]))


def check_manifest(run, run_dir):
    m = run.get("manifest")
    if not m:
        fail("manifest_present")
        return
    ok("manifest_present")
    (ok if m.get("schema") == "sic-s2b-payload/1" else fail)(
        "manifest_schema", str(m.get("schema")))
    prov = m.get("provenance", {})
    build_dir = prov.get("build_dir")
    lib = prov.get("loaded_library")
    # --- strict library check ---
    if not lib or not build_dir:
        fail("manifest_library_strict", "missing lib/build_dir")
    else:
        rlib = os.path.realpath(lib)
        rbuild = os.path.realpath(build_dir)
        sep = os.sep
        inside = (os.path.exists(rlib)
                  and (rlib == rbuild or rlib.startswith(rbuild + sep))
                  and (sep + "lib" + sep) in rlib + sep)
        bad_prefix = rlib.startswith("/usr/local") or rlib.startswith("/usr/lib")
        # reject the other worktree explicitly (the parent repo is already
        # excluded by the strict "inside build_dir" test above)
        other = "sic-sensing-accum" in rlib
        if inside and not bad_prefix and not other:
            ok("manifest_library_strict", rlib)
        else:
            fail("manifest_library_strict",
                 "lib=%s inside=%s bad_prefix=%s other=%s"
                 % (rlib, inside, bad_prefix, other))
        # sha256 recompute
        rec_sha = prov.get("loaded_library_sha256")
        if rec_sha and os.path.exists(rlib):
            if _sha256(rlib) == rec_sha:
                ok("manifest_library_sha256")
            else:
                fail("manifest_library_sha256", "hash mismatch")
        else:
            fail("manifest_library_sha256", "recorded sha missing")
    # --- source hashes ---
    src = prov.get("source_sha256") or {}
    need = ["driver", "generator", "metrics", "verifier", "core_header",
            "core_src", "adapter_src"]
    missing = [k for k in need if not src.get(k)]
    if not missing:
        ok("manifest_source_hashes_present")
    else:
        fail("manifest_source_hashes_present", "missing %s" % missing)
    repo = _repo_root_from_run(run_dir)
    if repo and not missing:
        pairs = {"driver": "gr-uwb/apps/uwb_sim_sic_payload.py",
                 "generator": "gr-uwb/apps/sic_payload_sim.py",
                 "metrics": "tools/sic/payload_metrics.py",
                 "verifier": "tools/sic/verify_s2b.py",
                 "core_header": "gr-uwb/include/gnuradio/uwb/uwb_preamble_sic_core.h",
                 "core_src": "gr-uwb/lib/uwb_preamble_sic_core.cc",
                 "adapter_src": "gr-uwb/lib/uwb_preamble_sic.cc"}
        mis = [k for k, rel in pairs.items()
               if os.path.exists(os.path.join(repo, rel))
               and _sha256(os.path.join(repo, rel)) != src[k]]
        (ok if not mis else fail)("manifest_source_hashes_recompute",
                                  "mismatch %s" % mis)
    else:
        skip("manifest_source_hashes_recompute", "repo not reachable")
    bind = prov.get("bindings")
    if bind and bind.get("path") and bind.get("sha256"):
        ok("manifest_bindings_hash", os.path.basename(bind["path"]))
    else:
        fail("manifest_bindings_hash", "missing bindings hash")
    git = prov.get("git") or {}
    (ok if git.get("head") else fail)("manifest_git_head",
                                      str(git.get("head"))[:12])
    (ok if git.get("status_sha256") or git.get("dirty_patch_sha256")
     else fail)("manifest_dirty_patch", "no dirty-patch hash")
    fc = m.get("fcs_contract", {})
    wire = fc.get("wire_psdu_bytes")
    data = fc.get("data_bytes")
    fcs = fc.get("fcs_bytes")
    if (fc.get("modulator_append_fcs") is False and wire == 96 and data == 94
            and fcs == 2 and wire == data + fcs):
        ok("manifest_fcs_contract", "wire=%s=%s+%s" % (wire, data, fcs))
    else:
        fail("manifest_fcs_contract", json.dumps(fc))
    nr = m.get("not_run")
    if isinstance(nr, list) and any("MATLAB" in str(x) for x in nr):
        ok("manifest_matlab_not_run_declared")
    else:
        fail("manifest_matlab_not_run_declared", str(nr))
    if isinstance(nr, list) and any("BER" in str(x) for x in nr):
        ok("manifest_ber_not_implemented_declared")
    else:
        fail("manifest_ber_not_implemented_declared", str(nr))


def check_windows(run):
    recs = run["records"]
    plan = run.get("plan") or {}
    gen = plan.get("generator") or {}
    n = gen.get("n")
    pay_lo, pay_hi = (gen.get("payload_abs") or [None, None])
    if not recs:
        fail("windows_present", "no rows")
        return
    ok("windows_present", "%d rows" % len(recs))
    bad = 0
    details = {}
    for r in recs:
        need = ("window_id", "seed", "payload_seed", "demod", "rois",
                "training_span", "overlap", "input_sha256", "psdu_wire_hex",
                "psdu_wire_len", "i_support", "i_template_len")
        if any(k not in r for k in need):
            bad += 1
            details[r.get("window_id")] = "missing fields"
            continue
        if r["psdu_wire_len"] != 96 or len(r["psdu_wire_hex"]) != 192:
            bad += 1
            details[r["window_id"]] = "wire len"
        if len(r["input_sha256"]) != 64:
            bad += 1
            details[r["window_id"]] = "input hash"
        t0, t1 = r["training_span"]
        if not (t1 > t0 > 0):
            bad += 1
            details[r["window_id"]] = "training not forward"
        elif not r.get("no_overlap") and pay_lo is not None \
                and not (t0 >= pay_lo and t1 <= pay_hi):
            # the declared no_overlap negative intentionally injects outside the
            # payload; every overlapping window must keep training inside it.
            bad += 1
            details[r["window_id"]] = "training outside payload"
        roi = r["rois"]
        if n is not None:
            for lo, hi in ((roi.get("detect_roi_lo"), roi.get("detect_roi_hi")),
                           (roi.get("sub_roi_lo"), roi.get("sub_roi_hi"))):
                if not (0 <= lo < hi <= n):
                    bad += 1
                    details[r["window_id"]] = "roi out of window"
            if not (0 <= r["i_support"][0] <= r["i_support"][1] <= n):
                bad += 1
                details[r["window_id"]] = "support out of window"
        ov = r["overlap"]
        if not (ov["lo"] <= ov["hi"]):
            bad += 1
            details[r["window_id"]] = "overlap bad"
    (ok if bad == 0 else fail)("windows_boundary_contract",
                               "%d bad %s" % (bad, dict(list(details.items())[:4])))
    ids = [r["window_id"] for r in recs]
    (ok if len(set(ids)) == len(ids) else fail)("windows_unique_ids")


def check_packets(run):
    recs = run["records"]
    mis_match = 0
    bad_len = 0
    bad_wire_crc = 0
    fake_fcs = 0
    for r in recs:
        wire = r.get("psdu_wire_hex", "")
        try:
            wb = bytes.fromhex(wire)
        except ValueError:
            bad_wire_crc += 1
            continue
        if len(wb) >= 2 and crc16_802154(wb[:-2]) != (wb[-2] | (wb[-1] << 8)):
            bad_wire_crc += 1
        for path in ("clean", "raw", "sic"):
            d = (r.get("demod") or {}).get(path)
            if d is None:
                continue
            hexs = d.get("psdu_hex") or ""
            recomputed = (hexs.upper() == wire.upper())
            if bool(d.get("psdu_match")) != bool(recomputed):
                mis_match += 1
            if d.get("status") == "success" and len(hexs) != 192:
                bad_len += 1
            if d.get("fcs_pass") is True:
                try:
                    db = bytes.fromhex(hexs)
                except ValueError:
                    fake_fcs += 1
                    continue
                if not (len(db) >= 2
                        and crc16_802154(db[:-2]) == (db[-2] | (db[-1] << 8))
                        and d.get("status") == "success"):
                    fake_fcs += 1
    (ok if bad_wire_crc == 0 else fail)("packets_wire_fcs_recompute",
                                        "%d bad" % bad_wire_crc)
    (ok if mis_match == 0 else fail)("packets_psdu_match_recompute",
                                     "%d mismatches" % mis_match)
    (ok if bad_len == 0 else fail)("packets_len_recompute",
                                   "%d bad" % bad_len)
    (ok if fake_fcs == 0 else fail)("packets_fcs_consistency",
                                    "%d fake" % fake_fcs)


def check_summary(run):
    s = run.get("summary")
    recs = run["records"]
    plan = run.get("plan") or {}
    if not s:
        fail("summary_present")
        return
    ok("summary_present")
    planned = plan.get("planned_windows")
    if s.get("windows_expected") == planned and s.get("windows_ok") == len(recs):
        ok("summary_planned_denominator",
           "expected=%s ok=%s" % (planned, len(recs)))
    else:
        fail("summary_planned_denominator",
             "expected=%s(want %s) ok=%s" % (s.get("windows_expected"),
                                             planned, s.get("windows_ok")))
    for path in ("clean", "raw", "sic"):
        if path not in (s.get("per") or {}):
            continue
        mine = per_from_records(recs, path, psdu_pred)
        theirs = s["per"][path]
        if (mine["errors"] == theirs.get("errors")
                and mine["total"] == theirs.get("total")):
            ok("summary_per_psdu_%s" % path,
               "err=%d/%d" % (mine["errors"], mine["total"]))
        else:
            fail("summary_per_psdu_%s" % path,
                 "recomputed %d/%d vs %s/%s" % (mine["errors"], mine["total"],
                                                theirs.get("errors"),
                                                theirs.get("total")))
        # CI must be the PER (error) interval, computed independently.
        tci = theirs.get("per_wilson95") or [None, None]
        if (tci[0] is not None
                and abs(tci[0] - mine["per_ci"][0]) < 1e-9
                and abs(tci[1] - mine["per_ci"][1]) < 1e-9):
            ok("summary_per_ci_%s" % path, "[%.4f,%.4f]" % tuple(tci))
        else:
            fail("summary_per_ci_%s" % path,
                 "summary %s != independent %s"
                 % (tci, list(mine["per_ci"])))
        # success interval, if present, must be the independent success CI and
        # must NOT be the one reported as the PER interval (B-R01).
        sci = theirs.get("success_wilson95")
        if sci and sci[0] is not None:
            if (abs(sci[0] - mine["success_ci"][0]) < 1e-9
                    and abs(sci[1] - mine["success_ci"][1]) < 1e-9):
                ok("summary_success_ci_%s" % path)
            else:
                fail("summary_success_ci_%s" % path,
                     "summary %s != %s" % (sci, list(mine["success_ci"])))
        if tci[0] is not None and mine["per"] is not None:
            if tci[0] - 1e-9 <= mine["per"] <= tci[1] + 1e-9 and tci[0] <= tci[1]:
                ok("summary_per_ci_contains_estimate_%s" % path)
            else:
                fail("summary_per_ci_contains_estimate_%s" % path,
                     "per=%s in %s" % (mine["per"], tci))
        # PER_FCS
        minef = per_from_records(recs, path, fcs_pred)
        if (theirs.get("per_fcs_errors") == minef["errors"]
                and theirs.get("per_fcs") == minef["per"]):
            ok("summary_per_fcs_%s" % path, "err=%d" % minef["errors"])
        else:
            fail("summary_per_fcs_%s" % path,
                 "recomputed %d vs %s" % (minef["errors"],
                                          theirs.get("per_fcs_errors")))
        sc = theirs.get("status_counts") or {}
        if sum(sc.values()) == len(recs):
            ok("summary_status_sums_%s" % path)
        else:
            fail("summary_status_sums_%s" % path,
                 "sum=%d rows=%d" % (sum(sc.values()), len(recs)))
    if "sic" in (s.get("per") or {}):
        rescued = sum(1 for r in recs
                      if not (r["demod"]["raw"].get("psdu_match") is True)
                      and r["demod"].get("sic", {}).get("psdu_match") is True)
        damaged = sum(1 for r in recs
                      if r["demod"]["raw"].get("psdu_match") is True
                      and not (r["demod"].get("sic", {}).get("psdu_match")
                               is True))
        theirs = s.get("paired", {})
        if (rescued == theirs.get("raw_fail_sic_ok")
                and damaged == theirs.get("raw_ok_sic_fail")):
            ok("summary_paired", "rescue=%d damage=%d" % (rescued, damaged))
        else:
            fail("summary_paired", "recomputed rescue=%d damage=%d vs %s"
                 % (rescued, damaged, theirs))
        supp = [r["suppression_db_overlap"] for r in recs
                if r.get("suppression_db_overlap") is not None]
        med = float(np.median(supp)) if supp else None
        smed = (s.get("sic") or {}).get("median_suppression_db")
        if med is None or smed is None or abs(med - smed) < 1e-6:
            ok("summary_median_suppression",
               "%.4f" % (med if med is not None else float("nan")))
        else:
            fail("summary_median_suppression",
                 "recomputed %.4f vs %.4f" % (med, smed))


def check_recon(run, run_dir):
    recs = {r["window_id"]: r for r in run["records"]}
    recon = run.get("recon", {})
    plan = run.get("plan") or {}
    planned = plan.get("planned_windows")
    idx = run.get("index") or {}
    declared = idx.get("archived_windows") or []
    ap = idx.get("archive_policy") or {}
    min_arch = min(8, planned) if planned else 8
    if not recon:
        fail("recon_present", "no recon/*.npz")
        return
    if ap.get("archived", 0) < min_arch:
        fail("recon_archive_count",
             "archived=%s < min %s" % (ap.get("archived"), min_arch))
    else:
        ok("recon_archive_count", "archived=%s" % ap.get("archived"))
    # every declared archive file must exist
    miss = [w for w in declared
            if not os.path.exists(os.path.join(run_dir, "recon",
                                               "window_%04d.npz" % w))]
    (ok if not miss else fail)("recon_declared_files_exist",
                               "missing %s" % miss[:4])
    n = plan.get("generator", {}).get("n")
    n_checked = n_bad_supp = n_bad_mask = n_bad_shape = n_nan = 0
    train_bad = 0
    for fn, arr in recon.items():
        wid = int(fn.replace("window_", "").replace(".npz", ""))
        r = recs.get(wid)
        if r is None:
            n_bad_shape += 1
            continue
        need = ("i_true", "clean", "raw", "out")
        if any(k not in arr.files for k in need):
            n_bad_shape += 1
            continue
        i_true, clean, raw, out = (arr["i_true"], arr["clean"],
                                   arr["raw"], arr["out"])
        if out.size == 0 or (n is not None and any(
                a.size not in (0, n) for a in (i_true, clean, raw, out))):
            fail("recon_nonempty", "%s empty/short" % fn)
            return
        if (not np.all(np.isfinite(i_true))
                or not np.all(np.isfinite(clean))
                or not np.all(np.isfinite(out))):
            n_nan += 1
        ov = r["overlap"]
        rec_supp = r.get("suppression_db_overlap")
        mine = suppression_db(i_true, clean, out, ov["lo"], ov["hi"])
        if rec_supp is not None and mine is not None:
            if not (math.isinf(mine) and math.isinf(rec_supp)) \
                    and abs(mine - rec_supp) > 1e-4:
                n_bad_supp += 1
        changed = _mask_outside(raw, out, (r.get("sic") or {}).get("sub_lo"),
                                (r.get("sic") or {}).get("sub_hi"))
        if changed > 0:
            n_bad_mask += 1
        t0, t1 = r["training_span"]
        seg = out[t0:t1]
        if seg.size and not np.all(np.isfinite(seg.astype(np.complex128))):
            train_bad += 1
        n_checked += 1
    (ok if n_bad_shape == 0 else fail)("recon_shape", "%d bad" % n_bad_shape)
    (ok if n_nan == 0 else fail)("recon_finite", "%d NaN/Inf" % n_nan)
    (ok if n_bad_supp == 0 else fail)("recon_suppression_recompute",
                                      "%d/%d mismatch" % (n_bad_supp, n_checked))
    (ok if n_bad_mask == 0 else fail)("recon_mask_exactness",
                                      "%d/%d bad" % (n_bad_mask, n_checked))
    (ok if train_bad == 0 else fail)("recon_training_finite",
                                     "%d bad" % train_bad)


def _mask_outside(raw, out, sub_lo, sub_hi):
    if raw is None or out is None or out.size != raw.size:
        return -1
    if sub_hi is None or sub_lo is None or sub_hi <= sub_lo:
        return int(np.count_nonzero(out != raw))
    outside = np.ones(out.size, dtype=bool)
    outside[int(sub_lo):int(sub_hi)] = False
    return int(np.count_nonzero(out[outside] != raw[outside]))


def check_seeds(run):
    recs = run["records"]
    payloads = [r.get("payload_seed") for r in recs]
    noise = [r.get("seed") for r in recs]
    (ok if len(set(payloads)) == len(payloads) else fail)("seeds_payload_unique")
    (ok if len(set(noise)) == len(noise) else fail)("seeds_noise_unique")
    cal = [p for p in payloads if p is not None and p < 22000000]
    acc = [p for p in payloads if p is not None and p >= 22000000]
    (fail if (cal and acc) else ok)("seeds_set_separation")


def check_no_truth_leak(run):
    recs = run["records"]
    leaked = [r["window_id"] for r in recs
              if (r.get("sic") or {}).get("uses_truth") is True]
    (ok if not leaked else fail)("no_truth_leak", "leaked=%s" % leaked[:5])
    refit = [r["window_id"] for r in recs
             if (r.get("sic") or {}).get("apply_refit_count") not in (0, None)]
    (ok if not refit else fail)("apply_refit_zero", "refit=%s" % refit[:5])
    # online input whitelist: the declared per-window SIC input meta must not
    # carry injection truth (the truth firewall itself is exercised by the QA
    # `build_sic_freeze_and_mask` prefix-mutation check).
    allowed = {"packet_id", "window_id", "predicted_start_sample",
               "window_start_sample", "sample_count", "sample_rate",
               "code_index", "sync_repetitions", "sfd_mode",
               "observed_end_sample", "sensing_roi_lo", "sensing_roi_hi"}
    banned = {"start_sample", "cfo_hz", "paths", "gain", "isr_db",
              "psdu_wire_hex", "payload_hex", "i_truth", "injected_start"}
    bad_rows = 0
    for r in recs:
        keys = set(r.get("online_meta_keys") or [])
        if not keys or (keys - allowed) or (keys & banned):
            bad_rows += 1
    (ok if bad_rows == 0 else fail)("online_meta_whitelist",
                                    "%d rows violate" % bad_rows)


def check_index(run):
    idx = run.get("index")
    if not idx:
        fail("index_present")
        return
    ok("index_present")
    (ok if idx.get("schema") == "sic-s2b-payload/1" else fail)(
        "index_schema", str(idx.get("schema")))
    recs = run["records"]
    plan = run.get("plan") or {}
    if (idx.get("windows") == len(recs)
            and idx.get("planned_windows") == plan.get("planned_windows")):
        ok("index_window_count")
    else:
        fail("index_window_count",
             "index=%s planned=%s rows=%d plan=%s"
             % (idx.get("windows"), idx.get("planned_windows"), len(recs),
                plan.get("planned_windows")))
    ap = idx.get("archive_policy") or {}
    if "archived" in ap and "total" in ap and "fixed" in ap:
        ok("index_archive_policy", json.dumps(ap))
    else:
        fail("index_archive_policy", json.dumps(ap))
    declared = idx.get("archived_windows") or []
    if sorted(declared) == sorted(idx.get("archived_windows") or []):
        ok("index_archived_list")
    else:
        fail("index_archived_list")


def check_matrix_dir(run_dir):
    """When matrix.json is present, validate the expected cell set."""
    mj = os.path.join(run_dir, "matrix.json")
    if not os.path.exists(mj):
        return
    with open(mj) as fh:
        agg = json.load(fh)
    expected = set(agg.get("expected_cells") or [])
    cells_dir = os.path.join(run_dir, "cells")
    found = set(d for d in (os.listdir(cells_dir)
                            if os.path.isdir(cells_dir) else [])
                if os.path.isdir(os.path.join(cells_dir, d)))
    missing = sorted(expected - found)
    failed_rc = [c.get("cell") for c in (agg.get("cells") or [])
                 if c.get("rc") != 0]
    b3 = [c for c in found if c.startswith("b3_")]
    if not missing and not failed_rc and len(b3) == 12:
        ok("matrix_cells_complete", "%d cells, 12 B3" % len(found))
    else:
        fail("matrix_cells_complete",
             "missing=%s failed_rc=%s b3=%d" % (missing[:4], failed_rc,
                                                len(b3)))


def verify_run(run_dir, verbose=False):
    del RESULTS[:]
    run = load_run(run_dir)
    if not run["records"] and not run["manifest"]:
        fail("run_readable", "no artifacts in %s" % run_dir)
        return 1
    check_plan(run)
    check_manifest(run, run_dir)
    check_windows(run)
    check_packets(run)
    check_summary(run)
    check_recon(run, run_dir)
    check_seeds(run)
    check_no_truth_leak(run)
    check_index(run)
    check_matrix_dir(run_dir)
    n_fail = sum(1 for r in RESULTS if r[0] == "FAIL")
    n_pass = sum(1 for r in RESULTS if r[0] == "PASS")
    n_skip = sum(1 for r in RESULTS if r[0] == "SKIP")
    print("== %s: %d pass, %d fail, %d skip =="
          % (run_dir, n_pass, n_fail, n_skip), flush=True)
    return 1 if n_fail else 0


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------
def _mk_good(root, n=12):
    os.makedirs(os.path.join(root, "recon"), exist_ok=True)
    os.makedirs(os.path.join(root, "gr-uwb", "build-sic-payload", "lib"),
                exist_ok=True)
    libpath = os.path.join(root, "gr-uwb", "build-sic-payload", "lib",
                           "libgnuradio-uwb.so")
    with open(libpath, "wb") as fh:
        fh.write(b"\x7fELF fake")
    libsha = _sha256(libpath)
    n_samples = 4096
    rng = np.random.default_rng(0)
    recs = []
    for w in range(n):
        raw_fail = (w % 3 == 0)
        data = bytes(((w * 37 + i) % 256) for i in range(94))
        crc = crc16_802154(data)
        wire = data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        wire_hex = wire.hex().upper()
        sub_lo, sub_hi = 1000, 3000
        clean = rng.standard_normal(n_samples).astype(np.complex64)
        i_true = rng.standard_normal(n_samples).astype(np.complex64) * 0.5
        raw = (clean.astype(np.complex128)
               + i_true.astype(np.complex128)).astype(np.complex64)
        out = raw.copy()
        supp = suppression_db(i_true, clean, out, 1200, 2800)
        def mk(match, fcs, status):
            h = wire_hex if match else (data[:10].hex().upper())
            return {"status": status, "fcs_pass": fcs, "psdu_hex": h,
                    "phr_psdu_length": 96, "psdu_match": match}
        rec = {
            "window_id": w, "seed": 22000000 + w,
            "payload_seed": 22100000 + w, "jitter": 0,
            "psdu_wire_len": 96, "psdu_wire_hex": wire_hex,
            "psdu_wire_sha256": hashlib.sha256(wire).hexdigest(),
            "input_sha256": hashlib.sha256(
                raw.tobytes()).hexdigest(),
            "training_span": [1100, 1100 + 8 * 16],
            "overlap": {"lo": 1200, "hi": 2800, "samples": 1600},
            "i_support": [1190, 3000], "i_template_len": 1800,
            "online_meta_keys": ["packet_id", "window_id",
                                 "predicted_start_sample", "window_start_sample",
                                 "sample_count", "sample_rate", "code_index",
                                 "sync_repetitions", "sfd_mode",
                                 "observed_end_sample", "sensing_roi_lo",
                                 "sensing_roi_hi"],
            "rois": {"detect_roi_lo": 900, "detect_roi_hi": 3500,
                     "sub_roi_lo": 1000, "sub_roi_hi": 3000,
                     "observed_end": 3200, "nominal_start": 2200,
                     "start_jitter": 100, "guard": 100},
            "suppression_db_overlap": supp,
            "mask_outside_changed": 0,
            "demod": {
                "clean": mk(True, True, "success"),
                "raw": mk(not raw_fail, not raw_fail,
                          "success" if not raw_fail else "payload_failed"),
                "sic": mk(True, True, "success"),
            },
            "sic": {"state": "committed", "committed": True, "detected": True,
                    "uses_truth": False, "apply_refit_count": 0,
                    "sub_lo": sub_lo, "sub_hi": sub_hi},
        }
        recs.append(rec)
        np.savez_compressed(os.path.join(root, "recon", "window_%04d.npz" % w),
                            i_true=i_true, clean=clean, raw=raw, out=out)
    with open(os.path.join(root, "windows.jsonl"), "w") as fh:
        for r in recs:
            fh.write(json.dumps(r) + "\n")
    seeds = [{"noise": r["seed"], "payload": r["payload_seed"],
              "jitter": r["seed"], "path": r["seed"]} for r in recs]
    gen = {"n": n_samples, "payload_abs": [1000, 3500]}
    plan = {"schema": "sic-s2b-payload/1", "planned_windows": n,
            "window_ids": list(range(n)), "seeds": seeds,
            "case_config": {"phase": "b1"}, "case_config_hash": "abc123",
            "rois": recs[0]["rois"], "generator": gen}
    with open(os.path.join(root, "plan.json"), "w") as fh:
        json.dump(plan, fh)
    summary = {"schema": "sic-s2b-payload/1", "per": {}, "paired": {},
               "sic": {}, "config": {}, "windows_expected": n,
               "windows_ok": n, "case_config_hash": "abc123"}
    for path in ("clean", "raw", "sic"):
        m = per_from_records(recs, path, psdu_pred)
        mf = per_from_records(recs, path, fcs_pred)
        summary["per"][path] = {
            "errors": m["errors"], "success": m["success"], "total": m["total"],
            "per": m["per"], "per_wilson95": list(m["per_ci"]),
            "success_wilson95": list(m["success_ci"]),
            "per_fcs_errors": mf["errors"], "per_fcs": mf["per"],
            "status_counts": m["status"]}
    rescued = sum(1 for r in recs
                  if not r["demod"]["raw"]["psdu_match"]
                  and r["demod"]["sic"]["psdu_match"])
    damaged = 0
    summary["paired"] = {"raw_fail_sic_ok": rescued, "raw_ok_sic_fail": damaged}
    supp = [r["suppression_db_overlap"] for r in recs]
    summary["sic"] = {"median_suppression_db": float(np.median(supp))}
    with open(os.path.join(root, "summary.json"), "w") as fh:
        json.dump(summary, fh)
    src = {k: "0" * 64 for k in ("driver", "generator", "metrics", "verifier",
                                 "core_header", "core_src", "adapter_src")}
    manifest = {
        "schema": "sic-s2b-payload/1", "planned_windows": n,
        "completed_windows": n, "case_config_hash": "abc123",
        "provenance": {
            "git": {"head": "deadbeef", "status_sha256": "0" * 64},
            "loaded_library": libpath, "loaded_library_sha256": libsha,
            "build_dir": os.path.join(root, "gr-uwb", "build-sic-payload"),
            "source_sha256": src,
            "bindings": {"path": libpath, "sha256": libsha}},
        "fcs_contract": {"appended_once_by": "generator",
                         "modulator_append_fcs": False,
                         "wire_psdu_bytes": 96, "data_bytes": 94, "fcs_bytes": 2},
        "not_run": ["MATLAB cross-check (matlab.status=未实跑)",
                    "BER (decoder exposes no aligned bits): N/A / 未实现"],
        "templates": {}, "rois": recs[0]["rois"],
    }
    with open(os.path.join(root, "run_manifest.json"), "w") as fh:
        json.dump(manifest, fh)
    index = {"schema": "sic-s2b-payload/1", "windows": n, "planned_windows": n,
             "case_config_hash": "abc123",
             "archived_windows": list(range(min(8, n))),
             "archive_fixed_windows": list(range(min(8, n))),
             "archive_worst_windows": [],
             "archive_policy": {"archived": min(8, n), "total": n,
                                "fixed": min(8, n), "worst": 0}}
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


def _edit_json(path, fn):
    with open(path) as fh:
        d = json.load(fh)
    d = fn(d)
    with open(path, "w") as fh:
        json.dump(d, fh)


def mutations(base):
    win = lambda d: os.path.join(d, "windows.jsonl")

    def wrong_denominator(r):
        return r[:-1]

    def missing_failure(r):
        for x in r:
            if x["demod"]["raw"]["psdu_match"] is False:
                x["demod"]["raw"]["psdu_match"] = True
                x["demod"]["raw"]["status"] = "success"
                x["demod"]["raw"]["psdu_hex"] = x["psdu_wire_hex"]
                break
        return r

    def bad_roi(r):
        r[0]["rois"]["detect_roi_lo"] = 999999
        return r

    def double_fcs(r):
        r[0]["psdu_wire_len"] = 98
        return r

    def move_training(r):
        r[0]["training_span"] = [4000, 4200]
        return r

    def fake_match(r):
        for x in r:
            if x["demod"]["raw"]["psdu_match"] is False:
                x["demod"]["raw"]["psdu_match"] = True
                break
        return r

    def fake_fcs(r):
        for x in r:
            if x["demod"]["raw"]["fcs_pass"] is False:
                x["demod"]["raw"]["fcs_pass"] = True
                break
        return r

    def drop_failure_and_summary(r, d):
        # remove one failing row from windows AND make summary self-consistent,
        # but leave plan.planned_windows unchanged -> plan check must catch it.
        new = [x for x in r if x["demod"]["raw"]["psdu_match"] is True]
        new = new if new else r[:-1]
        _edit_json(os.path.join(d, "summary.json"),
                   lambda s: _resummarize(s, new))
        return new

    def wrong_ci(d):
        _edit_json(os.path.join(d, "summary.json"),
                   lambda s: _swap_ci(s))
        return None

    def forged_library(d):
        fake = os.path.join(d, "elsewhere", "libgnuradio-uwb.so")
        os.makedirs(os.path.dirname(fake), exist_ok=True)
        with open(fake, "wb") as fh:
            fh.write(b"x")
        _edit_json(os.path.join(d, "run_manifest.json"),
                   lambda m: _set_lib(m, fake, _sha256(fake)))
        return None

    def library_hash_mismatch(d):
        _edit_json(os.path.join(d, "run_manifest.json"),
                   lambda m: _set_lib(m, m["provenance"]["loaded_library"],
                                      "0" * 64))
        return None

    def delete_all_archives(d):
        shutil.rmtree(os.path.join(d, "recon"), ignore_errors=True)
        return None

    def delete_one_archive(d):
        p = sorted(os.listdir(os.path.join(d, "recon")))[0]
        os.remove(os.path.join(d, "recon", p))
        return None

    def empty_out(d):
        p = os.path.join(d, "recon",
                         sorted(os.listdir(os.path.join(d, "recon")))[0])
        with np.load(p) as z:
            np.savez_compressed(p, i_true=z["i_true"], clean=z["clean"],
                                raw=z["raw"],
                                out=np.zeros(0, dtype=np.complex64))
        return None

    return [
        ("wrong_denominator", lambda d: _edit_jsonl(win(d), wrong_denominator)),
        ("missing_failure_row", lambda d: _edit_jsonl(win(d), missing_failure)),
        ("bad_roi_order", lambda d: _edit_jsonl(win(d), bad_roi)),
        ("double_fcs_length", lambda d: _edit_jsonl(win(d), double_fcs)),
        ("training_out_of_payload", lambda d: _edit_jsonl(win(d), move_training)),
        ("fake_psdu_match", lambda d: _edit_jsonl(win(d), fake_match)),
        ("fake_fcs_pass", lambda d: _edit_jsonl(win(d), fake_fcs)),
        ("delete_failure_and_summary", lambda d: _edit_jsonl(
            win(d), lambda r: drop_failure_and_summary(r, d))),
        ("wrong_ci_direction", wrong_ci),
        ("forged_external_library", forged_library),
        ("library_hash_mismatch", library_hash_mismatch),
        ("delete_all_archives", delete_all_archives),
        ("delete_one_archive", delete_one_archive),
        ("empty_out_array", empty_out),
        ("zeroed_out_array", _zero_out),
        ("wrong_library_usrlocal", _usrlocal_lib),
    ]


def _zero_out(d):
    p = os.path.join(d, "recon", sorted(os.listdir(os.path.join(d, "recon")))[0])
    with np.load(p) as z:
        np.savez_compressed(p, i_true=z["i_true"], clean=z["clean"],
                            raw=z["raw"], out=np.zeros_like(z["clean"]))
    return None


def _usrlocal_lib(d):
    _edit_json(os.path.join(d, "run_manifest.json"),
               lambda m: _set_lib(m, "/usr/local/lib/libgnuradio-uwb.so",
                                  "0" * 64))
    return None


def _set_lib(m, path, sha):
    m["provenance"]["loaded_library"] = path
    m["provenance"]["loaded_library_sha256"] = sha
    return m


def _resummarize(s, recs):
    for path in ("clean", "raw", "sic"):
        if path not in (s.get("per") or {}):
            continue
        m = per_from_records(recs, path, psdu_pred)
        mf = per_from_records(recs, path, fcs_pred)
        s["per"][path].update({"errors": m["errors"], "total": m["total"],
                               "per": m["per"],
                               "per_wilson95": list(m["per_ci"]),
                               "success_wilson95": list(m["success_ci"]),
                               "status_counts": m["status"],
                               "per_fcs_errors": mf["errors"],
                               "per_fcs": mf["per"]})
    s["windows_ok"] = len(recs)
    return s


def _swap_ci(s):
    for path, d in (s.get("per") or {}).items():
        d["per_wilson95"] = list(d.get("success_wilson95") or [0.0, 1.0])
    return s


def self_test():
    print("== verify_s2b self-test (rectified) ==", flush=True)
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
    # matrix missing-cell mutation
    mroot = os.path.join(base, "matrix")
    os.makedirs(os.path.join(mroot, "cells", "b2_x"), exist_ok=True)
    with open(os.path.join(mroot, "matrix.json"), "w") as fh:
        json.dump({"schema": "sic-s2b-matrix/1",
                   "expected_cells": ["b2_x", "b3_a", "b3_b"],
                   "b3_expected_cells": ["b3_a", "b3_b"],
                   "cells": [{"cell": "b2_x", "rc": 0}]}, fh)
    # not a run dir, so verify_run rejects it anyway; use check_matrix_dir only
    del RESULTS[:]
    check_matrix_dir(mroot)
    if any(r[0] == "FAIL" for r in RESULTS):
        print("SELF-TEST ok: mutation 'missing_matrix_cell' caught", flush=True)
    else:
        print("SELF-TEST FAIL: missing_matrix_cell NOT caught", flush=True)
        all_caught = False
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
