#!/usr/bin/env python3
"""Versioned gate for a SIC performance summary.

Identity: sic-perf-binding/1

The frozen baseline benchmark does not compare the effect run's library
with the library it actually loaded.  This script is the mandatory check
for both binaries.  It refuses a summary whose repo root, source hash, or
effect-run library does not match the files on disk.  It does not rewrite
the summary.

The recorded git head must equal HEAD.  After a later commit, pass
``--code-tested-sha`` equal to the recorded head.  That commit must be an
ancestor of HEAD, and ``gr-uwb/lib``, ``gr-uwb/include`` and
``benchmark_preamble_sic.cc`` must be unchanged.  The flag does not turn
the check off.

Exit 0 when the requested verdict holds.  Exit 1 otherwise.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys

VERIFIER_ID = "sic-perf-binding/1"

SWITCHES = (
    "UWB_SIC_REFINE_DIAG",
    "UWB_SIC_COL_CACHE",
    "UWB_SIC_ROT_BASE_CACHE",
    "UWB_SIC_RECON_CACHE",
    "UWB_SIC_SELF_ENERGY",
    "UWB_SIC_INCR_RESID",
    "UWB_SIC_FUSED_SCORE",
)
# Identity of a core/GR benchmark.  A docs or pacer-only commit may follow
# the measurement; a change here must fail re-verification.
PERF_TREE_PATHS = (
    "gr-uwb/lib",
    "gr-uwb/include",
    "gr-uwb/apps/benchmark_preamble_sic.cc",
)

SWITCH_DEFAULTS = {
    "UWB_SIC_REFINE_DIAG": True,
    "UWB_SIC_COL_CACHE": True,
    "UWB_SIC_ROT_BASE_CACHE": True,
    "UWB_SIC_RECON_CACHE": True,
    "UWB_SIC_SELF_ENERGY": True,
    "UWB_SIC_INCR_RESID": False,
    "UWB_SIC_FUSED_SCORE": False,
}


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def same_path(a, b):
    if not a or not b:
        return False
    try:
        return os.path.samefile(a, b)
    except OSError:
        return os.path.realpath(a) == os.path.realpath(b)


def inside(child, parent):
    child_r = os.path.realpath(child)
    parent_r = os.path.realpath(parent)
    if child_r == parent_r:
        return True
    prefix = parent_r if parent_r.endswith(os.sep) else parent_r + os.sep
    return child_r.startswith(prefix)


def git_head(repo):
    proc = subprocess.run(
        ["git", "-C", repo, "rev-parse", "HEAD"],
        capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        return ""
    return proc.stdout.strip()


def git_dirty(repo):
    proc = subprocess.run(
        ["git", "-C", repo, "status", "--porcelain"],
        capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        return True
    return bool(proc.stdout.strip())


def git_commit(repo, rev):
    proc = subprocess.run(
        ["git", "-C", repo, "rev-parse", "--verify", rev + "^{commit}"],
        capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        return ""
    return proc.stdout.strip()


def git_is_ancestor(repo, older, newer):
    proc = subprocess.run(
        ["git", "-C", repo, "merge-base", "--is-ancestor", older, newer],
        capture_output=True, text=True, check=False)
    return proc.returncode == 0


def git_diff_names(repo, older, newer, paths):
    proc = subprocess.run(
        ["git", "-C", repo, "diff", "--name-only", older, newer, "--", *paths],
        capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        return None
    return [ln for ln in proc.stdout.splitlines() if ln.strip()]


def check_recorded_head(repo, recorded, code_tested):
    """Commit check for one archived summary.

    Returns ``(errors, info)``.  Exact equality with HEAD passes.  A later
    HEAD passes only when ``code_tested`` is the recorded commit, that
    commit is an ancestor of HEAD, and the performance sources are unchanged.
    """
    head = git_head(repo)
    info = {
        "git_head": head,
        "recorded_git_head": recorded or "",
        "code_tested_sha": "",
        "relation": "",
        "perf_sources_changed": [],
    }
    errors = []
    if not recorded:
        errors.append("provenance.git_head missing")
        return errors, info
    recorded_full = git_commit(repo, recorded)
    if recorded == head or (recorded_full and recorded_full == head):
        if code_tested:
            tested = git_commit(repo, code_tested)
            if not tested or tested != recorded_full:
                errors.append(
                    "code-tested SHA %s != recorded git_head %s" % (
                        code_tested, recorded_full or recorded))
        info["relation"] = "equal"
        info["code_tested_sha"] = recorded_full or recorded
        return errors, info
    if not code_tested:
        errors.append("provenance.git_head %s != %s" % (recorded, head))
        return errors, info
    tested = git_commit(repo, code_tested)
    info["code_tested_sha"] = tested
    if not tested or not recorded_full:
        errors.append("code-tested SHA or recorded git_head is not a commit")
        return errors, info
    if tested != recorded_full:
        errors.append("code-tested SHA %s != recorded git_head %s" % (
            tested, recorded_full))
        return errors, info
    if not git_is_ancestor(repo, tested, head):
        errors.append("code-tested SHA %s is not an ancestor of HEAD %s" % (
            tested, head))
        return errors, info
    changed = git_diff_names(repo, tested, head, PERF_TREE_PATHS)
    if changed is None:
        errors.append("git diff of performance sources failed")
        return errors, info
    info["relation"] = "ancestor"
    info["perf_sources_changed"] = changed
    if changed:
        errors.append(
            "performance sources changed since code-tested SHA: %s" % (
                ", ".join(changed),))
    return errors, info


def switch_from_env(env):
    """Archive one process environment.  Leading 0/n/N/f/F disables."""
    out = []
    for name in SWITCHES:
        raw = env.get(name, "")
        if raw == "":
            out.append({
                "name": name,
                "value": SWITCH_DEFAULTS[name],
                "default": SWITCH_DEFAULTS[name],
                "source": "default",
                "raw": "",
            })
        else:
            disabled = raw[0] in "0nNfF"
            out.append({
                "name": name,
                "value": not disabled,
                "default": SWITCH_DEFAULTS[name],
                "source": "environment",
                "raw": raw,
            })
    return out


def cmake_flag(build_dir, key):
    path = os.path.join(build_dir, "CMakeCache.txt")
    if not os.path.isfile(path):
        return ""
    prefix = key + ":"
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line.startswith(prefix):
                continue
            eq = line.find("=")
            if eq < 0:
                continue
            return line[eq + 1:].rstrip("\n")
    return ""


def effect_lib(effect_dir):
    man_path = os.path.join(effect_dir, "run_manifest.json")
    man = load_json(man_path)
    prov = man.get("provenance") or {}
    return {
        "run_id": man.get("run_id"),
        "lib_path": prov.get("libgnuradio_uwb_path"),
        "lib_sha256": prov.get("libgnuradio_uwb_sha256"),
        "repo_root": prov.get("repo_root"),
        "git_head": prov.get("git_head"),
    }


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--summary", required=True)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--effect-run", required=True)
    ap.add_argument("--manifest", required=True,
                    help="machine-readable verdict written by this verifier")
    ap.add_argument("--expect-refused", action="store_true",
                    help="the effect run must belong to another library.  "
                         "The frozen baseline binary can still print "
                         "effect_verified true; this verifier refuses that "
                         "pairing itself.  Pass --require-binary-refused to "
                         "also demand the summary flag is false.")
    ap.add_argument("--require-binary-refused", action="store_true",
                    help="with --expect-refused, summary.effect_verified "
                         "must be false (candidate binary)")
    ap.add_argument("--require-clean", action="store_true")
    ap.add_argument("--code-tested-sha", default="",
                    help="recorded measurement commit.  Required when HEAD "
                         "has moved.  It must equal provenance.git_head, be "
                         "an ancestor of HEAD, and leave gr-uwb/lib, "
                         "gr-uwb/include and benchmark_preamble_sic.cc "
                         "unchanged.  Omitting it keeps the exact HEAD check.")
    ap.add_argument("--require-switches", action="store_true",
                    help="the summary itself must archive UWB_SIC_* switches")
    ap.add_argument("--env-json", default="",
                    help="optional sidecar of the process environment")
    args = ap.parse_args(argv)

    errors = []
    if args.require_binary_refused and not args.expect_refused:
        errors.append("--require-binary-refused needs --expect-refused")
    summary_sha = sha256_file(args.summary)
    summary = load_json(args.summary)
    prov = summary.get("provenance") or {}
    binding = summary.get("effect_binding") or {}
    repo = os.path.realpath(args.repo)
    source = os.path.join(repo, "gr-uwb", "apps", "benchmark_preamble_sic.cc")
    if not os.path.isfile(source):
        errors.append("benchmark source missing: %s" % source)
        source_sha = ""
    else:
        source_sha = sha256_file(source)

    recorded_root = prov.get("repo_root") or ""
    if not recorded_root or not same_path(recorded_root, repo):
        errors.append("provenance.repo_root %s != %s" % (recorded_root, repo))
    recorded_source = prov.get("benchmark_source_sha256") or ""
    if recorded_source != source_sha:
        errors.append("benchmark_source_sha256 %s != file %s" % (
            recorded_source, source_sha))

    lib_path = prov.get("libgnuradio_uwb_path") or ""
    lib_sha = prov.get("libgnuradio_uwb_sha256") or ""
    build_dir = prov.get("build_dir") or ""
    if not lib_path or not os.path.isfile(lib_path):
        errors.append("loaded library path missing: %s" % lib_path)
        disk_lib = ""
    else:
        disk_lib = sha256_file(lib_path)
        if disk_lib != lib_sha:
            errors.append("library sha on disk != provenance")
    if build_dir and lib_path and not inside(lib_path, build_dir):
        errors.append("loaded library is outside build_dir")
    if build_dir and not inside(build_dir, repo):
        errors.append("build_dir is outside repo")

    head_errors, head_info = check_recorded_head(
        repo, prov.get("git_head") or "", args.code_tested_sha)
    errors.extend(head_errors)
    head = head_info["git_head"]
    dirty = git_dirty(repo)
    if args.require_clean and dirty:
        errors.append("repo is dirty")

    try:
        eff = effect_lib(args.effect_run)
    except (OSError, json.JSONDecodeError) as exc:
        errors.append("effect run unreadable: %s" % exc)
        eff = {}
    eff_sha = eff.get("lib_sha256") or ""
    libs_match = bool(eff_sha) and eff_sha == lib_sha
    verified = bool(summary.get("effect_verified"))
    if args.expect_refused:
        if libs_match:
            errors.append("expected a foreign library, but the hashes match")
        if args.require_binary_refused and verified:
            errors.append("effect_verified stayed true for a foreign effect run")
    else:
        if not verified:
            errors.append("effect_verified is false")
        if not libs_match:
            errors.append(
                "effect lib %s != loaded lib %s" % (eff_sha, lib_sha))

    switches = prov.get("opt_switches")
    if args.require_switches:
        if not isinstance(switches, list) or len(switches) != len(SWITCHES):
            errors.append("opt_switches missing or incomplete")
        else:
            names = [s.get("name") for s in switches]
            if names != list(SWITCHES):
                errors.append("opt_switches names/order != %s" % (names,))

    env_switches = None
    if args.env_json:
        env = load_json(args.env_json)
        if not isinstance(env, dict):
            errors.append("--env-json is not an object")
            env = {}
        env_switches = switch_from_env(env)
        if isinstance(switches, list):
            for got, want in zip(switches, env_switches):
                if bool(got.get("value")) != bool(want["value"]):
                    errors.append("switch %s summary=%s env=%s" % (
                        want["name"], got.get("value"), want["value"]))

    geom = prov.get("geometry") if isinstance(prov.get("geometry"), dict) else {}
    manifest = {
        "verifier": VERIFIER_ID,
        "verdict": "pass" if not errors else "fail",
        "expect_refused": bool(args.expect_refused),
        "errors": errors,
        "summary": args.summary,
        "summary_sha256": summary_sha,
        "repo": repo,
        "git_head": head,
        "git_head_relation": head_info["relation"],
        "code_tested_sha": head_info["code_tested_sha"],
        "recorded_git_head": head_info["recorded_git_head"],
        "perf_sources_changed": head_info["perf_sources_changed"],
        "git_dirty": dirty,
        "source_sha256": source_sha,
        "build_dir": build_dir,
        "lib_path": lib_path,
        "lib_sha256": lib_sha,
        "effect_run": os.path.realpath(args.effect_run),
        "effect_lib_sha256": eff_sha,
        "effect_lib_path": eff.get("lib_path"),
        "libs_match": libs_match,
        "effect_verified": verified,
        "binary_claimed_verified_on_foreign_lib": bool(
            args.expect_refused and verified and not libs_match),
        "formal_perf": summary.get("formal_perf"),
        "opt_switches": switches if isinstance(switches, list) else env_switches,
        "env_switches": env_switches,
        "compile": prov.get("compile") or {
            "build_type": cmake_flag(build_dir, "CMAKE_BUILD_TYPE"),
            "cxx_flags": cmake_flag(build_dir, "CMAKE_CXX_FLAGS"),
            "cxx_flags_release": cmake_flag(build_dir, "CMAKE_CXX_FLAGS_RELEASE"),
        },
        "geometry": geom,
        "window_len": summary.get("n_work"),
        "k_paths": (summary.get("resolved_config") or {}).get("k_paths"),
        "fit_reps": (summary.get("resolved_config") or {}).get("fit_reps"),
        "cold_core_us": geom.get("cold_core_us"),
        "prepare_us": geom.get("prepare_us"),
        "binding_lib_match_field": binding.get("lib_match"),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.manifest)), exist_ok=True)
    with open(args.manifest, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    if errors:
        for err in errors:
            print("FAIL", err, file=sys.stderr)
        print("BINDING FAIL", VERIFIER_ID)
        return 1
    print("BINDING PASS", VERIFIER_ID)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
