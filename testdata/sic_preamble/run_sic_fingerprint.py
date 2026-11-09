#!/usr/bin/env python3
"""Rebuild and compare the two-process SIC fingerprint.

The driver source lives next to this script.  Each process is compiled
against one worktree's header and linked only to that worktree's
libgnuradio-uwb.  Scratch binaries and outputs stay under --output, so
deleting /tmp does not remove the source and a later run still rebuilds.

Exit 0 when every identity file matches.  timings.txt is recorded and not
compared.  Fused scoring and incremental residual stay off.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys

IDENTITY_SKIP = {"timings.txt"}
SWITCHES_OFF = {
    "UWB_SIC_FUSED_SCORE": "0",
    "UWB_SIC_INCR_RESID": "0",
}


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git_head(repo):
    proc = subprocess.run(
        ["git", "-C", repo, "rev-parse", "HEAD"],
        capture_output=True, text=True, check=False)
    return proc.stdout.strip() if proc.returncode == 0 else ""


def run(cmd, env, cwd):
    proc = subprocess.run(cmd, env=env, cwd=cwd, text=True)
    return proc.returncode


def compile_one(src, root, build, exe, errors):
    lib = os.path.join(build, "lib", "libgnuradio-uwb.so")
    if not os.path.isfile(lib):
        errors.append("missing library %s" % lib)
        return ""
    cmd = [
        "g++", "-O3", "-DNDEBUG", "-std=c++17",
        "-I", os.path.join(root, "gr-uwb", "include"),
        src,
        "-L", os.path.join(build, "lib"),
        "-Wl,-rpath," + os.path.join(build, "lib"),
        "-lgnuradio-uwb",
        "-o", exe,
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        errors.append("compile failed for %s\n%s\n%s" % (
            exe, proc.stdout, proc.stderr))
        return ""
    return " ".join(cmd)


def launch(exe, outdir, extra_env):
    os.makedirs(outdir, exist_ok=True)
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    env.update(SWITCHES_OFF)
    env.update(extra_env)
    proc = subprocess.run([exe, outdir], env=env, capture_output=True, text=True)
    return proc.returncode, proc.stdout, proc.stderr, env


def identity_files(directory):
    names = []
    for name in sorted(os.listdir(directory)):
        path = os.path.join(directory, name)
        if os.path.isfile(path) and name not in IDENTITY_SKIP:
            names.append(name)
    return names


def compare_dirs(left, right, errors, label):
    a = identity_files(left)
    b = identity_files(right)
    if a != b:
        errors.append("%s file list %s != %s" % (label, a, b))
        return
    for name in a:
        pa = os.path.join(left, name)
        pb = os.path.join(right, name)
        with open(pa, "rb") as fa, open(pb, "rb") as fb:
            if fa.read() != fb.read():
                errors.append("%s byte mismatch: %s" % (label, name))


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--candidate-root", required=True)
    ap.add_argument("--baseline-root", required=True)
    ap.add_argument("--candidate-build", required=True)
    ap.add_argument("--baseline-build", required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args(argv)

    src = os.path.join(here, "sic_fp.cc")
    errors = []
    if not os.path.isfile(src):
        print("FAIL missing", src, file=sys.stderr)
        return 1
    if os.path.isdir(args.output):
        print("FAIL output exists, refusing to overwrite", args.output,
              file=sys.stderr)
        return 1
    os.makedirs(args.output)
    cand_exe = os.path.join(args.output, "sic_fp_cand")
    base_exe = os.path.join(args.output, "sic_fp_base")
    cand_cmd = compile_one(src, args.candidate_root, args.candidate_build,
                           cand_exe, errors)
    base_cmd = compile_one(src, args.baseline_root, args.baseline_build,
                           base_exe, errors)
    launches = []
    if not errors:
        for tag, exe, extra in (
            ("cand", cand_exe, {}),
            ("base", base_exe, {}),
            ("cand_self_off", cand_exe, {"UWB_SIC_SELF_ENERGY": "0"}),
        ):
            outdir = os.path.join(args.output, tag)
            rc, stdout, stderr, env = launch(exe, outdir, extra)
            launches.append({
                "tag": tag,
                "exe": exe,
                "outdir": outdir,
                "exit_code": rc,
                "self_energy": env.get("UWB_SIC_SELF_ENERGY", ""),
                "fused": env.get("UWB_SIC_FUSED_SCORE"),
                "incremental": env.get("UWB_SIC_INCR_RESID"),
            })
            if stdout:
                sys.stdout.write(stdout)
            if stderr:
                sys.stderr.write(stderr)
            if rc != 0:
                errors.append("%s exited %s" % (tag, rc))
    if not errors:
        compare_dirs(os.path.join(args.output, "cand"),
                     os.path.join(args.output, "base"),
                     errors, "candidate-vs-baseline")
        compare_dirs(os.path.join(args.output, "cand"),
                     os.path.join(args.output, "cand_self_off"),
                     errors, "candidate-self-energy-on-vs-off")

    files = {}
    cand_dir = os.path.join(args.output, "cand")
    if os.path.isdir(cand_dir):
        for name in sorted(os.listdir(cand_dir)):
            path = os.path.join(cand_dir, name)
            if os.path.isfile(path):
                files[name] = sha256_file(path)

    result = {
        "schema": "sic-perf-fingerprint/1",
        "verdict": "pass" if not errors else "fail",
        "errors": errors,
        "source": src,
        "source_sha256": sha256_file(src) if os.path.isfile(src) else "",
        "candidate_root": os.path.realpath(args.candidate_root),
        "baseline_root": os.path.realpath(args.baseline_root),
        "candidate_git_head": git_head(args.candidate_root),
        "baseline_git_head": git_head(args.baseline_root),
        "candidate_lib_sha256": sha256_file(os.path.join(
            args.candidate_build, "lib", "libgnuradio-uwb.so"))
        if os.path.isfile(os.path.join(
            args.candidate_build, "lib", "libgnuradio-uwb.so")) else "",
        "baseline_lib_sha256": sha256_file(os.path.join(
            args.baseline_build, "lib", "libgnuradio-uwb.so"))
        if os.path.isfile(os.path.join(
            args.baseline_build, "lib", "libgnuradio-uwb.so")) else "",
        "candidate_compile": cand_cmd,
        "baseline_compile": base_cmd,
        "candidate_exe_sha256": sha256_file(cand_exe) if os.path.isfile(cand_exe) else "",
        "baseline_exe_sha256": sha256_file(base_exe) if os.path.isfile(base_exe) else "",
        "launches": launches,
        "candidate_files_sha256": files,
        "identity_skip": sorted(IDENTITY_SKIP),
        "scenes": [
            "frac_cfo25k", "k1_int", "k4_multi", "cfo15k", "short6",
            "frac_off", "r01_boundary", "random_cplx", "chunk_apply",
            "model_switch",
        ],
        "switches_forced_off": SWITCHES_OFF,
        "note": "Two processes.  Byte compare of every file except timings.txt. "
                "Model fields are text lines in meta.txt, not a struct dump.",
    }
    out_path = os.path.join(args.output, "result.json")
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, sort_keys=True)
        f.write("\n")
    if errors:
        for err in errors:
            print("FAIL", err, file=sys.stderr)
        print("FINGERPRINT FAIL")
        return 1
    print("FINGERPRINT PASS", out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
