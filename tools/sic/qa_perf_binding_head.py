#!/usr/bin/env python3
"""QA for archived performance-summary re-verification.

A later docs or pacer commit must not fail a summary whose recorded head is
named with --code-tested-sha, when that commit is an ancestor and the SIC
library, headers, and benchmark source are unchanged. Dropping the commit
check, naming a different commit, or changing those sources still fails.
This test uses a temporary git repository. It does not read measurement
artifacts.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import verify_sic_perf_binding as binding  # noqa: E402


def _git(repo, *args):
    proc = subprocess.run(
        ["git", "-C", repo, *args],
        capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError("%s\n%s" % (" ".join(args), proc.stderr))
    return proc.stdout.strip()


def _commit(repo, path, text, message):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    _git(repo, "add", "--", os.path.relpath(path, repo))
    _git(repo, "-c", "user.email=qa@local", "-c", "user.name=qa",
         "commit", "-m", message)
    return _git(repo, "rev-parse", "HEAD")


def main():
    fails = 0

    def check(name, cond):
        nonlocal fails
        print("%-8s %s" % ("PASS" if cond else "FAIL", name), flush=True)
        if not cond:
            fails += 1

    repo = tempfile.mkdtemp(prefix="sic-bind-qa-")
    try:
        _git(repo, "init", "-q")
        _git(repo, "config", "user.email", "qa@local")
        _git(repo, "config", "user.name", "qa")
        base = _commit(
            repo, os.path.join(repo, "gr-uwb", "lib", "a.cc"),
            "int a;\n", "lib")
        _commit(repo, os.path.join(repo, "gr-uwb", "include", "a.h"),
                "int a;\n", "header")
        measured = _commit(
            repo,
            os.path.join(repo, "gr-uwb", "apps", "benchmark_preamble_sic.cc"),
            "int bench;\n", "bench")
        docs = _commit(repo, os.path.join(repo, "docs", "note.md"),
                       "note\n", "docs only")
        head = _git(repo, "rev-parse", "HEAD")

        errors, info = binding.check_recorded_head(repo, head, "")
        check("equal head", errors == [] and info["relation"] == "equal")

        errors, info = binding.check_recorded_head(repo, measured, "")
        check("moved head without sha fails",
              len(errors) == 1 and "provenance.git_head" in errors[0])

        errors, info = binding.check_recorded_head(repo, measured, measured)
        check("docs descendant passes",
              errors == [] and info["relation"] == "ancestor"
              and info["perf_sources_changed"] == [])

        errors, _info = binding.check_recorded_head(repo, measured, docs)
        check("sha must equal recorded head",
              any("!=" in err for err in errors))

        errors, _info = binding.check_recorded_head(repo, measured, "deadbeef")
        check("unknown sha fails",
              any("not a commit" in err for err in errors))

        changed = _commit(
            repo, os.path.join(repo, "gr-uwb", "lib", "a.cc"),
            "int a;\nint b;\n", "lib change")
        errors, info = binding.check_recorded_head(repo, measured, measured)
        check("lib change fails",
              info["perf_sources_changed"]
              and any("performance sources changed" in err for err in errors))
        check("changed head is the lib commit",
              binding.git_commit(repo, "HEAD") == changed)

        errors, info = binding.check_recorded_head(repo, changed, "")
        check("new equal head passes",
              errors == [] and info["relation"] == "equal")

        errors, _info = binding.check_recorded_head(repo, "", measured)
        check("missing recorded head fails",
              any("missing" in err for err in errors))
    finally:
        shutil.rmtree(repo, ignore_errors=True)
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
