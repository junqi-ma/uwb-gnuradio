#!/usr/bin/env python3
"""Reject old libraries from the integrated worktree's trusted root.

Copies one historical B cell into a temporary directory and edits only that
copy. The frozen evidence and the original worktree are not modified.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import verify_s2b as V  # noqa: E402


MAIN = Path("/home/junqima/workspace/uwb-gnuradio")
OLD_B_BUILD = (MAIN / ".worktrees/sic-payload-collision/gr-uwb/build-sic-payload")
OLD_B_LIB = OLD_B_BUILD / "lib/libgnuradio-uwb.so.1.0.0.0"
OLD_B_BIND = (OLD_B_BUILD / "python/uwb/bindings"
              / "uwb_python.cpython-310-x86_64-linux-gnu.so")
PARENT_LIB = MAIN / "gr-uwb/build-sic/lib/libgnuradio-uwb.so.1.0.0.0"
SOURCE_CELL = (OLD_B_BUILD / "results/s2b_matrix_rectification/cells"
               / "b1_center_isr10_snr30")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def statuses(run_dir: Path, trusted_root: Path, trusted_build: Path | None = None) -> dict:
    V.verify_run(str(run_dir), trusted_root=str(trusted_root),
                 trusted_build=None if trusted_build is None else str(trusted_build))
    return {name: kind for kind, name, _detail in V.RESULTS}


def edit_manifest(run_dir: Path, fn) -> None:
    path = run_dir / "run_manifest.json"
    data = json.loads(path.read_text())
    fn(data)
    path.write_text(json.dumps(data))


def require(label: str, got: str, want: str, problems: list) -> None:
    if got != want:
        problems.append(f"{label}: got {got}, want {want}")
    else:
        print(f"PASS {label}")


def main() -> int:
    repo = Path(__file__).resolve().parents[2]
    problems = []
    if not SOURCE_CELL.is_dir() or not OLD_B_LIB.is_file():
        print("missing historical B cell or library", file=sys.stderr)
        return 2
    old_sha = sha256(OLD_B_LIB)
    with tempfile.TemporaryDirectory(prefix="s2-trust-") as tmp:
        base = Path(tmp)
        joint = base / "joint-old-b"
        shutil.copytree(SOURCE_CELL, joint, symlinks=True)
        edit_manifest(joint, lambda m: V._set_build_lib(
            m, str(OLD_B_BUILD), str(OLD_B_LIB), old_sha))
        st = statuses(joint, repo)
        require("joint old-B build_dir", st.get("manifest_build_dir_trusted"), "FAIL", problems)
        require("joint old-B library", st.get("manifest_library_strict"), "FAIL", problems)

        if PARENT_LIB.is_file():
            parent = base / "joint-parent"
            parent_build = PARENT_LIB.parents[1]
            shutil.copytree(SOURCE_CELL, parent, symlinks=True)
            edit_manifest(parent, lambda m: V._set_build_lib(
                m, str(parent_build), str(PARENT_LIB.resolve()), sha256(PARENT_LIB)))
            st = statuses(parent, repo)
            require("joint parent build_dir", st.get("manifest_build_dir_trusted"), "FAIL", problems)
            require("joint parent library", st.get("manifest_library_strict"), "FAIL", problems)
        else:
            print("NOTE parent build-sic library absent; parent case not executed")

        escape_root = base / "escape-root"
        escape_build = escape_root / "gr-uwb/build-sic-payload"
        libdir = escape_build / "lib"
        libdir.mkdir(parents=True)
        link = libdir / "libgnuradio-uwb.so.1.0.0.0"
        link.symlink_to(OLD_B_LIB)
        escaped = base / "symlink-escape"
        shutil.copytree(SOURCE_CELL, escaped, symlinks=True)
        edit_manifest(escaped, lambda m: V._set_build_lib(
            m, str(escape_build), str(link), old_sha))
        st = statuses(escaped, escape_root, escape_build)
        require("symlink escape build_dir stays trusted",
                st.get("manifest_build_dir_trusted"), "PASS", problems)
        require("symlink escape library", st.get("manifest_library_strict"), "FAIL", problems)

        if OLD_B_BIND.is_file():
            bind = base / "bindings-outside"
            shutil.copytree(SOURCE_CELL, bind, symlinks=True)
            edit_manifest(bind, lambda m: V._set_bindings(
                m, str(OLD_B_BIND), sha256(OLD_B_BIND)))
            st = statuses(bind, repo)
            require("outside bindings with correct hash",
                    st.get("manifest_bindings_strict"), "FAIL", problems)
        else:
            problems.append("historical B binding missing")

        tamper = base / "bindings-hash"
        shutil.copytree(SOURCE_CELL, tamper, symlinks=True)
        edit_manifest(tamper, lambda m: V._set_bindings(
            m, m["provenance"]["bindings"]["path"], "0" * 64))
        st = statuses(tamper, repo)
        require("bindings hash tamper", st.get("manifest_bindings_strict"), "FAIL", problems)

    if problems:
        print("TRUST CHECK FAIL")
        for item in problems:
            print(" -", item)
        return 1
    print("TRUST CHECK PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
