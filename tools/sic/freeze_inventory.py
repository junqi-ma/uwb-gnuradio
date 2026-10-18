#!/usr/bin/env python3
"""Audit an S2 evidence freeze: hash, copy, verify, and summarize.

This tool does not regenerate experiments. It records existing files and
copies them. Hash output is an integrity record; the copy and the git bundle
are the backup. Ignored IQ is not stored in git.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import stat
import subprocess
import sys
import time
from pathlib import Path


CHUNK = 8 * 1024 * 1024


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            block = handle.read(CHUNK)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def rel_to(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def walk_files(root: Path):
    """Yield (path, dirent) for every file and symlink under root, not following links."""
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        dirnames.sort()
        filenames.sort()
        current = Path(dirpath)
        for name in filenames:
            path = current / name
            yield path


def metadata_fingerprint(root: Path) -> dict:
    rows = []
    count = 0
    total = 0
    for path in walk_files(root):
        count += 1
        st = path.lstat()
        size = st.st_size
        total += size
        kind = "symlink" if stat.S_ISLNK(st.st_mode) else "file"
        rows.append(f"{kind}\t{size}\t{st.st_mtime_ns}\t{path.relative_to(root).as_posix()}")
    rows.sort()
    digest = hashlib.sha256(("\n".join(rows) + ("\n" if rows else "")).encode()).hexdigest()
    return {
        "root": str(root),
        "file_count": count,
        "total_bytes": total,
        "metadata_sha256": digest,
        "content_backed_up": False,
    }


def inventory_tree(root: Path, source_commit: str, dirty: bool, group: str) -> list[dict]:
    records = []
    for path in walk_files(root):
        st = path.lstat()
        is_link = stat.S_ISLNK(st.st_mode)
        record = {
            "group": group,
            "abs_path": str(path.resolve()) if not is_link else str(path.absolute()),
            "logical_path": str(path.absolute()),
            "rel_path": path.relative_to(root).as_posix(),
            "size": st.st_size,
            "sha256": None,
            "source_commit": source_commit,
            "dirty": dirty,
            "is_symlink": is_link,
            "symlink_target": os.readlink(path) if is_link else None,
            "resolved_path": None,
        }
        if is_link:
            try:
                resolved = path.resolve(strict=True)
                record["resolved_path"] = str(resolved)
                if resolved.is_file():
                    record["sha256"] = sha256_file(resolved)
                    record["resolved_size"] = resolved.stat().st_size
            except OSError as exc:
                record["resolve_error"] = str(exc)
        elif stat.S_ISREG(st.st_mode):
            record["sha256"] = sha256_file(path)
        else:
            record["skipped_kind"] = oct(st.st_mode)
        records.append(record)
    return records


def copy_tree(src: Path, dst: Path) -> None:
    if dst.exists():
        raise SystemExit(f"refusing to overwrite existing copy: {dst}")
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src, dst, symlinks=True, copy_function=shutil.copy2)


def verify_copy(records: list[dict], src_root: Path, dst_root: Path) -> list[str]:
    problems = []
    seen = set()
    for record in records:
        rel = record["rel_path"]
        seen.add(rel)
        src = src_root / rel
        dst = dst_root / rel
        if not dst.exists() and not dst.is_symlink():
            problems.append(f"missing copy: {rel}")
            continue
        src_link = src.is_symlink()
        dst_link = dst.is_symlink()
        if src_link != dst_link or src_link != record["is_symlink"]:
            problems.append(f"symlink mismatch: {rel}")
            continue
        if src_link:
            if os.readlink(src) != os.readlink(dst):
                problems.append(f"symlink target mismatch: {rel}")
            continue
        if sha256_file(dst) != record["sha256"]:
            problems.append(f"hash mismatch: {rel}")
        if dst.stat().st_size != record["size"]:
            problems.append(f"size mismatch: {rel}")
    for path in walk_files(dst_root):
        rel = path.relative_to(dst_root).as_posix()
        if rel not in seen:
            problems.append(f"unexpected copy: {rel}")
    return problems


def content_digest(records: list[dict]) -> str:
    lines = []
    for record in records:
        lines.append(
            f"{record['sha256'] or '-'}\t{record['size']}\t"
            f"{int(record['is_symlink'])}\t{record['rel_path']}"
        )
    lines.sort()
    return sha256_bytes(("\n".join(lines) + ("\n" if lines else "")).encode())


def copy_resolved_file(src: Path, dst: Path) -> dict:
    """Copy the real file behind src, recording the original symlink chain."""
    chain = []
    current = src
    seen = set()
    while current.is_symlink():
        target = os.readlink(current)
        chain.append({"path": str(current.absolute()), "target": target})
        nxt = (current.parent / target).absolute()
        key = str(nxt)
        if key in seen:
            raise SystemExit(f"symlink loop at {current}")
        seen.add(key)
        current = nxt
    resolved = current.resolve(strict=True)
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        raise SystemExit(f"refusing to overwrite {dst}")
    shutil.copy2(resolved, dst)
    digest = sha256_file(dst)
    if digest != sha256_file(resolved):
        raise SystemExit(f"resolved copy mismatch: {src}")
    return {
        "original_path": str(src.absolute()),
        "resolved_path": str(resolved),
        "copy_path": str(dst),
        "size": dst.stat().st_size,
        "sha256": digest,
        "symlink_chain": chain,
    }


def collect_runtime_strings(root: Path) -> list[str]:
    found = []
    for path in walk_files(root):
        if path.suffix != ".json" or path.is_symlink():
            continue
        name = path.name
        if "manifest" not in name and name not in {"matrix.json", "index.json"}:
            continue
        try:
            data = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        stack = [data]
        while stack:
            item = stack.pop()
            if isinstance(item, dict):
                stack.extend(item.values())
            elif isinstance(item, list):
                stack.extend(item)
            elif isinstance(item, str) and (
                "libgnuradio-uwb" in item or "uwb_python" in item or item.endswith(".so")
            ):
                found.append(item)
    return sorted(set(found))


def git_show_sha256(repo: Path, rev: str, path: str) -> dict:
    proc = subprocess.run(
        ["git", "show", f"{rev}:{path}"],
        cwd=repo,
        check=False,
        capture_output=True,
    )
    if proc.returncode != 0:
        return {
            "rev": rev,
            "path": path,
            "present": False,
            "error": proc.stderr.decode()[:400],
        }
    return {
        "rev": rev,
        "path": path,
        "present": True,
        "size": len(proc.stdout),
        "sha256": sha256_bytes(proc.stdout),
    }


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def recheck(inventory_path: Path) -> int:
    payload = json.loads(inventory_path.read_text())
    problems = []
    for record in payload["files"]:
        path = Path(record["logical_path"])
        if record["is_symlink"]:
            if not path.is_symlink() or os.readlink(path) != record["symlink_target"]:
                problems.append(f"symlink changed: {path}")
            continue
        if not path.is_file():
            problems.append(f"missing original: {path}")
            continue
        if path.stat().st_size != record["size"] or sha256_file(path) != record["sha256"]:
            problems.append(f"content changed: {path}")
    digest = content_digest(payload["files"])
    print(json.dumps({
        "files": len(payload["files"]),
        "content_digest": digest,
        "expected_digest": payload["content_digest"],
        "match": digest == payload["content_digest"] and not problems,
        "problems": problems[:20],
        "problem_count": len(problems),
    }, indent=2))
    return 0 if digest == payload["content_digest"] and not problems else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)
    run = sub.add_parser("run")
    run.add_argument("--repo", type=Path, required=True)
    run.add_argument("--dest", type=Path, required=True)
    run.add_argument("--a-root", type=Path, required=True)
    run.add_argument("--b-root", type=Path, required=True)
    run.add_argument("--a-commit", required=True)
    run.add_argument("--b-commit", required=True)
    check = sub.add_parser("recheck")
    check.add_argument("--inventory", type=Path, required=True)
    return parser


def worktree_dirty(path: Path) -> str:
    proc = subprocess.run(
        ["git", "status", "--porcelain=v1"],
        cwd=path,
        check=True,
        capture_output=True,
        text=True,
    )
    return proc.stdout


def cmd_run(args: argparse.Namespace) -> int:
    dest = args.dest
    if dest.exists() and any(dest.iterdir()):
        raise SystemExit(f"destination exists and is not empty: {dest}")
    dest.mkdir(parents=True, exist_ok=True)
    started = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    a_status = worktree_dirty(args.a_root)
    b_status = worktree_dirty(args.b_root)
    if a_status or b_status:
        raise SystemExit(
            "A/B worktree is dirty; freeze the exact accepted trees only.\n"
            f"A:\n{a_status}\nB:\n{b_status}"
        )
    a_groups = [
        ("a-smoke-rectification", args.a_root / "gr-uwb/build-sic-accum/results/s2a/smoke_rectification"),
        ("a-calibration-rectification", args.a_root / "gr-uwb/build-sic-accum/results/s2a/calibration_rectification"),
        ("a-acceptance-rectification", args.a_root / "gr-uwb/build-sic-accum/results/s2a/acceptance_rectification"),
        ("a-bypass-rectification", args.a_root / "gr-uwb/build-sic-accum/results/s2a/bypass_rectification"),
        ("a-budget-rectification", args.a_root / "gr-uwb/build-sic-accum/results/s2a/budget_rectification"),
        ("a-qa-logs", args.a_root / "gr-uwb/build-sic-accum/results/s2a/qa"),
    ]
    b_groups = [
        ("b-matrix-rectification", args.b_root / "gr-uwb/build-sic-payload/results/s2b_matrix_rectification"),
        ("b-matrix-ci-fix-derived", args.b_root / "gr-uwb/build-sic-payload/results/s2b_matrix_ci_fix"),
    ]
    excluded = [
        args.a_root / "gr-uwb/build-sic-accum/results/s2a/smoke",
        args.a_root / "gr-uwb/build-sic-accum/results/s2a/calibration",
        args.a_root / "gr-uwb/build-sic-accum/results/s2a/acceptance",
        args.a_root / "gr-uwb/build-sic-accum/results/s2a/bypass",
        args.b_root / "gr-uwb/build-sic-payload/results/cal_scan",
        args.b_root / "gr-uwb/build-sic-payload/results/qa_neg_off_center",
        args.b_root / "gr-uwb/build-sic-payload/results/qa_neg_on_no_overlap",
        args.b_root / "gr-uwb/build-sic-payload/results/qa_neg_wrong_code_center",
        args.b_root / "gr-uwb/build-sic-payload/results/s2b_b0_smoke",
        args.b_root / "gr-uwb/build-sic-payload/results/s2b_b1_center_isr10_snr30",
        args.b_root / "gr-uwb/build-sic-payload/results/s2b_matrix",
        args.b_root / "gr-uwb/build-sic-payload/results/index.json",
    ]
    files = []
    copy_problems = []
    for group, src in a_groups + b_groups:
        if not src.exists():
            raise SystemExit(f"required evidence missing: {src}")
        commit = args.a_commit if group.startswith("a-") else args.b_commit
        print(f"inventory {group}", flush=True)
        records = inventory_tree(src, commit, False, group)
        copy_dst = dest / "copies" / group
        print(f"copy {group} ({len(records)} entries)", flush=True)
        if src.is_file():
            copy_dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, copy_dst)
        else:
            copy_tree(src, copy_dst)
        print(f"verify {group}", flush=True)
        if src.is_dir():
            problems = verify_copy(records, src, copy_dst)
        else:
            problems = []
            if sha256_file(copy_dst) != records[0]["sha256"]:
                problems.append(f"hash mismatch: {group}")
        copy_problems.extend(f"{group}: {item}" for item in problems)
        for record in records:
            record["copy_rel"] = str(Path("copies") / group / record["rel_path"])
        files.extend(records)

    runtime = []
    runtime_specs = [
        ("a", args.a_commit, args.a_root / "gr-uwb/build-sic-accum"),
        ("b", args.b_commit, args.b_root / "gr-uwb/build-sic-payload"),
    ]
    referenced = {
        "a": collect_runtime_strings(args.a_root / "gr-uwb/build-sic-accum/results/s2a"),
        "b": collect_runtime_strings(args.b_root / "gr-uwb/build-sic-payload/results/s2b_matrix_rectification"),
    }
    for label, commit, build in runtime_specs:
        lib_link = build / "lib/libgnuradio-uwb.so.1.0.0"
        lib_real = build / "lib/libgnuradio-uwb.so.1.0.0.0"
        binding = build / "python/uwb/bindings/uwb_python.cpython-310-x86_64-linux-gnu.so"
        test_binding = build / "test_modules/gnuradio/uwb/uwb_python.cpython-310-x86_64-linux-gnu.so"
        out = dest / "copies" / f"runtime-{label}"
        out.mkdir(parents=True)
        saved = [
            copy_resolved_file(lib_real, out / "libgnuradio-uwb.so.1.0.0.0"),
            copy_resolved_file(binding, out / binding.name),
        ]
        if test_binding.exists():
            saved.append(copy_resolved_file(test_binding, out / ("test_modules_" + test_binding.name)))
        ldd = subprocess.run(["ldd", str(lib_real.resolve())], capture_output=True, text=True, check=False)
        (out / "ldd.txt").write_text(ldd.stdout + ldd.stderr)
        cache = build / "CMakeCache.txt"
        shutil.copy2(cache, out / "CMakeCache.txt")
        link_info = {
            "lib_so": os.readlink(build / "lib/libgnuradio-uwb.so"),
            "lib_so_1": os.readlink(lib_link),
            "lib_link_path": str(lib_link.absolute()),
            "lib_real_path": str(lib_real.resolve()),
        }
        write_json(out / "link-map.json", link_info)
        runtime.append({
            "label": label,
            "source_commit": commit,
            "dirty": False,
            "build_dir": str(build.resolve()),
            "saved": saved,
            "cmake_cache_sha256": sha256_file(out / "CMakeCache.txt"),
            "ldd_sha256": sha256_file(out / "ldd.txt"),
            "link_map": link_info,
            "manifest_path_strings": referenced[label],
        })

    source_dir = dest / "source"
    source_dir.mkdir()
    bundle = source_dir / "sic-s2-freeze-tags.bundle"
    subprocess.run(
        [
            "git", "bundle", "create", str(bundle),
            "sic-s2-base-20261018",
            "sic-s2a-accepted-20261018",
            "sic-s2b-accepted-20261018",
        ],
        cwd=args.repo,
        check=True,
    )
    verify = subprocess.run(
        ["git", "bundle", "verify", str(bundle)],
        cwd=args.repo,
        check=False,
        capture_output=True,
        text=True,
    )
    heads = subprocess.run(
        ["git", "bundle", "list-heads", str(bundle)],
        cwd=args.repo,
        check=True,
        capture_output=True,
        text=True,
    )
    (source_dir / "bundle-verify.txt").write_text(verify.stdout + verify.stderr)
    (source_dir / "bundle-heads.txt").write_text(heads.stdout)
    key_paths = [
        "gr-uwb/include/gnuradio/uwb/uwb_preamble_sic_core.h",
        "gr-uwb/lib/uwb_preamble_sic_core.cc",
        "gr-uwb/include/gnuradio/uwb/uwb_preamble_sic.h",
        "gr-uwb/lib/uwb_preamble_sic.cc",
        "tools/sic/sic_reference.py",
        "gr-uwb/lib/qa_uwb_preamble_sic_core.cc",
        "gr-uwb/lib/qa_uwb_preamble_sic.cc",
        "docs/sic/S1_八符号模型API.md",
        "UWB_Preamble_SIC_GNU_Radio_Design.md",
    ]
    a_docs = [
        "docs/sic/S2A_整改开发报告_2026-10-18.md",
        "docs/sic/S2A_二次复核补充意见_2026-10-18.md",
        "docs/sic/S2A_累加后CIR开发报告.md",
        "testdata/sic_accum/cells.json",
        "testdata/sic_accum/operator.json",
    ]
    b_docs = [
        "docs/sic/S2B_整改开发报告_2026-10-18.md",
        "docs/sic/S2B_二次复核补充意见_2026-10-18.md",
        "docs/sic/S2B_感知前导干扰通信Payload开发报告.md",
        "testdata/sic_payload/cases/stages.json",
    ]
    source_rows = []
    for rev in ("sic-s2-base-20261018", "sic-s2a-accepted-20261018", "sic-s2b-accepted-20261018"):
        for path in key_paths:
            source_rows.append(git_show_sha256(args.repo, rev, path))
    for path in a_docs:
        source_rows.append(git_show_sha256(args.repo, "sic-s2a-accepted-20261018", path))
    for path in b_docs:
        source_rows.append(git_show_sha256(args.repo, "sic-s2b-accepted-20261018", path))
    write_json(source_dir / "source-sha256.json", source_rows)

    env_lines = [f"started_utc={started}"]
    for cmd in (
        ["uname", "-a"],
        ["python3", "--version"],
        ["cmake", "--version"],
        ["c++", "--version"],
        ["git", "rev-parse", "HEAD"],
        ["git", "status", "--porcelain=v1"],
    ):
        proc = subprocess.run(cmd, cwd=args.repo, capture_output=True, text=True, check=False)
        env_lines.append(f"$ {' '.join(cmd)}")
        env_lines.append((proc.stdout or "") + (proc.stderr or ""))
    (source_dir / "env.txt").write_text("\n".join(env_lines))

    excluded_rows = []
    for path in excluded:
        if path.is_file():
            st = path.stat()
            excluded_rows.append({
                "root": str(path),
                "file_count": 1,
                "total_bytes": st.st_size,
                "metadata_sha256": sha256_bytes(
                    f"file\t{st.st_size}\t{st.st_mtime_ns}\t{path.name}\n".encode()
                ),
                "content_backed_up": False,
            })
        elif path.is_dir():
            print(f"metadata-only {path}", flush=True)
            excluded_rows.append(metadata_fingerprint(path))
        else:
            excluded_rows.append({"root": str(path), "missing": True, "content_backed_up": False})

    archive = dest / "archives" / "s2-evidence-copies.tar"
    archive.parent.mkdir()
    print("archive copies", flush=True)
    subprocess.run(
        ["tar", "-C", str(dest / "copies"), "-cf", str(archive), "."],
        check=True,
    )
    archive_sha = sha256_file(archive)
    digest = content_digest(files)
    payload = {
        "started_utc": started,
        "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "a_commit": args.a_commit,
        "b_commit": args.b_commit,
        "a_dirty": False,
        "b_dirty": False,
        "file_count": len(files),
        "total_bytes": sum(item["size"] for item in files),
        "content_digest": digest,
        "copy_problem_count": len(copy_problems),
        "copy_problems": copy_problems,
        "archive": {
            "path": str(archive),
            "size": archive.stat().st_size,
            "sha256": archive_sha,
        },
        "runtime": runtime,
        "excluded_not_content_backed_up": excluded_rows,
        "bundle": {
            "path": str(bundle),
            "size": bundle.stat().st_size,
            "sha256": sha256_file(bundle),
            "verify_returncode": verify.returncode,
            "heads": heads.stdout,
        },
        "files": files,
    }
    write_json(dest / "inventory" / "freeze-inventory.json", payload)
    summary = {key: payload[key] for key in payload if key != "files"}
    summary["inventory_json"] = str(dest / "inventory" / "freeze-inventory.json")
    summary["inventory_json_sha256"] = sha256_file(dest / "inventory" / "freeze-inventory.json")
    write_json(dest / "inventory" / "summary.json", summary)
    print(json.dumps({
        "file_count": payload["file_count"],
        "total_bytes": payload["total_bytes"],
        "content_digest": digest,
        "copy_problem_count": len(copy_problems),
        "archive_sha256": archive_sha,
        "bundle_sha256": payload["bundle"]["sha256"],
        "bundle_verify": verify.returncode,
    }, indent=2))
    return 0 if not copy_problems and verify.returncode == 0 else 1


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.cmd == "run":
        return cmd_run(args)
    if args.cmd == "recheck":
        return recheck(args.inventory)
    return 2


if __name__ == "__main__":
    sys.exit(main())
