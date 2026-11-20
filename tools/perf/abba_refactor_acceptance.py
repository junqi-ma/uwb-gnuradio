#!/usr/bin/env python3
"""Phase 4 ABBA acceptance runner for the openUWB architecture refactor.

Compares a baseline build tree (Phase 0 commit 0A) against the current
refactor HEAD on the same machine, with the same CMake options and ISA.

The three measurements are the ones the Phase 4 brief requires:

  kernel_avx512  bench_demod_sic_kernels (reference AVX2 arm and AVX-512 arm)
  steady_avx2    benchmark_sc16_pipeline_steady, fixed feeder worker/queue
  replay_avx2    openuwb_rx (baseline: uwb_dw3000_live) replaying one capture

Every case is run in ABBA order (A B B A ...) on a pinned CPU set, strictly
serially: the script never starts two children at once, and it refuses to be
mixed with other load.  Raw stdout, the benchmark JSON and a summary with
medians are written out so the numbers in the acceptance report can be
re-derived.

Usage
-----
  python3 tools/perf/abba_refactor_acceptance.py \
      --baseline /tmp/opencode/uwb-baseline \
      --out /tmp/opencode/abba-phase4

  # one case at a time (still serial)
  python3 tools/perf/abba_refactor_acceptance.py ... --case steady_avx2

Notes
-----
- Set OPENUWB_ABBA_CPUS to choose the isolated CPUs (default "0").
- The baseline tree is expected to be a Phase 0 worktree built as
  gr-uwb/build-native (AVX2) and gr-uwb/build-avx512 (AVX-512).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import statistics
import subprocess
import sys
import time
from pathlib import Path

CASES = ("kernel_avx512", "steady_avx2", "replay_avx2")

KERNEL_ROW = re.compile(
    r"^\s{2}(?P<name>.+?)\s{2,}(?P<avx2>[0-9.]+)\s+(?P<cand>[0-9.]+)\s+"
    r"(?P<delta>[+-][0-9.]+)%"
)
FINAL_FIELD = re.compile(r"(\w+)=([0-9.]+)")


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def git_head(root: Path) -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def parse_kernel(text: str) -> dict:
    rows = {}
    section = ""
    for line in text.splitlines():
        stripped = line.strip()
        if re.match(r"^K[0-9]", stripped):
            section = stripped.split("(")[0].strip()
        m = KERNEL_ROW.match(line)
        if m:
            key = f"{section}: {m.group('name').strip()}"
            rows[f"{key} | avx2_ns"] = float(m.group("avx2"))
            rows[f"{key} | cand_ns"] = float(m.group("cand"))
    return rows


def parse_final(text: str) -> dict:
    """FINAL has the counters; SUMMARY_PARTIAL has the per-packet cost stats."""
    merged = {}
    for line in text.splitlines():
        if line.startswith("SUMMARY_PARTIAL "):
            merged.update({k: float(v) for k, v in FINAL_FIELD.findall(line)})
        elif line.startswith("FINAL "):
            merged.update({k: float(v) for k, v in FINAL_FIELD.findall(line)})
    return merged


def parse_steady(path: Path) -> dict:
    data = json.loads(path.read_text())
    meta = data["meta"]
    ledger = data.get("ledger", {})
    return {
        "e2e_us_mean": meta["e2e_us_mean"],
        "e2e_us_p95": meta["e2e_us_p95"],
        "steady_window_us": meta["steady_window_us"],
        "sic_us_mean": meta["sic_us_mean"],
        "resample_us_mean": meta["resample_us_mean"],
        "measure_windows": meta["measure_windows_actual"],
        "dropped_admission": ledger.get("dropped_admission", 0),
    }


def build_cases(args, fixture: Path) -> dict:
    baseline = Path(args.baseline).resolve()
    current = Path(args.current).resolve()
    steady_common = [
        "--file", str(fixture),
        "--native-rate", "737280000",
        "--work-rate", "998400000",
        "--warmup-windows", "20",
        "--measure-windows", "300",
        "--dsp-workers", "1",
        "--pool-size", "16",
        "--flow-in-flight", "24",
    ]
    return {
        "kernel_avx512": {
            "A": [str(baseline / "build-avx512/apps/bench_demod_sic_kernels"),
                  "--rounds", "5"],
            "B": [str(current / "build/native-avx512/tests/bench_demod_sic_kernels"),
                  "--rounds", "5"],
            "parser": "kernel",
            "cpus": "0",
        },
        "steady_avx2": {
            "A": [str(baseline / "build-native/apps/benchmark_sc16_pipeline_steady"),
                  *steady_common, "--label", "baseline", "--json", "{json}"],
            "B": [str(current / "build/native-avx2/tests/benchmark_sc16_pipeline_steady"),
                  *steady_common, "--label", "current", "--json", "{json}"],
            "parser": "steady",
            "cpus": "0,1",
        },
        "replay_avx2": {
            "A": [str(baseline / "build-native/apps/uwb_dw3000_live"),
                  "--replay", str(fixture), "--summary", "1000"],
            "B": [str(current / "build/native-avx2/apps/openuwb_rx"),
                  "--replay", str(fixture), "--summary", "1000", "--workers", "2"],
            "parser": "replay",
            "cpus": "0,1",
        },
    }


def run_once(cmd, env, prefix, cpus, out_dir, tag, parser, json_path=None):
    real_cmd = list(cmd)
    if json_path is not None:
        real_cmd = [str(json_path) if c == "{json}" else c for c in real_cmd]
    if prefix:
        real_cmd = prefix + real_cmd
    if cpus:
        real_cmd = ["taskset", "-c", cpus] + real_cmd
    started = time.monotonic()
    proc = subprocess.run(real_cmd, capture_output=True, text=True, env=env)
    wall = time.monotonic() - started
    (out_dir / f"{tag}.stdout").write_text(proc.stdout)
    (out_dir / f"{tag}.stderr").write_text(proc.stderr)
    if proc.returncode != 0:
        raise SystemExit(
            f"run failed ({proc.returncode}): {' '.join(real_cmd)}\n"
            + proc.stderr[-2000:])
    if parser == "kernel":
        metrics = parse_kernel(proc.stdout)
    elif parser == "steady":
        metrics = parse_steady(json_path)
    else:
        metrics = parse_final(proc.stdout)
    return {"tag": tag, "wall_s": round(wall, 4), "metrics": metrics}


def summarize(runs: dict, keys, keep=None) -> dict:
    out = {}
    a_values = [r["metrics"] for r in runs["A"]]
    b_values = [r["metrics"] for r in runs["B"]]
    if keep is not None:
        a_values = [m for m in a_values if keep(m)]
        b_values = [m for m in b_values if keep(m)]
    for key in keys:
        a = [m[key] for m in a_values if key in m]
        b = [m[key] for m in b_values if key in m]
        if not a or not b:
            continue
        med_a, med_b = statistics.median(a), statistics.median(b)
        out[key] = {
            "baseline_median": round(med_a, 4),
            "current_median": round(med_b, 4),
            "delta_pct": round((med_b - med_a) / med_a * 100.0, 3) if med_a else None,
            "baseline_values": a,
            "current_values": b,
        }
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baseline", required=True,
                    help="Phase 0 worktree root (built native + avx512)")
    ap.add_argument("--current", default=".",
                    help="current refactor worktree root (default: .)")
    ap.add_argument("--fixture", default="testdata/x410_fixed_tone_live_20261115/rx_1ms_1s.sc16")
    ap.add_argument("--out", required=True)
    ap.add_argument("--case", choices=CASES, action="append",
                    help="run only this case (repeatable; default: all)")
    ap.add_argument("--cycles", type=int, default=2,
                    help="ABBA cycles per case (default 2 -> A B B A A B B A)")
    args = ap.parse_args()

    fixture = Path(args.fixture).resolve()
    if not fixture.is_file():
        raise SystemExit(f"fixture not found: {fixture}")

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env.pop("LD_LIBRARY_PATH", None)
    env["UWB_TESTDATA_DIR"] = str(Path(args.current).resolve() / "testdata")
    cpus = os.environ.get("OPENUWB_ABBA_CPUS", "0")

    cases = build_cases(args, fixture)
    selected = args.case or list(CASES)
    report = {
        "schema": "openuwb-abba-acceptance/1",
        "baseline_commit": git_head(Path(args.baseline)),
        "current_commit": git_head(Path(args.current)),
        "cpu": cpu_model(),
        "nproc": os.cpu_count(),
        "affinity": cpus,
        "cgroup_cpus": Path("/sys/fs/cgroup/cpuset.cpus.effective").read_text().strip()
            if Path("/sys/fs/cgroup/cpuset.cpus.effective").exists() else "n/a",
        "isa": "native-avx2 (steady/replay), native-avx512 (kernel bench)",
        "compiler": subprocess.run(
            [os.environ.get("CXX", "c++"), "--version"],
            capture_output=True, text=True).stdout.splitlines()[0],
        "fixture": str(fixture),
        "fixture_sha256": sha256(fixture),
        "cycles": args.cycles,
        "cases": {},
    }

    for name in selected:
        spec = cases[name]
        runs = {"A": [], "B": []}
        order = []
        for _ in range(args.cycles):
            for tag in ("A", "B", "B", "A"):
                json_path = out_dir / f"{name}.{tag}.{len(runs[tag])}.json"
                result = run_once(
                    spec[tag], env, prefix=None, cpus=spec.get("cpus", cpus),
                    out_dir=out_dir, tag=f"{name}.{tag}.{len(runs[tag])}",
                    parser=spec["parser"],
                    json_path=None if spec["parser"] != "steady" else json_path)
                runs[tag].append(result)
                order.append(tag)
        if spec["parser"] == "kernel":
            keys = sorted(runs["A"][0]["metrics"].keys())
        elif spec["parser"] == "steady":
            keys = ["e2e_us_mean", "e2e_us_p95", "steady_window_us",
                    "sic_us_mean", "resample_us_mean", "measure_windows"]
        else:
            keys = ["completed", "fcs_ok", "compute_mean", "compute_p95",
                    "resample_mean", "demod_mean",
                    "raw_samples", "raw_drop_samples"]
        case = {
            "cpus": spec.get("cpus", cpus),
            "order": order,
            "runs": runs,
            "summary": summarize(runs, keys),
        }
        if spec["parser"] == "replay":
            # A real-time replay run can be descheduled by the host: the
            # reader then drops input samples before admission, which shows up
            # as raw_drop_samples.  Those runs measure the machine, not the
            # pipeline, so they are reported separately instead of being
            # silently averaged in.
            case["drop_limit_samples"] = int(0.10 * runs["A"][0]["metrics"]["raw_samples"])
            case["summary_clean"] = summarize(
                runs, keys,
                keep=lambda m: m.get("raw_drop_samples", 0) <= case["drop_limit_samples"])
        report["cases"][name] = case
        print(f"\n=== {name} (order {' '.join(order)}) ===")
        summaries = [("all", report["cases"][name]["summary"])]
        if "summary_clean" in report["cases"][name]:
            summaries.append(("clean", report["cases"][name]["summary_clean"]))
        for label, summary in summaries:
            for key, s in summary.items():
                delta = s["delta_pct"]
                delta_text = "n/a" if delta is None else f"{delta:+.2f}%"
                print(f"  [{label:5s}] {key:26s} A={s['baseline_median']:>12.3f}  "
                      f"B={s['current_median']:>12.3f}  delta={delta_text}")

    (out_dir / "abba.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {out_dir / 'abba.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
