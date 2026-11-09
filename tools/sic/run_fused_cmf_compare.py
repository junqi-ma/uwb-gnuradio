#!/usr/bin/env python3
"""Paired feasibility test for the fused payload CMF + PN-despread path.

The driver runs the SAME binary twice: the accepted fast-RX baseline (four
explicit RX options) and that baseline plus ``--rx-fused-payload-cmf``.  It
does not duplicate any DSP.  Formal runs take the exclusive
``/tmp/uwb-sic-benchmark.lock`` and remove ``LD_LIBRARY_PATH``.

Correctness and performance are reported separately.  A missing/empty run, a
source-window mismatch, or a fused attempt that was never exercised fails
correctness; a slower or non-improving result is reported as a valid
"correct but not faster" conclusion and does not masquerade as PASS.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
LOCK = Path("/tmp/uwb-sic-benchmark.lock")
DEFAULT_CELLS = ["payload_base", "payload_four_frac", "payload_cfo_m50k",
                 "payload_cfo_p50k", "payload_len127", "payload_snr8"]
TIMING_CELLS = ["payload_base", "payload_four_frac", "payload_len127"]
BASE_ARGS = ["--rx-fast-sfd", "--rx-fast-cfo", "--rx-seed-refine-tail",
             "--rx-softchip-on-demand"]
FUSED_ARGS = BASE_ARGS + ["--rx-fused-payload-cmf"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def git_output(*args: str) -> str:
    return subprocess.run(["git", "-C", str(ROOT), *args], text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          check=True).stdout.strip()


def clean_env() -> dict:
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    env["OMP_NUM_THREADS"] = "1"
    env["OPENBLAS_NUM_THREADS"] = "1"
    return env


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               allow_nan=False) + "\n")


def check_library(binary: Path, build: Path, env: dict) -> dict:
    completed = subprocess.run(["ldd", str(binary)], env=env, text=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, check=True)
    libraries = []
    for line in completed.stdout.splitlines():
        if "libgnuradio-uwb" in line and "=>" in line:
            candidate = Path(line.split("=>", 1)[1].strip().split()[0]).resolve()
            if not candidate.is_relative_to(build.resolve()):
                raise RuntimeError(
                    f"runner loaded uwb library outside build: {candidate}")
            libraries.append({"path": str(candidate), "sha256": sha256(candidate)})
    if not libraries:
        raise RuntimeError("ldd found no uwb library")
    return {"ldd": completed.stdout, "libraries": libraries}


def load_rows(run: Path) -> list[dict]:
    path = run / "windows.jsonl"
    if not path.is_file():
        raise RuntimeError(f"missing {path}")
    rows = [json.loads(line) for line in path.read_text().splitlines()
            if line.strip()]
    if not rows:
        raise RuntimeError(f"runner produced no rows: {run}")
    return rows


def run_variant(runner: Path, case: Path, out: Path, env: dict, repeats: int,
                warmup: int, extra: list[str], reconstruction: str) -> dict:
    if out.exists():
        raise RuntimeError(f"refusing to overwrite {out}")
    out.mkdir(parents=True)
    command = [str(runner), "--case", str(case), "--out", str(out),
               "--mode", "full", "--reconstruction", reconstruction,
               "--repeat", str(repeats), "--warmup", str(warmup)] + extra
    with (out / "runner.log").open("w") as log:
        started = utc_now()
        completed = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                   stdout=log, stderr=subprocess.STDOUT)
    record = {"argv": command, "returncode": completed.returncode,
              "started_utc": started, "finished_utc": utc_now(),
              "repeats": repeats, "warmup": warmup, "extra_args": extra}
    write_json(out / "command.json", record)
    if completed.returncode != 0:
        raise RuntimeError(f"runner failed ({completed.returncode}): {out}")
    rows = load_rows(out)
    if len(rows) != repeats:
        raise RuntimeError(f"{out}: expected {repeats} rows, got {len(rows)}")
    return record


def p95(values: list[float]) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1,
                max(0, int(round(0.95 * (len(ordered) - 1)))))
    return ordered[index]


def stats(values: list[float]) -> dict:
    return {"mean": statistics.fmean(values) if values else 0.0,
            "p95": p95(values), "n": len(values)}


def row_metric(row: dict) -> float:
    # Old: payload CMF extension + payload BPM/FEC.  New: fused template +
    # four-candidate correlation + payload FEC (soft_extend is zero).  This is
    # the like-for-like "payload path" cost; it does not include SYNC/SFD/PHR.
    return (float(row["rx_diag"].get("soft_extend_us", 0)) +
            float(row["comm_stage_us"]["payload"]))


def summarize(rows: list[dict]) -> dict:
    accepted = [r for r in rows if not r.get("rejected")]
    comm = [float(r["times_us"]["comm_demod"]) for r in accepted]
    sic = [float(r["times_us"]["sic_total"]) for r in accepted]
    payload = [float(r["comm_stage_us"]["payload"]) for r in accepted]
    payload_path = [row_metric(r) for r in accepted]
    cir = [float(r["comm_stage_us"]["cir"]) for r in accepted]
    extend = [float(r["rx_diag"].get("soft_extend_us", 0)) for r in accepted]
    fused_tpl = [float(r["rx_diag"].get("fused_template_us", 0))
                 for r in accepted]
    fused_corr = [float(r["rx_diag"].get("fused_correlate_us", 0))
                  for r in accepted]
    chips = [float(r["rx_diag"].get("softchip_final", 0)) for r in accepted]
    return {
        "comm_demod": stats(comm),
        "sic_total": stats(sic),
        "stage_cir": stats(cir),
        "stage_payload": stats(payload),
        "payload_path": stats(payload_path),
        "soft_extend": stats(extend),
        "fused_template": stats(fused_tpl),
        "fused_correlate": stats(fused_corr),
        "chips": stats(chips),
        "fused_attempts": sum(int(r["rx_diag"].get("fused_attempts", 0))
                              for r in accepted),
        "fused_fallbacks": sum(int(r["rx_diag"].get("fused_fallbacks", 0))
                               for r in accepted),
        "fused_symbols": sum(int(r["rx_diag"].get("fused_symbols", 0))
                             for r in accepted),
        "softchip_fallbacks": sum(int(r["rx_diag"].get("softchip_fallbacks", 0))
                                  for r in accepted),
        "comm_fcs": sum(bool(r["comm_fcs"]) for r in accepted),
        "sensing_fcs": sum(bool(r["sensing_fcs"]) for r in accepted),
    }


def functional_compare(old_rows: list[dict], new_rows: list[dict],
                       cell: str) -> dict:
    fields = ("comm_fcs", "comm_status", "comm_psdu_hex", "comm_phr_length",
              "sensing_fcs", "sensing_status", "sensing_psdu_hex",
              "committed", "reason")
    old_by_source = {int(r["source_window"]): r for r in old_rows}
    new_by_source = {int(r["source_window"]): r for r in new_rows}
    problems = []
    if len(old_by_source) != len(old_rows):
        problems.append("old source_window not unique")
    if len(new_by_source) != len(new_rows):
        problems.append("new source_window not unique")
    if set(old_by_source) != set(new_by_source):
        problems.append("source-window set mismatch")
    mismatches = []
    for source in sorted(set(old_by_source) & set(new_by_source)):
        for field in fields:
            if old_by_source[source].get(field) != new_by_source[source].get(field):
                mismatches.append({"source_window": source, "field": field,
                                   "old": old_by_source[source].get(field),
                                   "new": new_by_source[source].get(field)})
    fused_attempts = sum(int(r["rx_diag"].get("fused_attempts", 0))
                         for r in new_rows)
    fused_fallbacks = sum(int(r["rx_diag"].get("fused_fallbacks", 0))
                          for r in new_rows)
    old_fused_attempts = sum(int(r["rx_diag"].get("fused_attempts", 0))
                             for r in old_rows)
    if fused_attempts == 0:
        problems.append("fused kernel never attempted")
    if old_fused_attempts != 0:
        problems.append("baseline unexpectedly attempted fused kernel")
    if fused_fallbacks != 0:
        problems.append("fused fallback on a nominal cell")
    for row in new_rows:
        phr_len = int(row.get("comm_phr_length", 0))
        if phr_len <= 0:
            problems.append(f"source {row.get('source_window')} has no PHR length")
            continue
        psdu_bits = phr_len * 8
        expected = psdu_bits + 48 * ((psdu_bits + 329) // 330)
        got = int(row["rx_diag"].get("fused_symbols", 0))
        if got != expected:
            problems.append(
                f"source {row.get('source_window')} fused_symbols {got} != {expected}")
        if int(row["rx_diag"].get("softchip_final", 0)) != \
                int(row["rx_diag"].get("softchip_initial", -1)):
            problems.append(
                f"source {row.get('source_window')} generated payload soft chips")
        if int(row["rx_diag"].get("fused_template_us", 0)) + \
                int(row["rx_diag"].get("fused_correlate_us", 0)) > \
                int(row["times_us"]["comm_demod"]):
            problems.append(
                f"source {row.get('source_window')} fused sub-timers exceed comm")
    return {
        "cell": cell,
        "passed": not mismatches and not problems,
        "problems": problems,
        "windows": len(old_by_source),
        "mismatches": mismatches,
        "fused_attempts": fused_attempts,
        "fused_fallbacks": fused_fallbacks,
        "old_comm_fcs": sum(bool(r["comm_fcs"]) for r in old_rows),
        "new_comm_fcs": sum(bool(r["comm_fcs"]) for r in new_rows),
        "old_sensing_fcs": sum(bool(r["sensing_fcs"]) for r in old_rows),
        "new_sensing_fcs": sum(bool(r["sensing_fcs"]) for r in new_rows),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build",
                        default="gr-uwb/build-sic-fullpacket-fused-cmf")
    parser.add_argument("--runner",
                        default="apps/benchmark_fullpacket_compare_fast_rx")
    parser.add_argument("--inputs",
                        default="gr-uwb/build-sic-fullpacket/results/inputs16")
    parser.add_argument("--out", required=True)
    parser.add_argument("--mode", choices=["functional", "timing", "both"],
                        default="both")
    parser.add_argument("--cells", nargs="+", default=None)
    parser.add_argument("--timing-cells", nargs="+", default=None)
    parser.add_argument("--observations", type=int, default=128)
    parser.add_argument("--warmup", type=int, default=16)
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--reconstruction", default="discrete_cir",
                        choices=["full", "discrete_cir", "payload_cached"])
    parser.add_argument("--payload-target-ratio", type=float, default=0.80)
    parser.add_argument("--regression-ratio", type=float, default=1.05)
    args = parser.parse_args()

    build = (ROOT / args.build).resolve()
    runner = (build / args.runner).resolve()
    inputs = (ROOT / args.inputs).resolve()
    out = (ROOT / args.out).resolve()
    if out.exists():
        raise RuntimeError(f"output exists; choose a new run: {out}")
    if not runner.is_file():
        raise RuntimeError(f"missing runner: {runner}")
    cells = args.cells or DEFAULT_CELLS
    timing_cells = args.timing_cells or [c for c in cells
                                        if c in TIMING_CELLS]
    for cell in cells:
        if not (inputs / cell / "public.json").is_file():
            raise RuntimeError(f"missing cell inputs: {inputs / cell}")
    out.mkdir(parents=True)
    env = clean_env()
    library = check_library(runner, build, env)
    flags_file = build / "apps/CMakeFiles" / (runner.name + ".dir") / "flags.make"
    status = git_output("status", "--porcelain=v1", "--untracked-files=normal")
    provenance = {
        "schema": "fused-cmf-compare/1",
        "created_utc": utc_now(),
        "git_head": git_output("rev-parse", "HEAD"),
        "git_dirty": bool(status),
        "git_status": status.splitlines(),
        "build_dir": str(build),
        "runner": {"path": str(runner), "sha256": sha256(runner)},
        "compile_flags": {"path": str(flags_file),
                          "sha256": sha256(flags_file),
                          "content": flags_file.read_text()},
        "libraries": library["libraries"],
        "ldd": library["ldd"],
        "cpu": next((line.split(":", 1)[1].strip()
                     for line in Path("/proc/cpuinfo").read_text().splitlines()
                     if line.startswith("model name")), "unknown"),
        "platform": platform.platform(),
        "base_args": BASE_ARGS,
        "fused_args": FUSED_ARGS,
        "reconstruction": args.reconstruction,
        "lock_path": str(LOCK),
        "ld_library_path_removed": True,
        "full_wait_policy": "whole_public_context",
        "sources": {str(path.relative_to(ROOT)): sha256(path)
                    for path in [
                        ROOT / "gr-uwb/apps/benchmark_fullpacket_compare.cc",
                        ROOT / "gr-uwb/apps/fullpacket_rx_pipeline.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_demod_core.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_demod_result.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_phy_profile.h",
                        ROOT / "gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h",
                        ROOT / "gr-uwb/lib/qa_uwb_rx_fused_cmf.cc",
                        Path(__file__).resolve(),
                    ] if path.is_file()},
    }
    write_json(out / "provenance.json", provenance)

    summary = {"schema": "fused-cmf-compare/1", "created_utc": utc_now(),
               "mode": args.mode, "cells": {}, "functional": [],
               "timing": [], "provenance": str(out / "provenance.json")}

    with LOCK.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.mode in ("functional", "both"):
            for cell in cells:
                case = inputs / cell
                windows = [p for p in (case / "windows").iterdir() if p.is_dir()]
                old_dir = out / "functional" / cell / "old"
                new_dir = out / "functional" / cell / "new"
                run_variant(runner, case, old_dir, env, len(windows), 0,
                            BASE_ARGS, args.reconstruction)
                run_variant(runner, case, new_dir, env, len(windows), 0,
                            FUSED_ARGS, args.reconstruction)
                result = functional_compare(load_rows(old_dir),
                                            load_rows(new_dir), cell)
                summary["functional"].append(result)
                verdict = "PASS" if result["passed"] else "FAIL"
                print(f"correctness {cell}: {verdict} "
                      f"windows={result['windows']} "
                      f"mismatches={len(result['mismatches'])} "
                      f"fused={result['fused_attempts']} "
                      f"fallbacks={result['fused_fallbacks']}")
        if args.mode in ("timing", "both"):
            for cell in timing_cells:
                case = inputs / cell
                runs = []
                order = []
                for round_index in range(max(1, args.rounds)):
                    pair = ([("old", BASE_ARGS), ("new", FUSED_ARGS)]
                            if round_index % 2 == 0 else
                            [("new", FUSED_ARGS), ("old", BASE_ARGS)])
                    order.extend(pair)
                for index, (variant, extra) in enumerate(order):
                    run_dir = out / "timing" / cell / f"r{index + 1}_{variant}"
                    run_variant(runner, case, run_dir, env, args.observations,
                                args.warmup, extra, args.reconstruction)
                    rows = load_rows(run_dir)
                    runs.append({"round": index + 1, "variant": variant,
                                 "dir": str(run_dir), "summary": summarize(rows),
                                 "rows": rows})
                old_rows = [row for run in runs if run["variant"] == "old"
                            for row in run["rows"] if not row.get("rejected")]
                new_rows = [row for run in runs if run["variant"] == "new"
                            for row in run["rows"] if not row.get("rejected")]
                old_summary = summarize(old_rows)
                new_summary = summarize(new_rows)

                def ratio(metric: str) -> float:
                    base = old_summary[metric]["mean"]
                    return new_summary[metric]["mean"] / base if base else 0.0

                def p95_ratio(metric: str) -> float:
                    base = old_summary[metric]["p95"]
                    return new_summary[metric]["p95"] / base if base else 0.0

                result = {
                    "cell": cell,
                    "old": old_summary,
                    "new": new_summary,
                    "payload_path_mean_ratio": ratio("payload_path"),
                    "payload_path_p95_ratio": p95_ratio("payload_path"),
                    "payload_stage_mean_ratio": ratio("stage_payload"),
                    "comm_mean_ratio": ratio("comm_demod"),
                    "comm_p95_ratio": p95_ratio("comm_demod"),
                    "sic_mean_ratio": ratio("sic_total"),
                    "sic_p95_ratio": p95_ratio("sic_total"),
                    "payload_target_ratio_max": args.payload_target_ratio,
                    "regression_mean_ratio_max": args.regression_ratio,
                    "payload_target_met":
                        ratio("payload_path") <= args.payload_target_ratio,
                    "no_regression":
                        ratio("comm_demod") <= args.regression_ratio and
                        ratio("sic_total") <= args.regression_ratio,
                    "rounds": [{k: run[k] for k in ("round", "variant", "summary")}
                               for run in runs],
                }
                result["performance_target_met"] = (
                    result["payload_target_met"] and result["no_regression"])
                summary["timing"].append(result)
                print(
                    f"timing {cell}: payload_path "
                    f"{old_summary['payload_path']['mean']:.1f} -> "
                    f"{new_summary['payload_path']['mean']:.1f} us "
                    f"(ratio {result['payload_path_mean_ratio']:.3f}), "
                    f"comm ratio {result['comm_mean_ratio']:.3f}, "
                    f"sic ratio {result['sic_mean_ratio']:.3f}")
    write_json(out / "summary.json", summary)

    with (out / "timing.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell", "variant", "metric", "mean_us", "p95_us",
                         "n", "mean_ratio_vs_old", "p95_ratio_vs_old"])
        metrics = ("comm_demod", "sic_total", "stage_cir", "stage_payload",
                   "payload_path", "soft_extend", "fused_template",
                   "fused_correlate", "chips")
        for item in summary["timing"]:
            for metric in metrics:
                old = item["old"][metric]
                new = item["new"][metric]
                mean_ratio = (new["mean"] / old["mean"]) if old["mean"] else 0.0
                p95_ratio = (new["p95"] / old["p95"]) if old["p95"] else 0.0
                writer.writerow([item["cell"], "old", metric,
                                 f"{old['mean']:.6f}", f"{old['p95']:.6f}",
                                 old["n"], "1.000000", "1.000000"])
                writer.writerow([item["cell"], "new", metric,
                                 f"{new['mean']:.6f}", f"{new['p95']:.6f}",
                                 new["n"], f"{mean_ratio:.6f}",
                                 f"{p95_ratio:.6f}"])
    with (out / "functional.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cell", "passed", "windows", "mismatches",
                         "fused_attempts", "fused_fallbacks",
                         "old_comm_fcs", "new_comm_fcs",
                         "old_sensing_fcs", "new_sensing_fcs"])
        for item in summary["functional"]:
            writer.writerow([item["cell"], item["passed"], item["windows"],
                             len(item["mismatches"]), item["fused_attempts"],
                             item["fused_fallbacks"], item["old_comm_fcs"],
                             item["new_comm_fcs"], item["old_sensing_fcs"],
                             item["new_sensing_fcs"]])

    functional_ok = all(item["passed"] for item in summary["functional"])
    performance_ok = all(item["performance_target_met"]
                         for item in summary["timing"]) if summary["timing"] else False
    if summary["functional"]:
        print(f"correctness={'PASS' if functional_ok else 'FAIL'}")
    else:
        print("correctness=NOT_RUN")
    if summary["timing"]:
        print(f"performance_target={'MET' if performance_ok else 'NOT_MET'}")
    else:
        print("performance_target=NOT_RUN")
    return 0 if (functional_ok if summary["functional"] else True) else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 - CLI boundary
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
