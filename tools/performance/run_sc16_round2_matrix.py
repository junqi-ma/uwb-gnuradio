#!/usr/bin/env python3
"""ABBA paired benchmark driver for the 2026-11-11 SC16 round-2 optimisation.

Ownership: main agent.  This is the ONLY process allowed to execute the formal
performance matrix.  It serialises every run on /tmp/uwb-sic-benchmark.lock (by
passing --benchmark to the binary, which flocks it), enforces ABBA ordering and
records the raw JSON of every run so a reviewer can re-derive every number.

Protocol
--------
For each configuration the variants are run in A B B A order, repeated
`--pairs` times.  "A" is the control (instrumented baseline) and "B" the
candidate.  Each run produces one JSON file; the steady-state interval is the
[min rx_push_time, max commit_time] window of the k-th measurement region, as
reported by the binary itself (meta.steady_interval_s / meta.steady_window_us).

Nothing is ever averaged across a dropped/partial run silently: if a run's
meta reports ready_ok=false, ledger_invariants_ok=false or fewer measured
windows than requested, the row is flagged and excluded from the summary with
an explicit count.

Usage
-----
  run_sc16_round2_matrix.py --control <bin> --candidate <bin> \
      --tag control=baseline --tag candidate=A1 \
      --outdir docs/performance/data_sc16_round2/A1 \
      --pairs 5 --warmup 0 --measure 10000 \
      [--configs 56k-rt-1,56k-rt-2,...] [--dry-run]

Config names are "<window>-<profile>-n<workers>", e.g. 56k-rt-4.
"""

import argparse
import csv
import json
import os
import statistics
import subprocess
import sys
import time

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

WINDOWS = {
    # name: (input file, capture, pre, post, period, consumer)
    "4k": ("/tmp/opencode/bench/steady_4000.bin", "3200", "400", "400", "10000", "sensing"),
    "56k": ("/tmp/opencode/bench/steady_56000.bin", "56000", "0", "0", "56000", "sic"),
}
PROFILES = {
    "rt": "realtime_minorder",
    "q": "quality_minorder",
}
CPU_RX = "0"
CPU_DSP = {1: "2", 2: "2,4", 4: "2,4,6,8"}


def config_names():
    out = []
    for w in WINDOWS:
        for p in PROFILES:
            for n in CPU_DSP:
                out.append(f"{w}-{p}-n{n}")
    return out


def parse_config(name):
    window, profile, workers = name.split("-")
    workers = int(workers[1:])
    if window not in WINDOWS:
        raise SystemExit(f"unknown window {window!r} in config {name!r}")
    if profile not in PROFILES:
        raise SystemExit(f"unknown profile {profile!r} in config {name!r}")
    if workers not in CPU_DSP:
        raise SystemExit(f"unsupported worker count in config {name!r}")
    return window, profile, workers


def build_cmd(binary, name, args):
    window, profile, workers = parse_config(name)
    path, capture, pre, post, period, consumer = WINDOWS[window]
    if args.driver == "runner":
        # Historical cold-start protocol: the runner consumes the file once, so
        # the window count is bounded by the file length (100 x 56k, 400 x 4k).
        cmd = [
            binary,
            "--source", "replay",
            "--file", path,
            "--native-rate", "737280000",
            "--work-rate", "998400000",
            "--pre-guard", pre,
            "--capture", capture,
            "--post-guard", post,
            "--period", period,
            "--consumer", consumer,
            "--resampler-profile", PROFILES[profile],
            "--dsp-workers", str(workers),
            "--cpu-rx", CPU_RX,
            "--cpu-dsp", CPU_DSP[workers],
            # The runner has no feeder flow control, so the pool must be able to
            # hold EVERY window of the cold run; otherwise RX outruns the
            # workers and the run turns into a drop storm (observed: 131/400
            # drops with pool=128 on the 4k path).
            "--pool-size", str((400 if window == "4k" else args.cold_windows) + 16),
            "--max-windows", str(400 if window == "4k" else args.cold_windows),
            "--label", args.run_label,
            "--benchmark",
        ]
        if args.diag_rx:
            cmd.append("--diag-rx")
        if args.diag_commit:
            cmd.append("--diag-commit")
        return cmd
    cmd = [
        binary,
        "--file", path,
        "--native-rate", "737280000",
        "--work-rate", "998400000",
        "--pre-guard", pre,
        "--capture", capture,
        "--post-guard", post,
        "--period", period,
        "--consumer", consumer,
        "--resampler-profile", PROFILES[profile],
        "--dsp-workers", str(workers),
        "--cpu-rx", CPU_RX,
        "--cpu-dsp", CPU_DSP[workers],
        "--pool-size", "16",
        # 8 x workers gives the feeder enough slack that the gate is not the
        # bottleneck at 4k window sizes, while staying under the per-worker
        # pool depth (16) so the run is still zero-drop.
        "--flow-in-flight", str(8 * workers),
        "--warmup-windows", str(args.warmup),
        "--measure-windows", str(args.measure),
        "--label", args.run_label,
        "--benchmark",
    ]
    if args.experiment:
        cmd += ["--resampler-experiment", args.experiment]
    if args.diag_rx:
        cmd.append("--diag-rx")
    if args.diag_commit:
        cmd.append("--diag-commit")
    return cmd


def read_meta(path):
    with open(path) as f:
        d = json.load(f)
    return d.get("meta", {}), d.get("ledger", {}), d.get("windows", [])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--control", required=True, help="control binary")
    ap.add_argument("--candidate", required=True, help="candidate binary")
    ap.add_argument("--control-tag", default="control")
    ap.add_argument("--candidate-tag", default="candidate")
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--pairs", type=int, default=5)
    ap.add_argument("--warmup", type=int, default=0)
    ap.add_argument("--measure", type=int, default=10000)
    ap.add_argument("--configs", default="", help="comma separated subset; empty = all")
    ap.add_argument("--experiment", default="",
                    help="resampler experiment for the CANDIDATE variant (steady driver only)")
    ap.add_argument("--control-experiment", default="",
                    help="resampler experiment for the CONTROL variant (default: none)")
    ap.add_argument("--driver", choices=["steady", "runner"], default="steady",
                    help="steady = benchmark_sc16_pipeline_steady (warmup/measure region); "
                         "runner = uwb_uhd_sc16_pipeline_runner (historical cold start)")
    ap.add_argument("--cold-windows", type=int, default=100,
                    help="--max-windows for the runner driver (default 100)")
    ap.add_argument("--diag-rx", action="store_true")
    ap.add_argument("--diag-commit", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--timeout", type=int, default=1800)
    args = ap.parse_args()

    configs = [c.strip() for c in args.configs.split(",") if c.strip()] or config_names()
    for c in configs:
        parse_config(c)

    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%dT%H%M%S")
    raw_dir = os.path.join(args.outdir, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    variants = [(args.control_tag, args.control), (args.candidate_tag, args.candidate)]
    rows = []
    # ABBA: control, candidate, candidate, control
    order = [0, 1, 1, 0]

    for cfg_name in configs:
        for pair in range(args.pairs):
            for slot, v in enumerate(order):
                tag, binary = variants[v]
                run_id = f"{cfg_name}_p{pair}_{slot}_{tag}"
                json_path = os.path.join(raw_dir, f"{run_id}.json")
                argv = build_cmd(binary, cfg_name, argparse.Namespace(
                    warmup=args.warmup, measure=args.measure,
                    run_label=f"{cfg_name}/{tag}/{run_id}",
                    driver=args.driver, cold_windows=args.cold_windows,
                    experiment=(args.control_experiment if v == 0 else args.experiment),
                    diag_rx=args.diag_rx, diag_commit=args.diag_commit))
                argv += ["--json-out" if args.driver == "runner" else "--json", json_path]
                printable = " ".join(argv)
                print(f"[{stamp}] RUN {run_id}")
                if args.dry_run:
                    print("   " + printable)
                    continue
                t0 = time.time()
                proc = subprocess.run(argv, capture_output=True, text=True,
                                      timeout=args.timeout)
                dur = time.time() - t0
                log_path = os.path.join(raw_dir, f"{run_id}.log")
                with open(log_path, "w") as f:
                    f.write(proc.stdout)
                    f.write("\n=== STDERR ===\n")
                    f.write(proc.stderr)
                row = {
                    "config": cfg_name,
                    "variant": tag,
                    "pair": pair,
                    "slot": slot,
                    "run_id": run_id,
                    "returncode": proc.returncode,
                    "wall_s": dur,
                    "json": os.path.relpath(json_path, BASE_DIR),
                }
                if os.path.exists(json_path):
                    meta, ledger, windows = read_meta(json_path)
                    row.update({
                        "cold_total_s": meta.get("cold_total_s"),
                        "startup_ready_ms": meta.get("startup_ready_ms"),
                        "drain_tail_ms": meta.get("drain_tail_ms"),
                        "steady_interval_s": meta.get("steady_interval_s"),
                        "steady_window_us": meta.get("steady_window_us"),
                        "steady_window_rate": meta.get("steady_window_rate"),
                        "steady_window_sample_msps": meta.get("steady_window_sample_msps"),
                        "resample_us_mean": meta.get("resample_us_mean"),
                        "resample_us_p50": meta.get("resample_us_p50"),
                        "resample_us_p95": meta.get("resample_us_p95"),
                        "resample_us_p99": meta.get("resample_us_p99"),
                        "sic_us_mean": meta.get("sic_us_mean"),
                        "e2e_us_mean": meta.get("e2e_us_mean"),
                        "e2e_us_p50": meta.get("e2e_us_p50"),
                        "e2e_us_p95": meta.get("e2e_us_p95"),
                        "e2e_us_p99": meta.get("e2e_us_p99"),
                        "measure_windows_actual": meta.get("measure_windows_actual"),
                        "ready_ok": meta.get("ready_ok"),
                        "ledger_invariants_ok": meta.get("ledger_invariants_ok"),
                        "rx_read_msps": meta.get("rx_read_msps"),
                        "feeder_flow_waits": meta.get("feeder_flow_waits"),
                        "rx_read_us": meta.get("rx_read_us"),
                        "rx_assemble_us": meta.get("rx_assemble_us"),
                        "rx_slot_copy_us": meta.get("rx_slot_copy_us"),
                        "rx_dispatch_us": meta.get("rx_dispatch_us"),
                        "rx_stream_buf_resize_count": meta.get("rx_stream_buf_resize_count"),
                        "rx_zero_init_bytes": meta.get("rx_zero_init_bytes"),
                        "rx_compact_move_bytes": meta.get("rx_compact_move_bytes"),
                        "commit_lock_wait_us": meta.get("commit_lock_wait_us"),
                        "commit_reorder_us": meta.get("commit_reorder_us"),
                        "commit_callback_us": meta.get("commit_callback_us"),
                        "offered": ledger.get("offered_windows"),
                        "admitted": ledger.get("admitted_windows"),
                        "completed": ledger.get("completed_windows"),
                        "dropped": ledger.get("dropped_admission"),
                        "cancelled": ledger.get("cancelled_windows"),
                        "n_windows": len(windows),
                    })
                    need_windows = args.measure if args.driver == "steady" else 1
                    row["valid"] = (
                        proc.returncode == 0
                        and row.get("ready_ok") is True
                        and row.get("ledger_invariants_ok") is True
                        and (row.get("measure_windows_actual") or 0) >= need_windows
                    )
                else:
                    row["valid"] = False
                rows.append(row)
                print(f"   rc={row['returncode']} valid={row.get('valid')} "
                      f"steady_us={row.get('steady_window_us')} "
                      f"resample_mean={row.get('resample_us_mean')}")

    if args.dry_run:
        return 0

    csv_path = os.path.join(args.outdir, "runs.csv")
    fieldnames = sorted({k for r in rows for k in r})
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # ---- summary: paired ABBA difference per configuration -----------------
    summary_path = os.path.join(args.outdir, "summary.csv")
    summary = []
    for cfg_name in configs:
        cell = [r for r in rows if r["config"] == cfg_name]
        for tag in (args.control_tag, args.candidate_tag):
            v = [r for r in cell if r["variant"] == tag and r.get("valid")]
            if not v:
                summary.append({
                    "config": cfg_name, "variant": tag, "n_valid": 0,
                    "median_steady_window_us": None, "median_resample_us_mean": None,
                    "median_cold_total_s": None, "median_startup_ready_ms": None,
                    "median_drain_tail_ms": None, "median_fed_rate": None,
                })
                continue
            summary.append({
                "config": cfg_name,
                "variant": tag,
                "n_valid": len(v),
                "median_steady_window_us": statistics.median(r["steady_window_us"] for r in v),
                "median_resample_us_mean": statistics.median(r["resample_us_mean"] for r in v),
                "median_cold_total_s": statistics.median(r["cold_total_s"] for r in v),
                "median_startup_ready_ms": statistics.median(r["startup_ready_ms"] for r in v),
                "median_drain_tail_ms": statistics.median(r["drain_tail_ms"] for r in v),
                "median_fed_rate": statistics.median(r["steady_window_rate"] for r in v),
            })
    with open(summary_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        for r in summary:
            w.writerow(r)
    print(f"\nwrote {csv_path}\nwrote {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
