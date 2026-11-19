#!/usr/bin/env python3
"""Compare the native PAR-only experiment with the default detector."""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import subprocess
from collections import Counter
from pathlib import Path

from fixed_scenarios import sha256


ROOT = Path(__file__).resolve().parents[3]
FIXTURES = ("control", "m8", "m10", "m12", "m14", "m16", "noise")
SEED_OFFSETS = (0, -23, 23, -87, 87, 120, -120)
WORK_PER_NATIVE = 65 / 48
WORK_PER_SYNC = 1016
PRE_GUARD = {"acquire": 36864, "locked": 8192, "sfd-probe": 8192}
TIMERS = ("timing_us", "sfd_us", "demod_us")
KEY_VALUE = re.compile(r"(\w+)=([^\s]+)")
PSDU_PREFIX = re.compile(r"\bpsdu=((?:[a-f0-9]{2} ){12})")


def run_logged(command, log):
    with log.open("w") as stream:
        subprocess.run([str(arg) for arg in command], check=True, timeout=180,
                       cwd=ROOT, stdout=stream,
                       env={k: v for k, v in os.environ.items() if k != "LD_LIBRARY_PATH"})
    return log.read_text().splitlines()


def latency(values):
    if not values:
        return None
    values = sorted(values)
    return {"mean": statistics.mean(values), "p95": values[math.ceil(.95 * len(values)) - 1]}


def fixed_measure(lines, starts, positions, truth, pre, probe):
    rows = list(map(json.loads, lines))
    if [row.get("seed") for row in rows] != starts or any("input_error" in row for row in rows):
        raise RuntimeError("Decoder output does not match the fixed input windows")
    # eq16 is causal. The centre of its symmetric high-grid impulse response
    # adds (1039 - 1) / (2 * 48) work samples to each simulated coordinate.
    expected = [round((actual - seed + pre) * WORK_PER_NATIVE + 519 / 48)
                for actual, seed in zip(positions, starts)]
    result = {"windows": len(rows), "sfd_ok": sum(r["sfd_ok"] for r in rows)}
    if not probe:
        result["timing_ok"] = sum(r["timing_ok"] for r in rows)
        result["fcs"] = sum(r["fcs"] for r in rows)
        result["folded_timing"] = sum(r["peaks_folded"] > 0 for r in rows)
        result.update({name: latency([r[name] for r in rows]) for name in TIMERS})
    if truth is not None:
        if not probe:
            exact = {i for i, (r, t) in enumerate(zip(rows, truth), start=1)
                     if r["fcs"] and r["hex"] == t}
            result["exact_psdu"] = len(exact)
            result["wrong_fcs"] = result["fcs"] - len(exact)
            result["exact_slots"] = sorted(exact)
            sync_errors = [r["preamble_start"] - e for r, e in zip(rows, expected)
                           if r["timing_ok"]]
            result["sync_within_4"] = sum(abs(e) <= 4 for e in sync_errors)
            result["sync_error_quantiles"] = latency(list(map(abs, sync_errors)))
        sfd_errors = [r["sfd_start"] - (e + 256 * WORK_PER_SYNC)
                      for r, e in zip(rows, expected) if r["sfd_ok"]]
        result["sfd_within_4"] = sum(abs(e) <= 4 for e in sfd_errors)
        result["sfd_error_quantiles"] = latency(list(map(abs, sfd_errors)))
    return result


def fixed_runs(fixture, positions, truth, decoder, strategy, out, par_window):
    # The first slot lacks pre-guard. SFD probe injects a timing hypothesis at
    # the exact SYNC phase to expose false alarms that timing might mask.
    positions = positions[1:]
    if truth is not None:
        truth = truth[1:]
    probe = strategy == "sfd-probe"
    offsets = (0,) if probe else SEED_OFFSETS
    starts = [p + offsets[i % len(offsets)] for i, p in enumerate(positions)]
    seed_list = out / f"{fixture.stem}-{strategy}-starts.txt"
    seed_list.write_text("".join(f"{s}\n" for s in starts))
    pre = PRE_GUARD[strategy]
    args = [decoder, fixture, seed_list, ROOT / "testdata/reference_preamble_code10_998p4.cf32",
            ROOT / "testdata/resampler_65_48/taps_realtime_eq16.txt", "256", "4z2",
            pre, 450000]
    if par_window:
        order = ("guarded", par_window, par_window, "guarded")
    elif probe:
        order = ("off", "guarded", "guarded", "off", "sfd")
    else:
        order = ("off", "both", "both", "off", "off", "guarded", "guarded", "off", "sync", "sfd")
    runs = {}
    for index, mode in enumerate(order):
        log = out / f"fixed-{fixture.stem}-{strategy}-{index}-{mode}.jsonl"
        lines = run_logged(args + [mode, strategy], log)
        runs.setdefault(mode, []).append(fixed_measure(lines, starts, positions, truth, pre, probe))
    return runs


def live_measure(lines, truth):
    pipe = {}
    packet_stats = []
    good = Counter()
    prefixes = {t[:24]: i for i, t in enumerate(truth or ())}
    wrong = 0
    for line in lines:
        tag, _, _ = line.partition(" ")
        fields = dict(KEY_VALUE.findall(line))
        if tag in {"START", "LOCK_STATS", "COST_TOTAL", "FINAL"}:
            pipe[tag] = fields
        if tag != "PACKET":
            continue
        packet_stats.append(fields)
        if fields.get("status") != "OK":
            continue
        prefix = PSDU_PREFIX.search(line)
        slot = prefixes.get(prefix[1].replace(" ", "")) if prefix else None
        if slot is None:
            wrong += 1
        else:
            good[slot] += 1
    if "FINAL" not in pipe or "COST_TOTAL" not in pipe:
        raise RuntimeError("Missing live replay summary")
    return {"summary": pipe, "evaluated": len(packet_stats), "correct_slots": len(good),
            "correct_slot_ids": sorted(good), "wrong_fcs_prefix": wrong,
            "duplicate_fcs": sum(good.values()) - len(good),
            "timing_ok": sum("start" in p for p in packet_stats),
            "sfd_ok": sum(p.get("sfd") == "1" for p in packet_stats),
            "timing_us": latency([float(p["st_timing"]) for p in packet_stats if "st_timing" in p]),
            "sfd_us": latency([float(p["st_sfd"]) for p in packet_stats if "st_sfd" in p]),
            "demod_us": latency([float(p["demod_us"]) for p in packet_stats if "demod_us" in p])}


def live_runs(fixture, truth, binary, strategy, out, par_window):
    command = [binary, "--replay", fixture, "--seconds", 0, "--period-ms", 1,
               "--mode", "continuous", "--sync", strategy, "--workers", 4, "--queue", 64,
               "--cir", "full", "--fused", "on", "--taps", "eq16", "--tone", "off",
               "--print", "packets"]
    runs = {}
    order = ("guarded", par_window, par_window, "guarded") if par_window else (
        "off", "both", "both", "off", "off", "guarded", "guarded", "off")
    for index, mode in enumerate(order):
        log = out / f"live-{fixture.stem}-{strategy}-{index}-{mode}.log"
        lines = run_logged(command + ["--par-only", mode], log)
        runs.setdefault(mode, []).append(live_measure(lines, truth))
    return runs


def summarize(runs, key):
    return {mode: [r.get(key) for r in items] for mode, items in runs.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixtures", type=Path, required=True)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--cases", nargs="+", choices=FIXTURES, default=FIXTURES)
    parser.add_argument("--kind", choices=("fixed", "live", "probe", "both"), default="both")
    parser.add_argument("--par-window", choices=("window3", "window3-sfd"),
                        help="compare the selected three-sample PAR mode with guarded")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    build = args.build.resolve()
    fixed_bin, live_bin = (build / "apps" / p for p in ("uwb_dw3000_decode", "uwb_dw3000_live"))
    metadata = {"head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                                text=True).strip(),
                "dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)),
                "cpu": next(line.partition(":")[2].strip()
                            for line in Path("/proc/cpuinfo").read_text().splitlines()
                            if line.startswith("model name")),
                "cpus": sorted(os.sched_getaffinity(0)),
                "cmake": [line for line in (build / "CMakeCache.txt").read_text().splitlines()
                          if line.startswith(("CMAKE_CXX_COMPILER:", "CMAKE_BUILD_TYPE:",
                                              "UWB_SIMD:", "ENABLE_UHD_BACKEND:"))],
                "bin_sha256": {p.name: sha256(p) for p in (fixed_bin, live_bin) if p.exists()}}
    report = {"metadata": metadata, "cases": {}}
    for case in args.cases:
        fixture = args.fixtures.resolve() / (case + ".sc16")
        spec = json.loads(fixture.with_suffix(".sc16.json").read_text())
        if sha256(fixture) != spec["sha256"]:
            raise RuntimeError(f"Unexpected fixture hash for {fixture}")
        truth_path = fixture.parent / f"truth-{spec['seed']}.txt"
        truth = None if spec["snr_db"] is None else truth_path.read_text().splitlines()[:spec["packets"]]
        if truth is not None and len(truth) != spec["packets"]:
            raise RuntimeError(f"Incomplete PSDU truth for {fixture}")
        record = {"snr_db": spec["snr_db"], "seed": spec["seed"], "sha256": spec["sha256"],
                  "truth_sha256": sha256(truth_path) if truth is not None else None}
        report["cases"][case] = record
        if args.kind in ("fixed", "probe", "both"):
            strategies = ("sfd-probe",) if args.kind == "probe" else (
                "acquire", "locked", "sfd-probe")
            for strategy in strategies:
                runs = fixed_runs(fixture, spec["packet_starts"], truth, fixed_bin, strategy, out,
                                  args.par_window)
                record[f"fixed/{strategy}"] = runs
                key = "exact_psdu" if truth is not None and strategy != "sfd-probe" else "sfd_ok"
                print(case, "fixed", strategy, key, summarize(runs, key), flush=True)
        if args.kind in ("live", "both"):
            runs = live_runs(fixture, truth, live_bin, "acquire" if case == "noise" else "locked",
                            out, args.par_window)
            record["live"] = runs
            print(case, "live", "correct_slots", summarize(runs, "correct_slots"), flush=True)
        (out / "par_only.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
