#!/usr/bin/env python3
"""ABBA comparison of two uwb_dw3000_live binaries on the 6x40 fixtures.

The 6x40 fixtures (`control`, `seed1729`, `cfo40k`, ...) are produced by
`fixed_scenarios.py`.  Unlike `ab.py`, which drives the low-SNR matrix with
hard-coded file names, this tool accepts any fixture directory that follows
the `<name>.sc16` / `<name>-starts.txt` layout.

Each case is replayed A, B, B, A with one worker so slow machine drift
cancels.  The per-packet stage timers are averaged per run and the median of
the two runs per side is reported.  FCS/status patterns are compared between
the two binaries; a mismatch fails the run (timings are not part of that
comparison).

Example:
  taskset -c 5,6 env -u LD_LIBRARY_PATH python3 \
    tools/rx/low_snr_eval/ab_fixed.py BASE NEW --fixtures /tmp/uwb-low-snr-6x40 \
    --out /tmp/fixed_abba.json --only control
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import statistics
import subprocess
import sys

DEFAULT_CASES = ("control", "seed1729", "cfo40k")

# Packet fields that are numeric stage timers (microseconds) or diagnostics.
NUMERIC_KEYS = (
    "st_total", "st_timing", "st_cfo", "st_sfd", "st_cir", "st_nssfd",
    "st_phr", "st_payload", "st_fallback", "pay_soft", "pay_vit", "pay_rs",
    "pay_crc", "pay_other", "pay_derot", "st_fused", "pay_fused_calls",
    "pay_legacy_calls", "sfd", "phr", "fcs", "crc", "len", "cfo_khz",
    "full_retry", "ondemand_ext", "ondemand_fb", "taps",
)
# Fields that must be identical between A and B for a valid equivalence run.
MATCH_KEYS = ("status", "sfd", "phr", "len", "fcs", "crc", "pay_fused_calls",
              "pay_legacy_calls", "full_retry", "ondemand_ext", "ondemand_fb")

KV = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(\S+)")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.lower().startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def parse_packets(stdout: str) -> list[dict]:
    rows = []
    for line in stdout.splitlines():
        if not line.startswith("PACKET "):
            continue
        row = {}
        for key, value in KV.findall(line):
            row[key] = value
        rows.append(row)
    return rows


def to_float(value: str):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def run_once(binary: Path, fixture: Path, workers: int, queue: int) -> dict:
    cmd = [
        str(binary), "--replay", str(fixture), "--seconds", "0",
        "--period-ms", "1.0", "--mode", "continuous",
        "--workers", str(workers), "--queue", str(queue),
        "--cir", "full", "--fused", "on", "--taps", "eq16",
        "--tone", "off", "--print", "packets",
    ]
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    done = subprocess.run(cmd, capture_output=True, text=True, timeout=1800,
                          check=True, env=env)
    rows = parse_packets(done.stdout)
    if not rows:
        raise RuntimeError(f"{binary} produced no PACKET lines for {fixture}")
    means = {}
    for key in NUMERIC_KEYS:
        values = [to_float(r[key]) for r in rows if key in r]
        values = [v for v in values if v is not None]
        if values:
            means[key] = sum(values) / len(values)
    # FCS/status pattern; used to prove A/B equivalence independently of time.
    pattern = [tuple(r.get(k, "") for k in MATCH_KEYS) for r in rows]
    return {"rows": len(rows), "means": means, "pattern": pattern}


def summarize(runs: list[dict]) -> dict:
    keys = sorted({k for run in runs for k in run["means"]})
    med = {}
    for key in keys:
        vals = [run["means"][key] for run in runs if key in run["means"]]
        if vals:
            med[key] = statistics.median(vals)
    return {"packets": runs[0]["rows"], "median_of_run_means": med,
            "runs": [{"packets": r["rows"], "means": r["means"]} for r in runs]}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("a_binary", type=Path)
    ap.add_argument("b_binary", type=Path)
    ap.add_argument("--fixtures", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--only", help="case name substring filter")
    ap.add_argument("--cases", default=None,
                    help="comma-separated case names (default: control,seed1729,cfo40k)")
    ap.add_argument("--workers", type=int, default=1)
    ap.add_argument("--queue", type=int, default=256)
    ap.add_argument("--rounds", type=int, default=1,
                    help="repeat the A,B,B,A sequence per case")
    ap.add_argument("--label-a", default="baseline")
    ap.add_argument("--label-b", default="optimized")
    ap.add_argument("--allow-fcs-mismatch", action="store_true",
                    help="record but do not fail on FCS/status differences")
    args = ap.parse_args()

    cases = list(DEFAULT_CASES)
    if args.cases:
        cases = [c.strip() for c in args.cases.split(",") if c.strip()]
    if args.only is not None:
        cases = [c for c in cases if args.only in c]
    if not cases:
        ap.error("no cases selected")

    result = {
        "metadata": {
            "cpu": cpu_model(),
            "nproc": os.cpu_count(),
            "platform": platform.platform(),
            "workers": args.workers,
            "queue": args.queue,
            "rounds": args.rounds,
            "label_a": args.label_a,
            "label_b": args.label_b,
            "binaries": {
                "A": {"path": str(args.a_binary),
                      "sha256": sha256(args.a_binary)},
                "B": {"path": str(args.b_binary),
                      "sha256": sha256(args.b_binary)},
            },
            "fixtures_dir": str(args.fixtures),
        },
        "cases": {},
    }
    failures = []
    for name in cases:
        fixture = args.fixtures / f"{name}.sc16"
        if not fixture.exists():
            ap.error(f"missing fixture {fixture}")
        result["metadata"].setdefault("fixtures", {})[name] = sha256(fixture)

        order = [("A", args.a_binary), ("B", args.b_binary),
                 ("B", args.b_binary), ("A", args.a_binary)] * args.rounds
        sides = collections.defaultdict(list)
        print(f"[{name}] A,B,B,A x{args.rounds} ...", flush=True)
        for side, binary in order:
            rec = run_once(binary, fixture, args.workers, args.queue)
            sides[side].append(rec)
            m = rec["means"]
            print(
                f"  {side} packets={rec['rows']} "
                f"total={m.get('st_total', float('nan')):.1f} "
                f"cir={m.get('st_cir', float('nan')):.1f} "
                f"payload={m.get('st_payload', float('nan')):.1f} "
                f"soft={m.get('pay_soft', float('nan')):.1f} "
                f"derot={m.get('pay_derot', float('nan')):.1f} "
                f"rs={m.get('pay_rs', float('nan')):.1f} "
                f"vit={m.get('pay_vit', float('nan')):.1f}",
                flush=True,
            )

        pattern_a = sides["A"][0]["pattern"]
        pattern_b = sides["B"][0]["pattern"]
        same_pattern = pattern_a == pattern_b
        if not same_pattern and not args.allow_fcs_mismatch:
            diffs = [i for i, (a, b) in enumerate(zip(pattern_a, pattern_b))
                     if a != b]
            failures.append(f"{name}: A/B status pattern differs at {diffs}")

        case = {
            "A": summarize(sides["A"]),
            "B": summarize(sides["B"]),
            "pattern_match": same_pattern,
        }
        result["cases"][name] = case
        ma = case["A"]["median_of_run_means"]
        mb = case["B"]["median_of_run_means"]
        for key in ("st_total", "st_cir", "st_payload", "pay_soft",
                    "pay_derot", "pay_rs", "pay_vit"):
            if key in ma and key in mb and ma[key] and mb[key]:
                case.setdefault("speedup", {})[key] = ma[key] / mb[key]
        print(
            f"  => {name}: total {ma.get('st_total', 0):.1f}->"
            f"{mb.get('st_total', 0):.1f} us, payload "
            f"{ma.get('st_payload', 0):.1f}->{mb.get('st_payload', 0):.1f} us, "
            f"cir {ma.get('st_cir', 0):.1f}->{mb.get('st_cir', 0):.1f} us, "
            f"pattern {'OK' if same_pattern else 'DIFF'}",
            flush=True,
        )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=1) + "\n")
    print("wrote", args.out)
    if failures:
        raise SystemExit("; ".join(failures))
    return 0


if __name__ == "__main__":
    sys.exit(main())
