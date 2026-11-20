#!/usr/bin/env python3
"""FER-vs-SNR sweep for the BPM-BPSK ablation levels (M0..M4).

The BPM switches are selected at run time through the `UWB_BPM_LEVEL`
environment variable (0 = previous fused baseline, 1 = known-hop, 2 = joint ML
+ terminated, 3 = PHR refine + DD-PLL, 4 = fused SFO).  This tool reuses
`fixed_scenarios.py` for fixture generation so the synthesised frames match the
frozen 6x40 set (127 B / code10 / 256 SYNC / 4z2) and only the noise seed / SNR
changes.

Examples:
  python3 tools/rx/low_snr_eval/snr_sweep.py --fixtures /tmp/uwb-bpm-snr-sweep \
      --generate --build-dir build/resamp-avx2 \
      --decoder build/resamp-avx2/apps/uwb_dw3000_decode \
      --levels 0,4 --snr -11,-12,-13,-14,-15
"""
from __future__ import annotations

import argparse
import collections
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "rx" / "low_snr_eval"))
import fixed_scenarios as fs  # noqa: E402

TAPS = REPO / "testdata" / "resampler_65_48" / "taps_realtime_eq16.txt"
TEMPLATE = REPO / "testdata" / "reference_preamble_code10_998p4.cf32"


def case_name(snr: float) -> str:
    sign = "m" if snr < 0 else "p"
    return f"snr{sign}{abs(int(round(snr))):02d}db"


def run_level(decoder: Path, level: int, iq: Path, starts: Path,
              truth: list[str]) -> dict:
    env = os.environ.copy()
    env.pop("LD_LIBRARY_PATH", None)
    env["UWB_BPM_LEVEL"] = str(level)
    cmd = [
        str(decoder), str(iq), str(starts), str(TEMPLATE), str(TAPS),
        "256", "4z2", "36864", "450000",
    ]
    done = subprocess.run(cmd, env=env, check=True, text=True,
                          capture_output=True, timeout=1800)
    rows = [json.loads(line) for line in done.stdout.splitlines()]
    if len(rows) != len(truth):
        raise RuntimeError(
            f"level {level} {iq.name}: expected {len(truth)} rows, got {len(rows)}")
    exact = sum(1 for r, t in zip(rows, truth)
                if r.get("fcs") and r.get("hex") == t)
    wrong = sum(1 for r, t in zip(rows, truth)
                if r.get("fcs") and r.get("hex") != t)
    status = collections.Counter(str(r.get("status")) for r in rows)
    sfd_ok = sum(1 for r in rows if r.get("sfd_ok"))
    return {
        "frames": len(rows),
        "exact_psdu_fcs": exact,
        "wrong_psdu_fcs": wrong,
        "fer": 1.0 - exact / len(rows),
        "sfd_ok": sfd_ok,
        "status_counts": dict(status),
        "cfo_hz_median": statistics.median(
            [r["cfo_hz"] for r in rows if r.get("cfo_points")] or [0.0]),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fixtures", type=Path, required=True)
    ap.add_argument("--generate", action="store_true")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--build-dir", type=Path, default=None,
                    help="build dir with generated headers (generation only)")
    ap.add_argument("--decoder", type=Path, required=True)
    ap.add_argument("--levels", default="0,4")
    ap.add_argument("--snr", default="-11,-12,-13,-14,-15")
    ap.add_argument("--seed", type=int, default=1729)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    snrs = [float(v) for v in args.snr.split(",")]
    levels = [int(v) for v in args.levels.split(",")]

    fs.SCENARIOS = [
        (case_name(snr), args.seed, snr, 0, False, 0) for snr in snrs
    ]

    if args.generate:
        build_dir = fs.find_build_dir(args.build_dir)
        manifest = fs.generate_fixtures(args.fixtures, build_dir, args.force)
        (args.fixtures / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n")

    truth_path = args.fixtures / f"truth-{args.seed}.txt"
    truth = truth_path.read_text().splitlines()[1:]
    result = {
        "seed": args.seed,
        "levels": levels,
        "decoder": str(args.decoder),
        "fixtures": {},
        "cases": {},
    }
    failures = []
    for name, seed, snr, cfo, sfo, _ in fs.SCENARIOS:
        iq = args.fixtures / f"{name}.sc16"
        starts = args.fixtures / f"{name}-starts.txt"
        result["fixtures"][name] = {
            "snr_nominal_db": snr,
            "sha256": fs.sha256(iq),
        }
        case = {"snr_nominal_db": snr}
        print(f"[{name}] {snr:+.0f} dB", flush=True)
        for level in levels:
            m = run_level(args.decoder, level, iq, starts, truth)
            case[f"M{level}"] = m
            print(
                f"  M{level}: exact={m['exact_psdu_fcs']}/{m['frames']} "
                f"FER={m['fer']:.3f} wrong={m['wrong_psdu_fcs']} "
                f"sfd_ok={m['sfd_ok']} status={m['status_counts']}",
                flush=True,
            )
            if m["wrong_psdu_fcs"] != 0:
                failures.append(f"{name}/M{level}: wrong PSDU passed FCS")
        result["cases"][name] = case

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2) + "\n")
        print("wrote", args.out)
    if failures:
        raise SystemExit("; ".join(failures))
    return 0


if __name__ == "__main__":
    sys.exit(main())
