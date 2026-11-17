#!/usr/bin/env python3
"""Run uwb_dw3000_live on a replay fixture and parse the summary lines.

Used for the low-SNR evaluation matrix.  Prints one JSON object per run.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

KEYV = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^\s]+)")


def parse(text):
    out = {"raw": text}
    for line in text.splitlines():
        tag = line.split(" ", 1)[0]
        if tag in ("FINAL", "SUMMARY_PARTIAL", "LOCK_STATS", "COST_TOTAL",
                   "STREAM_STARTED", "START"):
            d = {}
            for k, v in KEYV.findall(line):
                try:
                    d[k] = float(v) if ("." in v or "e" in v.lower()) else int(v)
                except ValueError:
                    d[k] = v
            out[tag] = d
    return out


def run(binary, fixture, extra, timeout=600):
    cmd = [binary, "--replay", fixture, "--seconds", "0",
           "--period-ms", "1.0", "--mode", "continuous", "--workers", "4",
           "--cir", "full", "--fused", "on", "--taps", "eq16", "--tone", "off",
           "--print", "summary"] + extra
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    d = parse(p.stdout)
    d["returncode"] = p.returncode
    d["cmd"] = " ".join(cmd)
    d.pop("raw", None)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", default=str(Path(__file__).resolve().parents[3] / "gr-uwb/build-native/apps/uwb_dw3000_live"))
    ap.add_argument("--fixture", required=True)
    ap.add_argument("--sync", default="acquire")
    ap.add_argument("--extra", default="")
    ap.add_argument("--timeout", type=int, default=600)
    a = ap.parse_args()
    extra = ["--sync", a.sync] + a.extra.split()
    d = run(a.binary, a.fixture, extra, a.timeout)
    print(json.dumps(d, indent=1))


if __name__ == "__main__":
    main()
