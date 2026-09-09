#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Export 65/32 upfirdn goldens (491.52 -> 998.4 MS/s)."""
import json
import os

import numpy as np
from scipy.signal import upfirdn

R_IN = 491.52e6
R_OUT = 998.4e6
L, M = 65, 32
HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "resampler_65_32")
PROFILES = {
    "quality_minorder": os.path.join(RES, "taps_quality_minorder.txt"),
    "realtime_minorder": os.path.join(RES, "taps_realtime_minorder.txt"),
}


def load_taps(path):
    return np.fromfile(path, dtype=np.float32)


def write_cf32(path, z):
    z.astype(np.complex64).tofile(path)
    print("  wrote %s (%d samples)" % (path, len(z)))


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="quality_minorder",
                    choices=list(PROFILES.keys()))
    args = ap.parse_args()

    gold_dir = os.path.join(RES, "golden_%s" % args.profile)
    os.makedirs(gold_dir, exist_ok=True)
    n_in = 4096
    taps = load_taps(PROFILES[args.profile])
    meta = {"interpolation": L, "decimation": M,
            "input_rate_hz": R_IN, "output_rate_hz": R_OUT,
            "n_input_samples": n_in, "profile": args.profile,
            "tap_count": int(len(taps))}

    goldens = {}
    goldens["impulse"] = np.zeros(n_in, complex)
    goldens["impulse"][0] = 1.0 + 0j
    goldens["dc"] = np.ones(n_in, complex)
    t = np.arange(n_in) / R_IN
    goldens["tone_low"] = np.exp(2j * np.pi * 10e6 * t)
    goldens["tone_pb"] = np.exp(2j * np.pi * 200e6 * t)
    goldens["tone_sb"] = np.exp(2j * np.pi * 300e6 * t)
    rng = np.random.default_rng(20260909)
    z = rng.standard_normal(n_in) + 1j * rng.standard_normal(n_in)
    goldens["random"] = (z / np.sqrt(2.0)).astype(complex)

    lengths = {}
    for name, x in goldens.items():
        y = upfirdn(taps, x, L, M)
        write_cf32(os.path.join(gold_dir, "%s_in.cf32" % name), x)
        write_cf32(os.path.join(gold_dir, "%s_out.cf32" % name), y)
        lengths[name] = {"n_in": int(len(x)), "n_out": int(len(y))}

    meta["lengths"] = lengths
    with open(os.path.join(gold_dir, "golden.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("wrote", os.path.join(gold_dir, "golden.json"))


if __name__ == "__main__":
    main()
