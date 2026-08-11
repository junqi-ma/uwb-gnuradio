#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Export 65/48 resampler golden vectors (scipy.signal.upfirdn == MATLAB
upfirdn).  Each golden is a pair:
    <name>_in.cf32   : 737.28 MS/s complex input   (what the block consumes)
    <name>_out.cf32  : 998.4 MS/s complex output   (scipy upfirdn, canonical)

scipy.signal.upfirdn(x, taps, up, down) implements the identical
polyphase upfirdn as MATLAB's upfirdn(x, taps, up, down), so goldens are
cross-tool comparable.  The C++ core QA loads the _in/_out pair and must
reproduce _out from _in within the tolerances fixed in the QA.

Profiles: taps_quality.txt (B=330) and taps_realtime.txt (B=290).

Goldens:
  impulse  : unit complex impulse at offset 0
  dc       : (1+0j) constant
  tone_low : 10 MHz complex tone
  tone_pb  : 300 MHz complex tone (just inside passband edge)
  tone_sb  : 420 MHz complex tone (in stopband -> ~0 output)
  random   : seeded complex white noise
  uwb      : real UWB window: 998.4 MHz cfile decimated to 737.28 MS/s
             (anti-alias, same filter family) then upsampled back
"""
import json
import os
import sys

import numpy as np
from scipy.signal import upfirdn

R_IN = 737.28e6
R_OUT = 998.4e6
L, M = 65, 48
HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "resampler_65_48")
GOLD = os.path.join(RES, "golden")

PROFILES = {
    "quality": os.path.join(RES, "taps_quality.txt"),
    "realtime": os.path.join(RES, "taps_realtime.txt"),
    "quality_minorder": os.path.join(RES, "taps_quality_minorder.txt"),
    "realtime_minorder": os.path.join(RES, "taps_realtime_minorder.txt"),
}


def load_taps(path):
    return np.fromfile(path, dtype=np.float32)


def decimate_998p4_to_737p28(x, taps):
    """Decimate a 998.4 MS/s stream to 737.28 MS/s with the canonical
    anti-alias filter family (L=48, M=65).  Same virtual rate as the
    up-sample, so the same design rule applies (passband B, first alias at
    R_out - B).  Group delay is shared by both directions in QA."""
    return upfirdn(taps, x, M, L)   # 48/65; NOTE: upfirdn(h, x, up, down)


def write_cf32(path, z):
    z.astype(np.complex64).tofile(path)
    print(f"  wrote {path}  ({len(z)} samples)")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="quality",
                    choices=list(PROFILES.keys()))
    args = ap.parse_args()

    gold_dir = os.path.join(RES, f"golden_{args.profile}")
    os.makedirs(gold_dir, exist_ok=True)
    n_in = 4096
    taps = load_taps(PROFILES[args.profile])
    meta = {"interpolation": L, "decimation": M,
            "input_rate_hz": R_IN, "output_rate_hz": R_OUT,
            "n_input_samples": n_in, "profile": args.profile,
            "tap_count": int(len(taps))}

    # ---- synthetic goldens ----
    goldens = {}
    goldens["impulse"] = np.zeros(n_in, complex)
    goldens["impulse"][0] = 1.0 + 0j
    goldens["dc"] = np.ones(n_in, complex)
    t = np.arange(n_in) / R_IN
    goldens["tone_low"] = np.exp(2j * np.pi * 10e6 * t)
    goldens["tone_pb"] = np.exp(2j * np.pi * 300e6 * t)
    goldens["tone_sb"] = np.exp(2j * np.pi * 420e6 * t)
    rng = np.random.default_rng(20260811)
    z = (rng.standard_normal(n_in) + 1j * rng.standard_normal(n_in))
    goldens["random"] = (z / np.sqrt(2.0)).astype(complex)

    # ---- real UWB golden ----
    cfile = os.path.join(HERE, "uwb_code9_preamble64_payload128_standard_sfd.cfile")
    x998 = np.fromfile(cfile, dtype=np.complex64)
    start = 4992000
    win = x998[start:start + 8192]                 # preamble start window
    dec_taps = load_taps(PROFILES["realtime"])     # anti-alias for 48/65
    x737 = decimate_998p4_to_737p28(win, dec_taps)
    meta["uwb_998p4_start_sample"] = start
    meta["uwb_decimate_taps_profile"] = "realtime"
    goldens["uwb"] = x737.astype(complex)
    meta["uwb_input_len"] = int(len(x737))

    for name, x in goldens.items():
        y = upfirdn(taps, x, L, M)   # NOTE: upfirdn(h, x, up, down)
        write_cf32(os.path.join(gold_dir, f"{name}_in.cf32"), x)
        write_cf32(os.path.join(gold_dir, f"{name}_out.cf32"), y)
        print(f"  [{name}] in={len(x)} out={len(y)}")

    with open(os.path.join(gold_dir, "golden_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print(f"wrote golden_meta.json in {gold_dir}")


if __name__ == "__main__":
    main()
