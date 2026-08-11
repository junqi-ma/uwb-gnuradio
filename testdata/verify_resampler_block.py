#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Post-implementation verifier: feed each golden _in.cf32 through the fixed
65/48 resampler (Python-bound UwbRationalResamplerCcf65_48, or the built-in
rational_resampler_ccf) and compare against the golden _out.cf32.

Usage:
  python3 testdata/verify_resampler_block.py [--block fixed|builtin] [--profile quality|realtime]

For `fixed`, the gr-uwb python bindings must expose
gr::uwb::UwbRationalResamplerCcf65_48.  For `builtin`, uses
gr::filter::rational_resampler_ccf (boundary convention differs from
upfirdn — mismatch at the head/tail is expected; interior must agree).

Prereq for python imports:
  PYTHONPATH=gr-uwb/build/test_modules LD_LIBRARY_PATH=gr-uwb/build/lib
"""
import argparse
import os
import sys

import numpy as np

RES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "resampler_65_48")
TAPS = RES

# profile -> (taps file, golden dir)
PROFILE_PATHS = {
    "quality": ("taps_quality.txt", "golden"),
    "realtime": ("taps_realtime.txt", "golden_realtime"),
    "quality_minorder": ("taps_quality_minorder.txt", "golden_quality_minorder"),
    "realtime_minorder": ("taps_realtime_minorder.txt",
                          "golden_realtime_minorder"),
}


def read_cf32(path):
    return np.fromfile(path, dtype=np.complex64)


def err_report(name, a, b, interior=False):
    n = min(len(a), len(b))
    if n == 0:
        return None
    if interior:
        # skip group-delay transients at both ends
        skip = 100
        n = max(n - 2 * skip, 1)
        a, b = a[skip:skip + n], b[skip:skip + n]
    diff = a - b
    max_abs = float(np.max(np.abs(diff))) if n else 0.0
    rel_l2 = float(np.linalg.norm(diff) / (np.linalg.norm(b) + 1e-30))
    corr = float(np.abs(np.vdot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-30))
    print(f"  [{name}] len_gr={len(a)} len_gold={len(b)} "
          f"max_abs={max_abs:.3e} rel_L2={rel_l2:.3e} corr={corr:.6f}")
    return {"name": name, "max_abs": max_abs, "rel_l2": rel_l2, "corr": corr}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--block", default="fixed", choices=["fixed", "builtin"])
    ap.add_argument("--profile", default="quality", choices=list(PROFILE_PATHS))
    args = ap.parse_args()

    taps_name, gold_sub = PROFILE_PATHS[args.profile]
    gold = os.path.join(RES, gold_sub)
    taps_path = os.path.join(TAPS, taps_name)

    if args.block == "fixed":
        from gnuradio import gr, blocks, uwb  # noqa: F401 (registers bindings)
    else:
        from gnuradio import gr, blocks, filter

    results = []
    npass = 0
    nfail = 0
    for name in ["impulse", "dc", "tone_low", "tone_pb", "tone_sb",
                 "random", "uwb"]:
        taps = np.fromfile(taps_path, dtype=np.float32)
        if args.block == "fixed":
            blk = uwb.rational_resampler_ccf_65_48.make_from_taps(taps.tolist())
        else:
            blk = filter.rational_resampler_ccf(65, 48, taps.tolist())
        x = read_cf32(os.path.join(gold, f"{name}_in.cf32"))
        y_gold = read_cf32(os.path.join(gold, f"{name}_out.cf32"))
        src = blocks.vector_source_c(x.tolist(), False)
        snk = blocks.vector_sink_c()
        tb = gr.top_block()
        tb.connect((src, 0), (blk, 0))
        tb.connect((blk, 0), (snk, 0))
        tb.run()
        y_gr = np.array(snk.data(), dtype=np.complex64)
        # full-length compare for fixed block; interior compare for builtin
        interior = (args.block == "builtin")
        r = err_report(name, y_gr, y_gold, interior=interior)
        if r:
            results.append(r)
        if args.block == "fixed":
            ok = (len(y_gr) == len(y_gold) and r["max_abs"] < 2e-3)
            print(f"    -> {'PASS' if ok else 'FAIL'}")
            npass += 1 if ok else 0
            nfail += 0 if ok else 1
        else:
            print(f"    -> (builtin interior-only; length diff expected)")
    if args.block == "fixed":
        print(f"summary: {npass} pass / {nfail} fail (len must match exactly, "
              f"max_abs < 2e-3)")
        sys.exit(0 if nfail == 0 else 1)


if __name__ == "__main__":
    main()
