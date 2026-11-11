#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent double-precision reference for the 65/48 resampler core.

Why this exists
---------------
The equiripple experiment changes only the coefficient vector, but `T` also
changes `H_ = ceil(T/65)`, the polyphase arm split, the history length and the
output-length contract.  A pure frequency-response check cannot see any of
that.  This script re-derives the *time-domain* output from the binary tap file
with an independent float64 implementation and compares it against what the C++
core actually emitted (`benchmark_sc16_burst_core --dump-iq`).

It is independent in the useful sense: no C++ code is reused, the input is
regenerated from the published LCG, and the polyphase decomposition is written
from the definition rather than from the kernel.

Math
----
    v[j] = x[j/65]  for j % 65 == 0, 0 <= j <= (N-1)*65, else 0
    u    = v * h                       (full convolution)
    y[m] = u[48*m]
    n_out = ceil(((N-1)*65 + T) / 48)  (the core's contract)

Usage
-----
    python3 testdata/verify_resampler_core_reference.py \
        --taps testdata/resampler_65_48_equiripple/taps_realtime_eq_hpad16.txt \
        --iq /tmp/dump.bin --length 1000 --input random --seed 19088743
"""
import argparse
import json
import sys

import numpy as np
from scipy.signal import fftconvolve

L, M = 65, 48


# ------------------------------------------------------------- input model ---

def make_input(n, seed, mode):
    """Bit-exact reproduction of benchmark_sc16_burst_core's make_input()."""
    re = np.zeros(n, dtype=np.int32)
    im = np.zeros(n, dtype=np.int32)
    if mode == "zero":
        pass
    elif mode == "rails":
        rails = [32767, -32768, -32767, 0, 32767, -32768]
        idx = np.arange(n)
        re = np.array([rails[i % 6] for i in idx], dtype=np.int32)
        im = np.array([rails[(i + 3) % 6] for i in idx], dtype=np.int32)
    elif mode == "impulse":
        if n > 0:
            re[n // 2] = 32767
            im[n // 2] = -32768
    elif mode == "random":
        s = np.uint32(seed | 1)
        for i in range(n):
            s = np.uint32((int(s) * 1664525 + 1013904223) & 0xFFFFFFFF)
            re[i] = np.int32(int(s) >> 16) - 32768
            s = np.uint32((int(s) * 1664525 + 1013904223) & 0xFFFFFFFF)
            im[i] = np.int32(int(s) >> 16) - 32768
        # int16 wrap-around, exactly like the C++ static_cast<int16_t>
        re = ((re + 32768) & 0xFFFF) - 32768
        im = ((im + 32768) & 0xFFFF) - 32768
    else:
        raise SystemExit(f"unknown input mode {mode}")
    return re.astype(np.float64), im.astype(np.float64)


def expected_output_length(n, T):
    num = (n - 1) * L + T
    if num <= 0:
        return 0
    return (num + M - 1) // M


# --------------------------------------------------------------- reference ---

def reference(x_re, x_im, taps, n_out):
    """float64 upfirdn reference."""
    n = x_re.size
    vlen = (n - 1) * L + 1 if n > 0 else 0
    v_re = np.zeros(vlen, dtype=np.float64)
    v_im = np.zeros(vlen, dtype=np.float64)
    if n > 0:
        v_re[::L] = x_re
        v_im[::L] = x_im
    h = taps.astype(np.float64)
    u_re = fftconvolve(v_re, h)
    u_im = fftconvolve(v_im, h)
    return u_re[::M][:n_out], u_im[::M][:n_out]


# -------------------------------------------------------------------- main ---

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--taps", required=True)
    ap.add_argument("--iq", required=True, help="raw CF32 dump from --dump-iq")
    ap.add_argument("--length", type=int, required=True)
    ap.add_argument("--input", default="random")
    ap.add_argument("--seed", type=int, default=0x1234567)
    ap.add_argument("--scale", type=float, default=1.0 / 32768.0)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    taps = np.fromfile(args.taps, dtype="<f4")
    T = taps.size
    n_out = expected_output_length(args.length, T)

    got = np.fromfile(args.iq, dtype="<f4")
    if got.size % 2:
        raise SystemExit("dump is not an even number of floats")
    got = got.view("<c8") if False else (got[0::2] + 1j * got[1::2])

    x_re, x_im = make_input(args.length, args.seed, args.input)
    ref_re, ref_im = reference(x_re * args.scale, x_im * args.scale, taps, n_out)
    ref = ref_re + 1j * ref_im

    res = {"taps": args.taps, "taps_count": int(T), "length": args.length,
           "input": args.input, "seed": args.seed,
           "expected_out": int(n_out), "dumped_out": int(got.size)}

    if got.size != n_out:
        res["length_ok"] = False
        print(f"LENGTH MISMATCH: contract {n_out}, dumped {got.size}")
        if args.json:
            json.dump(res, open(args.json, "w"), indent=2)
        return 1
    res["length_ok"] = True

    n = min(got.size, ref.size)
    d = got[:n] - ref[:n]
    scale_ref = np.max(np.abs(ref[:n])) or 1.0
    res.update(
        max_abs_err=float(np.max(np.abs(d))),
        rms_err=float(np.sqrt(np.mean(np.abs(d) ** 2))),
        rel_l2=float(np.linalg.norm(d) / (np.linalg.norm(ref[:n]) or 1.0)),
        rel_max=float(np.max(np.abs(d)) / scale_ref),
        ref_peak=float(scale_ref),
        compared=int(n))

    print(f"[{args.input}] T={T} N={args.length} -> {n_out} out samples")
    print(f"  length contract   {'OK' if res['length_ok'] else 'FAIL'}")
    print(f"  max abs err       {res['max_abs_err']:.3e}")
    print(f"  rms err           {res['rms_err']:.3e}")
    print(f"  relative L2       {res['rel_l2']:.3e}")
    print(f"  relative max      {res['rel_max']:.3e}  (peak {res['ref_peak']:.4f})")

    if args.json:
        json.dump(res, open(args.json, "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
