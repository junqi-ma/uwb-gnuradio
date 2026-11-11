#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare two resampled waveforms produced by different 65/48 tap sets.

The equiripple filter has a different length T, so it has a different group
delay and a different output length.  Comparing the two dumps naively would
measure the delay, not the distortion.  This script

  1. aligns by the *theoretical* group-delay difference (T_a - T_b)/(2*48)
     output samples using an FFT phase ramp -- no free fitting of the shift,
  2. reports the residual with and without that alignment, so the alignment
     itself is evidenced rather than assumed,
  3. separates a pure complex gain from real distortion (best-fit gain),
  4. measures the cross-correlation peak shift and amplitude ratio, which is
     what the CIR peak-position / amplitude gates are actually about.

Usage
-----
    python3 testdata/compare_resampler_waveforms.py \
        --ref /tmp/g3/kaiser/win_0_dump.cf32 \
        --test /tmp/g3/eq/win_0_dump.cf32 \
        --taps-ref 1319 --taps-test 1039
"""
import argparse
import json
import sys

import numpy as np


def load_cf32(path):
    raw = np.fromfile(path, dtype="<f4")
    if raw.size % 2:
        raise SystemExit(f"{path}: odd float count")
    return raw[0::2].astype(np.float64) + 1j * raw[1::2].astype(np.float64)


def frac_shift(x, shift):
    """Shift x by `shift` samples (positive = later) via an FFT phase ramp."""
    n = x.size
    pad = 1 << int(np.ceil(np.log2(2 * n)))
    X = np.fft.fft(x, pad)
    f = np.fft.fftfreq(pad)
    return np.fft.ifft(X * np.exp(-2j * np.pi * f * shift))[:n]


def metrics(ref, test):
    n = min(ref.size, test.size)
    a, b = ref[:n], test[:n]
    d = b - a
    na = np.linalg.norm(a)
    out = {
        "compared": int(n),
        "rel_l2": float(np.linalg.norm(d) / na),
        "evm_pct": float(100.0 * np.linalg.norm(d) / na),
        "max_abs": float(np.max(np.abs(d))),
        "peak": float(np.max(np.abs(a))),
    }
    # best complex gain: how much of the residual is just a scale/rotation?
    denom = float(np.vdot(b, b).real)
    g = complex(np.vdot(b, a) / denom) if denom > 0 else 1.0 + 0j
    resid = b * g - a
    out["fit_gain_db"] = float(20 * np.log10(abs(g))) if abs(g) > 0 else None
    out["fit_phase_deg"] = float(np.degrees(np.angle(g)))
    out["rel_l2_after_gain"] = float(np.linalg.norm(resid) / na)
    out["evm_pct_after_gain"] = float(100.0 * np.linalg.norm(resid) / na)
    return out


def xcorr_peak(ref, test, max_lag=64):
    """Integer-lag cross-correlation peak: shift and amplitude ratio."""
    n = min(ref.size, test.size)
    a, b = ref[:n], test[:n]
    best = None
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            x, y = a[lag:], b[:n - lag]
        else:
            x, y = a[:n + lag], b[-lag:]
        if x.size < 1024:
            continue
        c = abs(np.vdot(x, y)) / (np.linalg.norm(x) * np.linalg.norm(y))
        if best is None or c > best[1]:
            best = (lag, c)
    lag, c = best
    if lag >= 0:
        x, y = a[lag:], b[:n - lag]
    else:
        x, y = a[:n + lag], b[-lag:]
    ratio = float(np.linalg.norm(x) / np.linalg.norm(y))
    return {"peak_lag_samples": int(lag), "peak_corr": float(c),
            "amplitude_ratio_db": float(20 * np.log10(ratio))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--test", required=True)
    ap.add_argument("--taps-ref", type=int, required=True)
    ap.add_argument("--taps-test", type=int, required=True)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    a = load_cf32(args.ref)
    b = load_cf32(args.test)

    dlen = (args.taps_ref - args.taps_test) / (2.0 * 48.0)
    res = {
        "ref": args.ref, "test": args.test,
        "taps_ref": args.taps_ref, "taps_test": args.taps_test,
        "len_ref": int(a.size), "len_test": int(b.size),
        "len_diff": int(a.size - b.size),
        "theoretical_group_delay_diff_samples": dlen,
        "unaligned": metrics(a, b),
        "aligned": metrics(a, frac_shift(b, dlen)),
        "xcorr_aligned": xcorr_peak(a, frac_shift(b, dlen)),
    }

    print(f"ref T={args.taps_ref} ({a.size} samples)  "
          f"test T={args.taps_test} ({b.size} samples)  "
          f"len diff {a.size - b.size}")
    print(f"theoretical group-delay difference {dlen:+.4f} output samples")
    print(f"  unaligned : rel L2 {res['unaligned']['rel_l2']:.4e} "
          f"({res['unaligned']['evm_pct']:.3f}% EVM)")
    print(f"  aligned   : rel L2 {res['aligned']['rel_l2']:.4e} "
          f"({res['aligned']['evm_pct']:.3f}% EVM)")
    print(f"  after gain: rel L2 {res['aligned']['rel_l2_after_gain']:.4e} "
          f"({res['aligned']['evm_pct_after_gain']:.3f}% EVM), "
          f"gain {res['aligned']['fit_gain_db']:+.4f} dB "
          f"phase {res['aligned']['fit_phase_deg']:+.3f} deg")
    x = res["xcorr_aligned"]
    print(f"  xcorr     : residual lag {x['peak_lag_samples']} samples, "
          f"corr {x['peak_corr']:.6f}, "
          f"amplitude {x['amplitude_ratio_db']:+.4f} dB")

    if args.json:
        json.dump(res, open(args.json, "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
