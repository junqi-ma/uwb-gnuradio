#!/usr/bin/env python3
"""Fixed 491.52 MS/s TX pulse-shape experiment (see
docs/固定491p52采样率_发射脉冲低拖尾方案.md).

Pulse cores come from the C++ modulator (`uwb.make_pulse_taps`) so the
numbers describe the actual TX output.  Each core is applied to a single
chip impulse on the 998.4 MS/s work grid, then resampled to the target
native rate with the same 32/65 stage the apps use.  Metrics are the
worst case over all work-grid fractional phases.

Outputs: analysis_outputs/fixed_rate_tx_pulse/metrics.json and .png
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import json
import os
import sys
import time

import numpy as np
from scipy.signal import resample_poly, resample

FS_WORK = 998.4e6
FS_NATIVE = 491.52e6
FS_737 = 737.28e6


def load_uwb():
    here = os.path.dirname(os.path.abspath(__file__))
    bindir = os.path.join(here, "gr-uwb", "build", "python", "uwb", "bindings")
    matches = sorted(glob.glob(os.path.join(bindir, "uwb_python*.so")))
    if not matches:
        bindir = os.path.join(here, "gr-uwb", "build", "test_modules")
        matches = sorted(glob.glob(os.path.join(bindir, "uwb*so")))
    if not matches:
        raise SystemExit("missing uwb bindings under gr-uwb/build; build first")
    spec = importlib.util.spec_from_file_location("uwb_python", matches[-1])
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def pulse_metrics(taps, fs, up=64):
    """Worst-case envelope metrics of one chip pulse after resampling.

    Returns max |envelope| in dB relative to the pulse peak for the lag
    windows from the design doc, plus the interpolated -3 dB width.
    """
    taps = np.asarray(taps, dtype=np.float64)
    worst = None
    width = 0.0
    for off in range(0, 65):
        work = np.zeros(len(taps) + 16384)
        work[8192 + off] = 1.0
        y = np.convolve(work, taps)[: len(work)]
        if fs == FS_NATIVE:
            nat = resample_poly(y, 32, 65)
        else:  # 737.28 MS/s reference
            nat = resample_poly(y, 48, 65)
        a = np.abs(nat)
        p = int(np.argmax(a))
        if a[p] == 0:
            continue
        peak = a[p]
        xf = np.abs(resample(nat, len(nat) * up))
        pf = int(np.argmax(xf))
        lag = (np.arange(len(xf)) - pf) / (fs * up) * 1e9
        scale = xf[pf]
        m = {}
        for name, lo, hi in (("pre_20_10", -20, -10), ("post_10_20", 10, 20),
                             ("pre_10_100", -100, -10),
                             ("post_10_100", 10, 100)):
            sel = (lag >= lo) & (lag <= hi)
            m[name] = 20 * np.log10(max(xf[sel].max(), 1e-300) / scale)
        m["both_10_100"] = max(m["pre_10_100"], m["post_10_100"])
        # -3 dB width around the peak (main lobe)
        half = scale / np.sqrt(2.0)
        i = pf
        while i > 0 and xf[i] > half:
            i -= 1
        j = pf
        while j < len(xf) - 1 and xf[j] > half:
            j += 1
        w = (j - i) / (fs * up) * 1e9
        if worst is None:
            worst = m
        else:
            for k in worst:
                worst[k] = max(worst[k], m[k])
        width = max(width, w)
    worst["width_3db_ns"] = width
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "analysis_outputs", "fixed_rate_tx_pulse"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    uwb = load_uwb()

    candidates = [
        ("legacy_737p28", "legacy", 0.0, 0.0, FS_737),
        ("legacy_491p52", "legacy", 0.0, 0.0, FS_NATIVE),
        ("gaussian_sigma2p2", "gaussian", 2.2, 0.0, FS_NATIVE),
        ("gaussian_sigma2p5", "gaussian", 2.5, 0.0, FS_NATIVE),
        ("blackman_bw180", "blackman", 0.0, 180.0, FS_NATIVE),
        ("blackman_bw200", "blackman", 0.0, 200.0, FS_NATIVE),
        ("blackman_bw220", "blackman", 0.0, 220.0, FS_NATIVE),
    ]
    rows = []
    for name, shape, sigma, bw, fs in candidates:
        taps = uwb.make_pulse_taps(shape, sigma, bw)
        m = pulse_metrics(taps, fs)
        m.update({"name": name, "shape": shape, "sigma_ns": sigma,
                  "bw_mhz": bw, "fs_hz": fs, "taps": len(taps)})
        rows.append(m)
        print("%-18s taps=%3d fs=%.2f MS/s  pre20-10=%7.1f  post10-20=%7.1f"
              "  both10-100=%7.1f  w3db=%.2f ns"
              % (name, len(taps), fs / 1e6, m["pre_20_10"], m["post_10_20"],
                 m["both_10_100"], m["width_3db_ns"]), flush=True)

    meta = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "work_rate_hz": FS_WORK,
        "native_rate_hz": FS_NATIVE,
        "metric": "max |envelope| in lag window relative to pulse peak, "
                  "worst over 65 work-grid phases, 64x sinc interpolation",
        "rows": rows,
    }
    with open(os.path.join(a.out, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
        f.write("\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
        for name, shape, sigma, bw, fs in candidates:
            taps = uwb.make_pulse_taps(shape, sigma, bw)
            work = np.zeros(len(taps) + 16384)
            work[8192] = 1.0
            y = np.convolve(work, taps)[: len(work)]
            nat = resample_poly(y, 32 if fs == FS_NATIVE else 48, 65)
            env = np.abs(nat)
            p = int(np.argmax(env))
            lag = (np.arange(len(nat)) - p) / fs * 1e9
            sel = (lag >= -100) & (lag <= 100)
            ax[0].plot(lag[sel],
                       20 * np.log10(env[sel] / env[p] + 1e-300), label=name)
            H = 20 * np.log10(np.abs(np.fft.rfft(taps, 8192)) + 1e-30)
            fr = np.fft.rfftfreq(8192, 1.0 / FS_WORK) / 1e6
            sel = fr <= 499.2
            H -= H[0]
            ax[1].plot(fr[sel], H[sel], label=name)
        ax[0].set_xlabel("lag (ns)")
        ax[0].set_ylabel("dB rel. peak")
        ax[0].set_ylim(-90, 5)
        ax[0].grid(True)
        ax[0].legend(fontsize=7)
        ax[1].set_xlabel("frequency (MHz)")
        ax[1].set_ylabel("|H| (dB, work grid)")
        ax[1].set_xlim(0, 499.2)
        ax[1].set_ylim(-80, 5)
        ax[1].axvline(245.76, color="k", lw=0.8, ls="--")
        ax[1].grid(True)
        fig.tight_layout()
        fig.savefig(os.path.join(a.out, "tx_pulse_comparison.png"), dpi=120)
        print("png: %s" % os.path.join(a.out, "tx_pulse_comparison.png"))
    except ImportError as e:
        print("matplotlib unavailable, skipped png: %s" % e, file=sys.stderr)
    print("metrics: %s" % os.path.join(a.out, "metrics.json"))


if __name__ == "__main__":
    main()
