#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 3.5: minimum-order 65/48 anti-image filter re-design.

Rationale (docs/performance/分析与下阶段建议_GNURadio软件升采样_65_48.md §3.1):
the original design interpreted "stopband >= 70 dB" as ABSOLUTE attenuation.
With DC gain scaled to sum=65 (+36.26 dB passband), the filters actually hit
~-130 dB RELATIVE to passband — ~60 dB over-specified.  This script re-designs
with the RELATIVE criterion:

    passband ripple      <= 0.1 dB
    |H(stop)|/|H(pass)|  <= -70 dB        (relative to passband gain)
    same passband edges  (quality B=330 MHz, realtime B=290 MHz)
    DC gain (sum)        = 65  (= interpolation)

and searches for the minimum tap count that still meets all specs.

Outputs (testdata/resampler_65_48/):
    taps_quality_minorder.txt / taps_realtime_minorder.txt (float32)
    taps_*_minorder.csv
    design_minorder.json
    taps_quality.txt / taps_realtime.txt are NOT overwritten (canonical kept).
"""
import json
import os

import numpy as np
from scipy.signal import firwin, kaiserord

R_IN = 737.28e6
R_OUT = 998.4e6
L, M = 65, 48
R_V = L * R_IN
PASSBAND_RIPPLE_DB = 0.1
REL_STOPBAND_DB = -70.0

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "resampler_65_48")


def response(taps, freqs):
    w = 2 * np.pi * np.asarray(freqs) / R_V
    return np.array([np.dot(taps, np.exp(-1j * np.arange(len(taps)) * wi))
                     for wi in w])


def meets_spec(taps, pb, sb):
    """Return (meets, metrics) for a candidate tap vector."""
    f = np.concatenate([np.linspace(0, pb, 4001),
                        np.linspace(sb, 500e6, 4001)])
    h = response(taps, f)
    mag = 20 * np.log10(np.abs(h) + 1e-30)
    n_pb = 4001
    pb_mag = mag[:n_pb]
    sb_mag = mag[n_pb:]
    pb_gain = pb_mag.max()
    ripple = pb_mag.max() - pb_mag.min()
    rel_sb = sb_mag.max() - pb_gain          # relative to passband
    dc = float(np.sum(taps))
    met = (ripple <= PASSBAND_RIPPLE_DB and rel_sb <= REL_STOPBAND_DB
           and abs(dc - L) < 1e-3)
    return met, {"ripple_db": float(ripple), "rel_sb_db": float(rel_sb),
                 "pb_gain_db": float(pb_gain), "dc": dc}


def design_min(passband_mhz, name):
    pb = passband_mhz * 1e6
    sb = R_IN - pb
    width_hz = sb - pb
    # scipy kaiserord `width` is normalized so 1 == pi rad/sample == fs/2,
    # i.e. width = transition/(R_V/2).  This is the MINIMUM-order Kaiser
    # design for the requested relative attenuation (per the Kaiser formula
    # N ~ (A-7.95)/(2.285 * 2pi * df_norm)).
    width_nyq = width_hz / (R_V / 2.0)
    ntaps, beta = kaiserord(-REL_STOPBAND_DB, width_nyq)
    if ntaps % 2 == 0:
        ntaps += 1
    cutoff = 0.5 * (pb + sb)

    # Verify the Kaiser estimate meets the RELATIVE spec; if not (e.g. tap
    # count rounding), search upward in odd steps until it does.
    best = None
    best_meta = None
    n = ntaps
    while n <= ntaps + 300:
        taps = firwin(n, cutoff, width=width_hz, window=("kaiser", beta),
                      scale=True, fs=R_V) * L
        met, meta = meets_spec(taps, pb, sb)
        if met:
            best = taps
            best_meta = meta
            break
        n += 2  # keep odd/symmetric
    if best is None:
        raise SystemExit(f"{name}: no design found meeting spec")
    gd_out = (len(best) - 1) / 2 / M
    print(f"[{name}] B={passband_mhz} taps={len(best)} "
          f"arms={int(np.ceil(len(best)/L))} "
          f"ripple={best_meta['ripple_db']:.4f} dB "
          f"rel_stopband={best_meta['rel_sb_db']:.2f} dB "
          f"dc={best_meta['dc']:.4f} gd_out={gd_out:.2f} "
          f"(Kaiser est {ntaps})")
    return best, best_meta


def main():
    quality, qm = design_min(330.0, "quality")
    realtime, rm = design_min(290.0, "realtime")

    for name, taps, meta in (("quality", quality, qm),
                             ("realtime", realtime, rm)):
        binp = os.path.join(OUT, f"taps_{name}_minorder.txt")
        csvp = os.path.join(OUT, f"taps_{name}_minorder.csv")
        taps.astype(np.float32).tofile(binp)
        with open(csvp, "w") as f:
            for t in taps:
                f.write(f"{t:.17g}\n")
        print(f"  wrote {binp} ({len(taps)} float32) + {csvp}")

    out = {
        "interpolation": L, "decimation": M,
        "input_rate_hz": R_IN, "output_rate_hz": R_OUT,
        "criterion": "relative stopband |Hstop|/|Hpass| <= -70 dB, "
                     "ripple <= 0.1 dB, same passband edges",
        "profiles": [
            {"name": "quality", "passband_mhz": 330.0, **qm,
             "taps": int(len(quality))},
            {"name": "realtime", "passband_mhz": 290.0, **rm,
             "taps": int(len(realtime))},
        ],
    }
    with open(os.path.join(OUT, "design_minorder.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("wrote design_minorder.json")


if __name__ == "__main__":
    main()
