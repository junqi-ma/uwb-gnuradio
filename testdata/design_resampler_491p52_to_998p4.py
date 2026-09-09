#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
65/32 anti-image filter for 491.52 -> 998.4 MS/s (X410 CG400).

Same relative-stopband rule as testdata/design_resampler_minorder.py:
  passband ripple      <= 0.1 dB
  |H(stop)|/|H(pass)|  <= -70 dB
  DC gain (sum)        = 65

Passbands stay inside the 491.52 MS/s Nyquist (245.76 MHz):
  quality  B=220 MHz
  realtime B=180 MHz

Outputs testdata/resampler_65_32/.
"""
import json
import os

import numpy as np
from scipy.signal import firwin, kaiserord

R_IN = 491.52e6
R_OUT = 998.4e6
L, M = 65, 32
R_V = L * R_IN
PASSBAND_RIPPLE_DB = 0.1
REL_STOPBAND_DB = -70.0
NYQUIST_MHZ = R_IN / 2e6

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "resampler_65_32")


def response(taps, freqs):
    w = 2 * np.pi * np.asarray(freqs) / R_V
    return np.array([np.dot(taps, np.exp(-1j * np.arange(len(taps)) * wi))
                     for wi in w])


def meets_spec(taps, pb, sb):
    f = np.concatenate([np.linspace(0, pb, 4001),
                        np.linspace(sb, min(R_IN, R_V / 2), 4001)])
    h = response(taps, f)
    mag = 20 * np.log10(np.abs(h) + 1e-30)
    n_pb = 4001
    pb_mag = mag[:n_pb]
    sb_mag = mag[n_pb:]
    pb_gain = pb_mag.max()
    ripple = pb_mag.max() - pb_mag.min()
    rel_sb = sb_mag.max() - pb_gain
    dc = float(np.sum(taps))
    met = (ripple <= PASSBAND_RIPPLE_DB and rel_sb <= REL_STOPBAND_DB
           and abs(dc - L) < 1e-3)
    return met, {"ripple_db": float(ripple), "rel_sb_db": float(rel_sb),
                 "pb_gain_db": float(pb_gain), "dc": dc}


def design_min(passband_mhz, name):
    if passband_mhz >= NYQUIST_MHZ:
        raise SystemExit("%s: passband %.1f MHz exceeds Nyquist %.2f MHz"
                         % (name, passband_mhz, NYQUIST_MHZ))
    pb = passband_mhz * 1e6
    sb = R_IN - pb
    width_hz = sb - pb
    width_nyq = width_hz / (R_V / 2.0)
    ntaps, beta = kaiserord(-REL_STOPBAND_DB, width_nyq)
    if ntaps % 2 == 0:
        ntaps += 1
    cutoff = 0.5 * (pb + sb)
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
        n += 2
    if best is None:
        raise SystemExit("%s: no design found meeting spec" % name)
    gd_out = (len(best) - 1) / 2 / M
    print("[%s] B=%s taps=%d arms=%d ripple=%.4f dB rel_sb=%.2f dB "
          "dc=%.4f gd_out=%.2f (Kaiser est %d)"
          % (name, passband_mhz, len(best), int(np.ceil(len(best) / L)),
             best_meta["ripple_db"], best_meta["rel_sb_db"], best_meta["dc"],
             gd_out, ntaps))
    return best, best_meta


def main():
    os.makedirs(OUT, exist_ok=True)
    quality, qm = design_min(220.0, "quality")
    realtime, rm = design_min(180.0, "realtime")

    for name, taps, meta in (("quality", quality, qm),
                             ("realtime", realtime, rm)):
        binp = os.path.join(OUT, "taps_%s_minorder.txt" % name)
        csvp = os.path.join(OUT, "taps_%s_minorder.csv" % name)
        taps.astype(np.float32).tofile(binp)
        with open(csvp, "w") as f:
            for t in taps:
                f.write("%.17g\n" % t)
        print("  wrote %s (%d float32)" % (binp, len(taps)))

    out = {
        "interpolation": L,
        "decimation": M,
        "input_rate_hz": R_IN,
        "output_rate_hz": R_OUT,
        "virtual_rate_hz": R_V,
        "nyquist_mhz": NYQUIST_MHZ,
        "uwb_occupancy_mhz": 249.6,
        "nyquist_note": (
            "491.52 MS/s Nyquist is 245.76 MHz; UWB occupancy is ~249.6 MHz. "
            "Band edges are a hardware limit of CG400, not recovered by 65/32."
        ),
        "criterion": "relative stopband |Hstop|/|Hpass| <= -70 dB, "
                     "ripple <= 0.1 dB, passband inside 245.76 MHz Nyquist",
        "profiles": [
            {"name": "quality", "passband_mhz": 220.0, **qm,
             "taps": int(len(quality))},
            {"name": "realtime", "passband_mhz": 180.0, **rm,
             "taps": int(len(realtime))},
        ],
    }
    with open(os.path.join(OUT, "design_minorder.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("wrote design_minorder.json")


if __name__ == "__main__":
    main()
