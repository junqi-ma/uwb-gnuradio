#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Canonical 65/48 resampler anti-image filter design for 737.28 -> 998.4 MS/s.

Design rule (polyphase LPF at the virtual rate R_v = L*R_in):
  passband  [0,          B]                B = signal bandwidth we preserve
  stopband  [R_in - B,  virtual Nyquist]   first image starts at R_in - B
  transition width = R_in - 2B  (widest possible -> fewest taps for a given
  stopband requirement).  The periodic filter response then covers every
  higher image band (spaced R_in apart) automatically.

scipy convention: kaiserord() `width` is a fraction of the *sampling rate*
(cycles/sample), NOT of Nyquist.  Verifying the actual frequency response is
the source of truth here; kaiserord() is only a starting estimate.

The canonical taps are exported to binary/csv so C++ blocks and the golden
generator share one contract.  MATLAB users: run
design_resampler_737p28_to_998p4.m to cross-check against MATLAB's own
design (should match within 1e-6) and to regenerate.

Outputs (testdata/resampler_65_48/):
  taps_quality.txt  taps_realtime.txt   (float32 binary, big-endian-free)
  taps_quality.csv  taps_realtime.csv   (human readable, 17 sig digits)
  design.json                          (parameters + verified metrics)
"""
import json
import os
import sys

import numpy as np
from scipy.signal import firwin, kaiserord

R_IN = 737.28e6          # input  sample rate
R_OUT = 998.4e6          # output sample rate
L = 65                   # interpolation
M = 48                   # decimation
R_V = L * R_IN           # virtual (post zero-insert) rate = 47.9232 GHz
PASSBAND_RIPPLE_DB = 0.1
STOPBAND_ATTEN_DB = 70.0

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "resampler_65_48")


def design_filter(passband_mhz, stopband_atten_db=STOPBAND_ATTEN_DB,
                  name="profile"):
    """Design + verify a 65/48 anti-image LPF. Returns taps (scaled by L)."""
    pb = passband_mhz * 1e6
    sb = R_IN - pb                    # first image start (GHz), not Nyquist
    width_hz = sb - pb
    width_cyc = width_hz / R_V        # cycles/sample at the VIRTUAL rate
    ntaps, beta = kaiserord(stopband_atten_db, width_cyc)
    if ntaps % 2 == 0:
        ntaps += 1
    taps = firwin(ntaps, 0.5 * (pb + sb), width=width_hz,
                  window=("kaiser", beta), scale=True, fs=R_V)
    taps = taps * L                   # DC gain == interpolation == 65

    # ---- verify against spec ----
    n_f = 4096
    freqs = np.linspace(0.0, R_V / 2.0, n_f + 1)
    h = np.fft.rfft(taps, n_f * 2)
    h = h[: len(freqs)]
    mag = 20.0 * np.log10(np.abs(h) + 1e-30)
    pb_mask = freqs <= pb
    sb_mask = freqs >= sb
    pb_ripple = mag[pb_mask].max() - mag[pb_mask].min()
    sb_worst = mag[sb_mask].max()
    dc_gain = float(np.sum(taps))
    group_delay_virtual = (ntaps - 1) / 2.0          # virtual-rate samples
    gd_input = group_delay_virtual / L               # input-rate samples
    gd_output = group_delay_virtual / M              # output-rate samples

    ok = (pb_ripple <= PASSBAND_RIPPLE_DB
          and sb_worst <= -STOPBAND_ATTEN_DB
          and abs(dc_gain - L) < 1e-3)
    print(f"[{name}] B={passband_mhz:.0f} MHz taps={ntaps} beta={beta:.3f} "
          f"arms={int(np.ceil(ntaps / L))} dc={dc_gain:.4f} "
          f"pb_ripple={pb_ripple:.4f} dB sb_worst={sb_worst:.2f} dB "
          f"gd_in={gd_input:.2f} gd_out={gd_output:.2f} "
          f"{'PASS' if ok else 'FAIL'}")
    if not ok:
        raise SystemExit(f"design {name} failed spec")
    return taps, {
        "name": name,
        "passband_mhz": passband_mhz,
        "passband_ripple_db": float(pb_ripple),
        "stopband_atten_db": float(-sb_worst),
        "taps": int(ntaps),
        "beta": float(beta),
        "dc_gain": dc_gain,
        "group_delay_virtual_samples": group_delay_virtual,
        "group_delay_input_samples": gd_input,
        "group_delay_output_samples": gd_output,
    }


def export(name, taps, meta):
    os.makedirs(OUT_DIR, exist_ok=True)
    binpath = os.path.join(OUT_DIR, f"taps_{name}.txt")
    csvpath = os.path.join(OUT_DIR, f"taps_{name}.csv")
    taps.astype(np.float32).tofile(binpath)
    with open(csvpath, "w") as f:
        for t in taps:
            f.write(f"{t:.17g}\n")
    print(f"  wrote {binpath} ({len(taps)} float32) + {csvpath}")
    return meta


def main():
    quality, qmeta = design_filter(330.0, name="quality")
    realtime, rmeta = design_filter(290.0, name="realtime")
    out = {
        "interpolation": L,
        "decimation": M,
        "input_rate_hz": R_IN,
        "output_rate_hz": R_OUT,
        "virtual_rate_hz": R_V,
        "passband_ripple_db_spec": PASSBAND_RIPPLE_DB,
        "stopband_atten_db_spec": STOPBAND_ATTEN_DB,
        "profiles": [export("quality", quality, qmeta),
                     export("realtime", realtime, rmeta)],
    }
    with open(os.path.join(OUT_DIR, "design.json"), "w") as f:
        json.dump(out, f, indent=2)
    print("wrote design.json")


if __name__ == "__main__":
    main()
