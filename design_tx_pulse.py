#!/usr/bin/env python3
"""Design low-tail TX pulse cores at the 998.4 MS/s work grid.

For the fixed 491.52 MS/s CG400 link: a raised-cosine spectrum ending at
f2 (< 245.76 MHz native Nyquist) removes the band-edge alias/ringing of the
legacy Butterworth-fit pulse, and a minimum-phase realization moves the
remaining pre-cursor tail behind the pulse (see
docs/固定491p52采样率_发射脉冲低拖尾方案.md).

With no arguments this regenerates every shipped core under
testdata/uwb_hrp_tx/ (see PRESETS): the 215 MHz min-phase/linear comparison
cores, the 160-240 min-phase/linear cores and the truncated linear-phase
default.  Pass `--f1-mhz` for a one-off custom design instead.

Outputs raw float32 taps plus a json sidecar under testdata/uwb_hrp_tx/.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np
from scipy.signal import firwin2, minimum_phase, resample, resample_poly

FS_WORK = 998.4e6
FS_NATIVE = 491.52e6
HERE = os.path.dirname(os.path.abspath(__file__))


def design_rc(f1_hz, f2_hz, prototype_taps):
    h = firwin2(prototype_taps, [0, f1_hz, f2_hz, FS_WORK / 2],
                [1, 1, 0, 0], fs=FS_WORK)
    return h


def metrics(taps, phases=range(0, 65, 4), interp=16):
    """Worst-phase envelope tails of one chip pulse after 32/65."""
    pre = post = -999.0
    width = 0.0
    for off in phases:
        pad = 32768
        x = np.zeros(len(taps) + 2 * pad)
        x[pad + off] = 1.0
        z = resample_poly(np.convolve(x, taps)[:len(x)], 32, 65)
        zf = np.abs(resample(z, len(z) * interp))
        p = int(np.argmax(zf))
        pk = zf[p]
        lag = (np.arange(len(zf)) - p) / (FS_NATIVE * interp) * 1e9
        pre = max(pre, 20 * np.log10(max(zf[(lag >= -100) & (lag <= -10)].max(),
                                        1e-30) / pk))
        post = max(post, 20 * np.log10(max(zf[(lag >= 10) & (lag <= 100)].max(),
                                         1e-30) / pk))
        half = pk / np.sqrt(2.0)
        i = j = p
        while i > 0 and zf[i] > half:
            i -= 1
        while j < len(zf) - 1 and zf[j] > half:
            j += 1
        width = max(width, (j - i) / (FS_NATIVE * interp) * 1e9)
    return pre, post, width


def alias_db(taps):
    n = 1 << 16
    K = np.abs(np.fft.rfft(taps, n)) ** 2
    f = np.fft.rfftfreq(n, 1.0 / FS_WORK)
    return 10 * np.log10(max(K[f >= 245.76e6].sum(), 1e-300) / K.sum())


def write_pulse(outdir, name, taps, meta):
    os.makedirs(outdir, exist_ok=True)
    f32 = os.path.join(outdir, name + ".f32")
    np.asarray(taps, dtype=np.float32).tofile(f32)
    meta = dict(meta)
    meta["file"] = name + ".f32"
    meta["taps"] = int(len(taps))
    meta["center_tap"] = int(np.argmax(np.abs(taps)))
    meta["created"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    json_path = os.path.join(outdir, name + ".json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
        f.write("\n")
    return f32, json_path


def process(outdir, f1_mhz, f2_mhz, prototype, min_phase):
    h = design_rc(f1_mhz * 1e6, f2_mhz * 1e6, prototype)
    if min_phase:
        h = minimum_phase(h, method="homomorphic")
    pre, post, width = metrics(h)
    ab = alias_db(h)
    tag = ("minphase" if min_phase else "linear") + \
        "_rc%d_%d" % (round(f1_mhz), round(f2_mhz))
    meta = {
        "description": "HRP TX pulse core, raised-cosine spectrum",
        "phase": "minimum" if min_phase else "linear",
        "f1_mhz": f1_mhz,
        "f2_mhz": f2_mhz,
        "prototype_taps": prototype if min_phase else len(h),
        "two_sided_bw_mhz": 2.0 * f2_mhz,
        "pre_10_100_db": round(pre, 1),
        "post_10_100_db": round(post, 1),
        "width_3db_ns": round(width, 2),
        "alias_above_245p76_db": round(ab, 1),
    }
    f32, jp = write_pulse(outdir, "pulse_" + tag, h, meta)
    print("%-28s taps=%3d pre %6.1f post %6.1f w3dB %.2f ns alias %6.1f dB -> %s"
          % (tag, len(h), pre, post, width, ab, os.path.basename(f32)))


def truncation_f1(f2_hz, target_3db_hz):
    """Linear-transition f1 so the prototype -3 dB sits at target_3db_hz."""
    x = 1.0 - 1.0 / np.sqrt(2.0)
    return (target_3db_hz - x * f2_hz) / (1.0 - x)


def process_truncated(outdir, f2_mhz, prototype, align, keep_taps,
                      target_3db_mhz):
    """Linear-phase RC prototype, truncated so its peak sits at `align`."""
    f1 = truncation_f1(f2_mhz * 1e6, target_3db_mhz * 1e6)
    h = design_rc(f1, f2_mhz * 1e6, prototype)
    pk = int(np.argmax(np.abs(h)))
    if pk < align or pk - align + keep_taps > len(h):
        raise SystemExit("truncation window outside prototype")
    taps = h[pk - align:pk - align + keep_taps].copy()
    pre, post, width = metrics(taps)
    ab = alias_db(taps)
    tag = "trunc_linear_rc%d_%d" % (round(f1 / 1e6), round(f2_mhz))
    meta = {
        "description": "HRP TX pulse core, linear-phase RC, truncated",
        "phase": "linear-truncated",
        "target_3db_mhz": target_3db_mhz,
        "f1_mhz": round(f1 / 1e6, 2),
        "f2_mhz": f2_mhz,
        "prototype_taps": prototype,
        "keep_taps": keep_taps,
        "align_taps": align,
        "two_sided_bw_mhz": 2.0 * f2_mhz,
        "pre_10_100_db": round(pre, 1),
        "post_10_100_db": round(post, 1),
        "width_3db_ns": round(width, 2),
        "alias_above_245p76_db": round(ab, 1),
    }
    f32, jp = write_pulse(outdir, "pulse_" + tag, taps, meta)
    print("%-28s taps=%3d pre %6.1f post %6.1f w3dB %.2f ns alias %6.1f dB -> %s"
          % (tag, len(taps), pre, post, width, ab, os.path.basename(f32)))


PRESETS = [
    # (f1_mhz, f2_mhz, prototype_taps)
    (100.0, 215.0, 513),
    (120.0, 215.0, 513),
    (160.0, 240.0, 2049),
]


def process_presets(outdir, trunc_align, trunc_keep_taps, trunc_3db_mhz):
    for f1, f2, proto in PRESETS:
        process(outdir, f1, f2, proto, True)
    for f1, f2, proto in PRESETS:
        process(outdir, f1, f2, proto, False)
    process_truncated(outdir, PRESETS[-1][1], PRESETS[-1][2], trunc_align,
                      trunc_keep_taps, trunc_3db_mhz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outdir",
                    default=os.path.join(HERE, "testdata", "uwb_hrp_tx"))
    ap.add_argument("--f1-mhz", type=float, nargs="+", default=None,
                    help="custom passband edge(s); omit to regenerate the "
                         "shipped PRESETS")
    ap.add_argument("--f2-mhz", type=float, default=240.0)
    ap.add_argument("--prototype-taps", type=int, default=2049)
    ap.add_argument("--trunc-align", type=int, default=32)
    ap.add_argument("--trunc-keep-taps", type=int, default=1025)
    ap.add_argument("--trunc-3db-mhz", type=float, default=200.0)
    a = ap.parse_args()
    if a.f1_mhz is None:
        process_presets(a.outdir, a.trunc_align, a.trunc_keep_taps,
                        a.trunc_3db_mhz)
        return
    for f1 in a.f1_mhz:
        process(a.outdir, f1, a.f2_mhz, a.prototype_taps, True)
    for f1 in a.f1_mhz:
        process(a.outdir, f1, a.f2_mhz, a.prototype_taps, False)
    if a.prototype_taps >= a.trunc_align + a.trunc_keep_taps:
        process_truncated(a.outdir, a.f2_mhz, a.prototype_taps, a.trunc_align,
                          a.trunc_keep_taps, a.trunc_3db_mhz)


if __name__ == "__main__":
    main()
