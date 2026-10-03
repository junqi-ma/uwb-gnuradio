#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
M2-A TX resampler tap design (G0 docs/twr/M2-A_G0接口与数字坐标.md §3.3).

TX direction is work -> native:
    UC200 : work 998.4e6 -> native 737.28e6   <Interp=48, Decim=65>
    CG400 : work 998.4e6 -> native 491.52e6   <Interp=32, Decim=65>

G0 §3.3 freezes:
  * taps are float32 little-endian, DC sum == Interp (48 / 32);
  * the prototype cutoff is at min(Nyquist_in, Nyquist_out), i.e. at
    virtual_rate / (2*max(Interp,Decim)); normalized to the virtual rate
    (1.0 == one virtual sample) that is 1/(2*max(L,M)) = 1/130 for both
    <48,65> and <32,65> -- the SAME normalized cutoff as <65,48>/<65,32>;
  * two independent routes must be produced and compared:
      (a) an independent design per Interp/Decim and passband/stopband;
      (b) the existing RX prototype rescaled to DC == Interp.

This script produces BOTH, records design.json, and prints sha256 of every
tap file.  It never overwrites the canonical RX assets under
testdata/resampler_65_48 / resampler_65_32.

Run (exact command recorded in design.json):
    python3 testdata/twr/m2a/taps/design_m2a_tx_taps.py

scipy is required for (a).  If scipy is unavailable the script must say so
explicitly and abort rather than silently falling back to (b) (G0 §3.3 /
task list: "do NOT silently claim (a)").
"""
import hashlib
import json
import os
import sys

import numpy as np

try:
    import scipy
    from scipy.signal import remez
except Exception as exc:  # pragma: no cover - explicit environment report
    sys.stderr.write(
        "ERROR: scipy is required for the independent design (a); "
        "could not import scipy.signal.remez: %r\n" % (exc,))
    sys.stderr.write(
        "Refusing to silently substitute the rescaled RX prototype (b).\n")
    sys.exit(2)

HERE = os.path.dirname(os.path.abspath(__file__))
# .../testdata/twr/m2a/taps -> repo root is four levels up
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))

WORK_RATE_HZ = 998.4e6
# Canonical RX prototypes (identical file content for 65/48 and 65/32).
RX_PROTO_REL = "testdata/resampler_65_48/taps_quality_minorder.txt"
RX_PROTO_SHA_EXPECTED = None  # filled at runtime

# Frozen per-direction spec.  passband/stopband are in the *virtual* rate
# domain, matching the existing RX minimum-order profiles:
#   min(Nyquist_in, Nyquist_out) = (pb + sb)/2 = R_V / (2*max(L,M)).
SPECS = [
    dict(name="tx_48_65",
         interp=48, decim=65,
         output_rate_hz=737.28e6,
         passband_hz=330.0e6, stopband_hz=407.28e6,
         profile="quality"),
    dict(name="tx_32_65",
         interp=32, decim=65,
         output_rate_hz=491.52e6,
         passband_hz=220.0e6, stopband_hz=271.52e6,
         profile="quality"),
]

# Equiripple design order and passband/stopband weight.  T is chosen equal to
# the RX minimum-order prototype length (2707) so (a) and (b) are directly
# comparable sample-by-sample.  weight=[1,2] reaches <= -70 dB relative
# stopband with margin (the RX prototype criterion).
REMEZ_TAPS = 2707
REMEZ_WEIGHT = [1.0, 2.0]
REL_STOPBAND_DB = -70.0
PASSBAND_RIPPLE_DB = 0.1


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def write_f32_le(path, taps):
    taps.astype("<f4").tofile(path)


def response_db(taps, freqs, rate_hz):
    n = np.arange(len(taps), dtype=np.float64)
    out = np.empty(len(freqs))
    for i, f in enumerate(freqs):
        out[i] = 20.0 * np.log10(
            abs(np.dot(taps, np.exp(-1j * n * 2.0 * np.pi * f / rate_hz)))
            + 1e-30)
    return out


def measure(taps, pb, sb, rate_hz):
    f = np.concatenate([np.linspace(0.0, pb, 4001),
                        np.linspace(sb, rate_hz / 2.0, 4001)])
    mag = response_db(taps, f, rate_hz)
    pb_mag = mag[:4001]
    sb_mag = mag[4001:]
    pb_gain = float(pb_mag.max())
    return {
        "passband_ripple_db": float(pb_mag.max() - pb_mag.min()),
        "relative_stopband_db": float(sb_mag.max() - pb_gain),
        "passband_gain_db": pb_gain,
        "dc_sum": float(np.sum(taps)),
    }


def design_independent(spec):
    """(a) independent equiripple design, scaled to DC == Interp."""
    L, M = spec["interp"], spec["decim"]
    R_V = L * WORK_RATE_HZ
    pb, sb = spec["passband_hz"], spec["stopband_hz"]
    cutoff = 0.5 * (pb + sb)
    max_lm = max(L, M)
    taps = remez(REMEZ_TAPS, [0.0, pb, sb, R_V / 2.0], [1.0, 0.0],
                 weight=REMEZ_WEIGHT, fs=R_V, maxiter=30)
    dc = float(np.sum(taps))
    taps = taps * (float(L) / dc)
    return {
        "taps": taps,
        "virtual_rate_hz": R_V,
        "cutoff_hz": cutoff,
        "normalized_cutoff": cutoff / R_V,
        "expected_normalized_cutoff": 1.0 / (2.0 * max_lm),
        "method": "remez_equiripple",
        "order": int(REMEZ_TAPS),
        "weight": list(REMEZ_WEIGHT),
        "design_dc_before_scale": dc,
    }


def design_rescale_rx(spec, rx_proto):
    """(b) existing RX prototype rescaled to DC == Interp."""
    L = spec["interp"]
    taps = rx_proto.astype(np.float64) * (float(L) / float(np.sum(rx_proto)))
    return {
        "taps": taps,
        "method": "rescale_rx_prototype",
        "source": RX_PROTO_REL,
        "order": int(len(taps)),
    }


def main():
    rx_proto_path = os.path.join(REPO, RX_PROTO_REL)
    if not os.path.exists(rx_proto_path):
        sys.exit("missing RX prototype: " + rx_proto_path)
    rx_proto = np.fromfile(rx_proto_path, dtype="<f4").astype(np.float64)
    rx_proto_sha = sha256_file(rx_proto_path)

    directions = []
    for spec in SPECS:
        L, M = spec["interp"], spec["decim"]
        R_V = L * WORK_RATE_HZ
        a = design_independent(spec)
        b = design_rescale_rx(spec, rx_proto)
        ma = measure(a["taps"], spec["passband_hz"], spec["stopband_hz"], R_V)
        mb = measure(b["taps"], spec["passband_hz"], spec["stopband_hz"], R_V)

        if ma["relative_stopband_db"] > REL_STOPBAND_DB:
            sys.exit("(a) %s fails stopband: %.2f dB" %
                     (spec["name"], ma["relative_stopband_db"]))
        if ma["passband_ripple_db"] > PASSBAND_RIPPLE_DB:
            sys.exit("(a) %s fails passband ripple: %.4f dB" %
                     (spec["name"], ma["passband_ripple_db"]))
        if abs(ma["dc_sum"] - L) > 1e-3:
            sys.exit("(a) %s DC sum != Interp" % spec["name"])
        if abs(mb["dc_sum"] - L) > 1e-3:
            sys.exit("(b) %s DC sum != Interp" % spec["name"])

        # frozen file = (a)
        frozen_name = "%s.f32" % spec["name"]
        frozen_path = os.path.join(HERE, frozen_name)
        write_f32_le(frozen_path, a["taps"])

        # (b) kept alongside so the comparison is reproducible
        rescale_name = "%s_rescale_rx.f32" % spec["name"]
        rescale_path = os.path.join(HERE, rescale_name)
        write_f32_le(rescale_path, b["taps"])

        same_len = len(a["taps"]) == len(b["taps"])
        if same_len:
            diff = np.abs(a["taps"] - b["taps"])
            max_abs = float(diff.max())
            peak = float(np.max(np.abs(b["taps"])))
            max_abs_over_peak = float(diff.max() / peak)
            rel_l2 = float(np.linalg.norm(a["taps"] - b["taps"]) /
                           np.linalg.norm(b["taps"]))
        else:
            max_abs = None
            max_abs_over_peak = None
            rel_l2 = None
        # impulse response: linear-phase prototype -> symmetry / peak index
        peak_index = int(np.argmax(np.abs(a["taps"])))
        symmetry_err = float(np.max(np.abs(a["taps"] -
                                           a["taps"][::-1])))

        gd_virtual = 0.5 * (a["order"] - 1)
        directions.append({
            "name": spec["name"],
            "interp": L,
            "decim": M,
            "input_rate_hz": WORK_RATE_HZ,
            "output_rate_hz": spec["output_rate_hz"],
            "virtual_rate_hz": R_V,
            "min_nyquist_hz": R_V / (2.0 * max(L, M)),
            "passband_hz": spec["passband_hz"],
            "stopband_hz": spec["stopband_hz"],
            "cutoff_hz": a["cutoff_hz"],
            "normalized_cutoff": a["normalized_cutoff"],
            "expected_normalized_cutoff": a["expected_normalized_cutoff"],
            "profile": spec["profile"],
            "criterion": ("relative stopband <= -70 dB, passband ripple "
                          "<= 0.1 dB, DC sum == Interp"),
            "method_a": a["method"],
            "order_a": a["order"],
            "weight_a": a["weight"],
            "design_dc_before_scale_a": a["design_dc_before_scale"],
            "dc_sum_a": ma["dc_sum"],
            "passband_ripple_db_a": ma["passband_ripple_db"],
            "relative_stopband_db_a": ma["relative_stopband_db"],
            "method_b": b["method"],
            "order_b": b["order"],
            "dc_sum_b": mb["dc_sum"],
            "passband_ripple_db_b": mb["passband_ripple_db"],
            "relative_stopband_db_b": mb["relative_stopband_db"],
            "group_delay_virtual_samples": gd_virtual,
            "group_delay_input_samples": gd_virtual / L,
            "group_delay_output_samples": gd_virtual / M,
            "frozen_file": frozen_name,
            "frozen_sha256": sha256_file(frozen_path),
            "rescale_rx_file": rescale_name,
            "rescale_rx_sha256": sha256_file(rescale_path),
            "peak_index_a": peak_index,
            "expected_peak_index_a": (a["order"] - 1) // 2,
            "symmetry_max_abs_err_a": symmetry_err,
            "a_vs_b": {
                "same_length": same_len,
                "max_abs_diff": max_abs,
                "max_abs_diff_over_peak": max_abs_over_peak,
                "relative_l2": rel_l2,
                "note": ("(a) independent equiripple vs (b) rescaled RX "
                         "prototype; both float32, order %d" % a["order"]),
            },
        })
        print("[%s] a: order=%d dc=%.6f ripple=%.4f dB rel_sb=%.2f dB | "
              "b: rel_sb=%.2f dB | max_abs(a-b)=%s" %
              (spec["name"], a["order"], ma["dc_sum"],
               ma["passband_ripple_db"], ma["relative_stopband_db"],
               mb["relative_stopband_db"], max_abs))
        print("  froze %s sha256=%s" %
              (frozen_name, directions[-1]["frozen_sha256"]))

    out = {
        "schema": "twr-m2a-tx-taps/1",
        "generated_by": "testdata/twr/m2a/taps/design_m2a_tx_taps.py",
        "command": "python3 testdata/twr/m2a/taps/design_m2a_tx_taps.py",
        "g0": "docs/twr/M2-A_G0接口与数字坐标.md §3.3",
        "work_rate_hz": WORK_RATE_HZ,
        "scipy_version": getattr(scipy, "__version__", None),
        "numpy_version": np.__version__,
        "rx_prototype": RX_PROTO_REL,
        "rx_prototype_sha256": rx_proto_sha,
        "rem_equiripple_order": REMEZ_TAPS,
        "rem_equiripple_weight": list(REMEZ_WEIGHT),
        "criterion": ("relative stopband <= -70 dB, passband ripple <= 0.1 dB, "
                      "DC sum == Interp"),
        "cutoff_rule": ("cutoff = virtual_rate/(2*max(Interp,Decim)); "
                        "normalized to virtual rate = 1/(2*max(L,M))"),
        "directions": directions,
    }
    with open(os.path.join(HERE, "design.json"), "w") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    print("wrote design.json")


if __name__ == "__main__":
    main()
