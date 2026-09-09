#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One-shot 32/65 downsample of the canonical 998.4 TX packet to 491.52 MS/s.

Uses scipy.signal.resample_poly (Kaiser anti-alias), matching the 737.28
path's one-shot resample(x, P, Q) of the *entire* packet.  Does not
repeat the 751-sample detector template.
"""
import json
import os

import numpy as np
from scipy.signal import resample_poly

HERE = os.path.dirname(os.path.abspath(__file__))
TX_998 = os.path.join(HERE, "tx_998p4.cf32")
TX_491 = os.path.join(HERE, "tx_491p52.cf32")
META = os.path.join(HERE, "metadata.json")


def main():
    x = np.fromfile(TX_998, dtype=np.complex64)
    y = resample_poly(x, 32, 65).astype(np.complex64)
    y.tofile(TX_491)
    print("tx_998p4=%d -> tx_491p52=%d (ceil(N*32/65)=%d)"
          % (len(x), len(y), int(np.ceil(len(x) * 32 / 65))))

    with open(META, "r") as f:
        meta = json.load(f)
    meta["rate_native_cg400_hz"] = 491520000.0
    meta["tx_length_491p52"] = int(len(y))
    meta["tx_491p52_peak"] = float(np.max(np.abs(y)))
    meta["tx_491p52_energy"] = float(np.sum(np.abs(y) ** 2))
    meta["files"]["tx_491p52_cf32"] = int(len(y))
    note = meta.get("note", "")
    extra = (" CG400 native TX is scipy resample_poly(x,32,65) of the same "
             "998.4 packet (testdata/uwb_radar/export_tx_491p52.py).")
    if "CG400" not in note:
        meta["note"] = note + extra
    with open(META, "w") as f:
        json.dump(meta, f, indent=2)
        f.write("\n")
    print("updated", META)


if __name__ == "__main__":
    main()
