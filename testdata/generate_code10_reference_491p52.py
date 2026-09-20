#!/usr/bin/env python3
"""Generate the production 491.52 MS/s DW1000 code-10 SYNC template.

Contract (mirrors testdata/generate_qm35_reference_491p52.py with
INTERP=32, DECIM=65, resample_poly Kaiser beta=5.0, half-length
10 * max(up, down), L2 normalize, group-delay record):

  testdata/reference_preamble_code10_998p4.cf32   # 998.4 MS/s, one SYNC
        -- resample_poly 32/65 -->
  testdata/reference_preamble_code10_491p52.cf32
  testdata/reference_preamble_code10_491p52_metadata.json

Must follow UWB_demodulation/+uwbdecoder/buildUwbReference.m:

  * source waveform is one SYNC symbol (1016 samples @ 998.4 MS/s)
  * energy-normalize with ||x||_2 (MATLAB: x / (norm(x)+eps))
  * sample-index base 0 (template[0] aligns with the first chip sample)

The 998.4 MS/s source is extracted bit-for-bit identically to
load_code10_template() in testdata/decode_scheduled_sc16_dump.py
(the shipped --dw1000 demod path: code 10 / decawave SFD) via
testdata/extract_code10_reference_998p4.py.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy.signal import resample_poly

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SRC = os.path.join(HERE, "reference_preamble_code10_998p4.cf32")
DEFAULT_OUT = os.path.join(HERE, "reference_preamble_code10_491p52.cf32")
DEFAULT_META = os.path.join(
    HERE, "reference_preamble_code10_491p52_metadata.json")

INPUT_RATE_HZ = 998.4e6
OUTPUT_RATE_HZ = 491.52e6
INTERP = 32
DECIM = 65
# scipy.signal.resample_poly default: numtaps ~= 2 * n + 1 with n = 10 * max(up, down)
RESAMPLE_HALF_LENGTH = 10


def l2_normalize(x: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(x))
    if n <= 0.0:
        return x.astype(np.complex64, copy=False)
    return (x / n).astype(np.complex64)


def resample_group_delay_samples(up: int, down: int, half: int) -> float:
    """Output-rate group delay of resample_poly's default FIR.

    The prototype FIR has length 2*half*max(up,down)+1 and is applied at the
    upsampled rate. After decimation the delay in output samples is
    ((ntaps-1)/2) / down.
    """
    ntaps = 2 * half * max(up, down) + 1
    return float(ntaps - 1) / 2.0 / float(down)


def generate(src_path: str, out_path: str, meta_path: str) -> dict:
    src = np.fromfile(src_path, dtype=np.complex64)
    if src.size == 0:
        raise RuntimeError(f"empty source template: {src_path}")
    src_energy = float(np.sum(np.abs(src) ** 2))
    src_norm = float(np.linalg.norm(src))
    # Source is already L2-normalized by extract_code10_reference_998p4.py.
    native = resample_poly(src, INTERP, DECIM, window=("kaiser", 5.0))
    native = np.asarray(native, dtype=np.complex64)
    gd = resample_group_delay_samples(INTERP, DECIM, RESAMPLE_HALF_LENGTH)
    native_unnorm_energy = float(np.sum(np.abs(native) ** 2))
    native_unnorm_norm = float(np.linalg.norm(native))
    native = l2_normalize(native)
    native_energy = float(np.sum(np.abs(native) ** 2))
    native.tofile(out_path)
    meta = {
        "description": (
            "One DW1000 / IEEE 802.15.4a HRP code-10 SYNC symbol at the X410 "
            "native rate 491.52 MS/s. Derived from the 998.4 MS/s "
            "single-SYNC template (extracted like load_code10_template in "
            "decode_scheduled_sc16_dump.py) by polyphase 32/65 resample."
        ),
        "code_index": 10,
        "preamble_repetitions": 16,
        "sfd_mode": "decawave",
        "input_file": os.path.relpath(src_path, HERE),
        "output_file": os.path.relpath(out_path, HERE),
        # Aliases requested by the 491.52 MHz task contract.
        "source": os.path.relpath(src_path, HERE),
        "input_rate_hz": INPUT_RATE_HZ,
        "input_rate": INPUT_RATE_HZ,
        "output_rate_hz": OUTPUT_RATE_HZ,
        "output_rate": OUTPUT_RATE_HZ,
        "interp": INTERP,
        "decim": DECIM,
        "resample_window": ["kaiser", 5.0],
        "resample_half_length": RESAMPLE_HALF_LENGTH,
        "source_length": int(src.size),
        "source_energy": src_energy,
        "src_energy": src_energy,
        "norm": src_norm,
        "template_length": int(native.size),
        "energy": native_energy,
        "native_energy": native_energy,
        "energy_before_normalize": native_unnorm_energy,
        "native_unnorm_energy": native_unnorm_energy,
        "native_unnorm_norm": native_unnorm_norm,
        "group_delay_samples": gd,
        "group_delay_out_samples": gd,
        "group_delay_domain": "native_491p52",
        "sample_index_base": 0,
        "dtype": "complex64",
        "filter_delay_tolerance_samples": 1,
        "generator": "testdata/generate_code10_reference_491p52.py",
        "matlab_reference": "UWB_demodulation/+uwbdecoder/buildUwbReference.m",
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, sort_keys=True)
        f.write("\n")
    return meta


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--src", default=DEFAULT_SRC)
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--meta", default=DEFAULT_META)
    args = p.parse_args()
    meta = generate(args.src, args.out, args.meta)
    print(
        "wrote {out}: length={n} energy={e:.9f} group_delay={gd:.6f}".format(
            out=args.out,
            n=meta["template_length"],
            e=meta["energy"],
            gd=meta["group_delay_samples"],
        )
    )
    print("wrote", args.meta)
    return 0


if __name__ == "__main__":
    sys.exit(main())
