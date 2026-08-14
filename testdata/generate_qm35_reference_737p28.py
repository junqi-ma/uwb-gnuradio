#!/usr/bin/env python3
"""Generate the production 737.28 MS/s QM35 code-9 SYNC template.

Contract (docs/phase1/GROK_X410_QM35盲捕获与自动周期锁定开发方案.md §8):

  testdata/reference_preamble.bin          # 998.4 MS/s, one SYNC, L2-normalized
        -- resample_poly 48/65 -->
  testdata/reference_preamble_code9_737p28.cf32
  testdata/reference_preamble_code9_737p28_metadata.json

Must follow UWB_demodulation/+uwbdecoder/buildUwbReference.m:

  * source waveform is one SYNC symbol
  * energy-normalize with ||x||_2 (MATLAB: x / (norm(x)+eps))
  * sample-index base 0 (template[0] aligns with the first chip sample)

Anti-aliasing is scipy.signal.resample_poly (FIR, Kaiser beta=5, half-length
10 * max(up, down)), the same family of polyphase convention used by the
65/48 host resampler. Group delay is recorded so C++ / MATLAB comparisons
can apply a documented 1-sample-or-filter-delay tolerance.

This script is the generator of record when MATLAB is not available.
The companion .m file implements the same contract.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy.signal import resample_poly

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SRC = os.path.join(HERE, "reference_preamble.bin")
DEFAULT_OUT = os.path.join(HERE, "reference_preamble_code9_737p28.cf32")
DEFAULT_META = os.path.join(HERE, "reference_preamble_code9_737p28_metadata.json")

INPUT_RATE_HZ = 998.4e6
OUTPUT_RATE_HZ = 737.28e6
INTERP = 48
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
    # Source is already L2-normalized by make_reference_template.py / MATLAB.
    native = resample_poly(src, INTERP, DECIM, window=("kaiser", 5.0))
    native = np.asarray(native, dtype=np.complex64)
    gd = resample_group_delay_samples(INTERP, DECIM, RESAMPLE_HALF_LENGTH)
    native_unnorm_energy = float(np.sum(np.abs(native) ** 2))
    native = l2_normalize(native)
    native_energy = float(np.sum(np.abs(native) ** 2))
    native.tofile(out_path)
    meta = {
        "description": (
            "One QM35 / IEEE 802.15.4a HRP code-9 SYNC symbol at the X410 "
            "native rate 737.28 MS/s. Derived from the 998.4 MS/s "
            "buildUwbReference.m template by polyphase 48/65 resample."
        ),
        "code_index": 9,
        "preamble_repetitions": 64,
        "sfd_mode": "4z2",
        "input_file": os.path.relpath(src_path, HERE),
        "output_file": os.path.relpath(out_path, HERE),
        "input_rate_hz": INPUT_RATE_HZ,
        "output_rate_hz": OUTPUT_RATE_HZ,
        "interp": INTERP,
        "decim": DECIM,
        "resample_window": ["kaiser", 5.0],
        "resample_half_length": RESAMPLE_HALF_LENGTH,
        "source_length": int(src.size),
        "source_energy": src_energy,
        "template_length": int(native.size),
        "energy": native_energy,
        "energy_before_normalize": native_unnorm_energy,
        "group_delay_samples": gd,
        "group_delay_domain": "native_737p28",
        "sample_index_base": 0,
        "dtype": "complex64",
        "filter_delay_tolerance_samples": 1,
        "generator": "testdata/generate_qm35_reference_737p28.py",
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
