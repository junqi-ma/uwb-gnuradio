#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deterministic INPUT generator for the M2-A MATLAB oracle.

WHAT THIS IS
------------
This script writes the *input* vectors (CF32) that both the C++ resampler and
the MATLAB reference scripts must consume bit-for-bit.  It generates INPUTS
ONLY.  It never computes a resampler output, never decodes anything and never
stands in for a MATLAB result.  See ../README.md and ../manifest.json:

    matlab_executed = false

The MATLAB scripts (matlab_resample_reference.m, matlab_decode_cpp_iq.m,
matlab_generate_twr_frame.m) are the oracle and MUST be run on a machine with
MATLAB + Communications Toolbox.  The presence of this generator does not
satisfy any MATLAB acceptance item.

Provenance / reproducibility:
    - impulse / zeros / random use numpy.random.default_rng(20261003) so the
      bytes are fixed by this file and its seed.
    - twr_waveform_in.cf32 is a fixed 4096-sample window (0-based offset
      4992000) of the repository's frozen work-grid golden
      testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile
      (998.4 MS/s, code 9, 64 SYNC, standard IEEE SFD), peak-normalised to 1.0.
      It is a real work-grid UWB waveform, NOT a TWR frame golden; the TWR
      frame golden is produced independently by matlab_generate_twr_frame.m.

Run (from the repository root):
    python3 testdata/twr/m2a/matlab/gen_golden_inputs.py
"""

import hashlib
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
INPUTS = os.path.join(HERE, "inputs")

TWR_WORK_CFILE = os.path.join(
    REPO, "testdata",
    "uwb_code9_preamble64_payload128_standard_sfd.cfile")
TWR_WINDOW_0BASED = 4992000
TWR_WINDOW_LEN = 4096


def write_cf32(path, z):
    z = np.asarray(z, dtype=np.complex64).reshape(-1)
    inter = np.empty(2 * z.size, dtype=np.float32)
    inter[0::2] = z.real
    inter[1::2] = z.imag
    inter.tofile(path)
    return z.size


def read_cf32(path):
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 2:
        raise SystemExit("interleaved float32 file has odd length: " + path)
    return raw[0::2].astype(np.float64) + 1j * raw[1::2].astype(np.float64)


def main():
    os.makedirs(INPUTS, exist_ok=True)
    manifest = {}

    # --- impulse: N=64, first sample = 1, rest exactly 0 ---------------------
    imp = np.zeros(64, dtype=np.complex128)
    imp[0] = 1.0 + 0.0j
    n = write_cf32(os.path.join(INPUTS, "impulse_in.cf32"), imp)
    manifest["impulse_in.cf32"] = {"n": n, "peak": 1.0}

    # --- zeros: N=256, exact zeros (all-zero contract) ----------------------
    zeros = np.zeros(256, dtype=np.complex128)
    n = write_cf32(os.path.join(INPUTS, "zeros_in.cf32"), zeros)
    manifest["zeros_in.cf32"] = {"n": n, "peak": 0.0}

    # --- random: N=4096, complex Gaussian, unit average power ---------------
    rng = np.random.default_rng(20261003)
    z = (rng.standard_normal(4096) + 1j * rng.standard_normal(4096))
    z = z / np.sqrt(np.mean(np.abs(z) ** 2))
    n = write_cf32(os.path.join(INPUTS, "random_in.cf32"), z)
    manifest["random_in.cf32"] = {
        "n": n, "seed": 20261003, "normalisation": "unit average power"}

    # --- twr waveform: real work-grid window, peak-normalised to 1 ----------
    if not os.path.exists(TWR_WORK_CFILE):
        raise SystemExit("missing frozen work cfile: " + TWR_WORK_CFILE)
    x = read_cf32(TWR_WORK_CFILE)
    if TWR_WINDOW_0BASED + TWR_WINDOW_LEN > x.size:
        raise SystemExit("work cfile shorter than the fixed window")
    win = x[TWR_WINDOW_0BASED:TWR_WINDOW_0BASED + TWR_WINDOW_LEN]
    win = win / np.max(np.abs(win))
    n = write_cf32(os.path.join(INPUTS, "twr_waveform_in.cf32"), win)
    manifest["twr_waveform_in.cf32"] = {
        "n": n,
        "source": os.path.relpath(TWR_WORK_CFILE, REPO),
        "source_sha256": hashlib.sha256(
            open(TWR_WORK_CFILE, "rb").read()).hexdigest(),
        "window_0based": TWR_WINDOW_0BASED,
        "window_len": TWR_WINDOW_LEN,
        "normalisation": "peak 1.0"}

    for name in sorted(manifest):
        p = os.path.join(INPUTS, name)
        h = hashlib.sha256(open(p, "rb").read()).hexdigest()
        print("%s  n=%d  sha256=%s" % (name, manifest[name]["n"], h))


if __name__ == "__main__":
    main()
