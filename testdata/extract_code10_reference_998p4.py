#!/usr/bin/env python3
"""Extract the code-10 single-SYNC 998.4 MS/s reference template.

Contract: bit-for-bit identical to load_code10_template() in
testdata/decode_scheduled_sc16_dump.py (the shipped --dw1000 demod path):

  source testdata/uwb_code10_preamble16_payload8.cfile (complex64 @ 998.4)
  SYNC_LEN = 1016
  start = argmax(mag > 0.05 * max(mag))
  L2 normalize: tmpl / sqrt(sum(|tmpl|^2))

Output: testdata/reference_preamble_code10_998p4.cf32 (1016 x complex64).

Refuses to overwrite an existing output unless it is bit-identical
(same length and energy); use --force to override explicitly.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SRC = os.path.join(HERE, "uwb_code10_preamble16_payload8.cfile")
DEFAULT_OUT = os.path.join(HERE, "reference_preamble_code10_998p4.cf32")
SYNC_LEN = 1016  # must match decode_scheduled_sc16_dump.SYNC_LEN


def extract(src_path: str) -> tuple[np.ndarray, int]:
    x = np.fromfile(src_path, np.complex64)
    if x.size == 0:
        raise RuntimeError(f"empty source: {src_path}")
    mag = np.abs(x)
    start = int(np.argmax(mag > 0.05 * np.max(mag)))
    tmpl = x[start:start + SYNC_LEN].astype(np.complex64)
    if tmpl.size != SYNC_LEN:
        raise RuntimeError(f"code-10 template short: {tmpl.size}")
    e = float(np.sum(np.abs(tmpl) ** 2))
    if e > 0:
        tmpl = (tmpl / np.sqrt(e)).astype(np.complex64)
    return tmpl, start


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--src", default=DEFAULT_SRC)
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--force", action="store_true",
                   help="allow overwriting a differing existing output")
    args = p.parse_args()
    if not os.path.isfile(args.src):
        print(f"ERROR: missing source {args.src}", file=sys.stderr)
        return 2
    tmpl, start = extract(args.src)
    energy = float(np.sum(np.abs(tmpl) ** 2))
    if os.path.isfile(args.out) and not args.force:
        old = np.fromfile(args.out, dtype=np.complex64)
        old_e = float(np.sum(np.abs(old) ** 2)) if old.size else float("nan")
        same = (old.size == tmpl.size
                and np.array_equal(old, tmpl))
        print(f"exists {args.out}: len={old.size} energy={old_e:.9f} "
              f"identical={same}")
        if not same:
            print("REFUSE to overwrite without --force", file=sys.stderr)
            return 1
        print(f"start={start} SYNC_LEN={SYNC_LEN} "
              f"len={tmpl.size} energy={energy:.9f}")
        return 0
    tmpl.tofile(args.out)
    print(f"wrote {args.out}: start={start} SYNC_LEN={SYNC_LEN} "
          f"len={tmpl.size} energy={energy:.9f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
