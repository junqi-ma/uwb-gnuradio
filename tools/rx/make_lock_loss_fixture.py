#!/usr/bin/env python3
"""Build a deterministic "target stops transmitting" replay fixture.

Takes a paced SC16 capture, keeps `--lead-seconds`, inserts
`--gap-seconds` of digital silence, then appends the capture starting at
`--tail-start-seconds`.  With tail_start == lead + gap the packet cadence
phase is unchanged across the silence, so the lock can reacquire the same
grid.  The result exercises Locked -> Holdover -> Reacquire on a continuous
sample timeline (there is no RX_GAP marker; use a device capture for that).

This is a synthetic lock-loss fixture, not hardware evidence.

Example (the 2026-11-20 1 ms DW3000 capture):
  python3 tools/rx/make_lock_loss_fixture.py \
    --input /tmp/opencode/uwb-live-1ms/dw3000_1ms_1s.sc16 \
    --output /tmp/opencode/uwb-live-1ms/dw3000_1ms_gap50ms.sc16
"""
import argparse
import hashlib
import os
import sys

RATE = 737_280_000
CHUNK = 1 << 26


def samples_to_bytes(seconds, rate):
    return int(round(seconds * rate)) * 4


def copy_bytes(src, dst, count):
    while count > 0:
        block = src.read(min(count, CHUNK))
        if not block:
            raise SystemExit("input ended before the requested slice")
        dst.write(block)
        count -= len(block)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True, help="source SC16 capture")
    ap.add_argument("--output", required=True, help="fixture to write")
    ap.add_argument("--rate", type=int, default=RATE, help="native rate (Hz)")
    ap.add_argument("--lead-seconds", type=float, default=0.40)
    ap.add_argument("--gap-seconds", type=float, default=0.05)
    ap.add_argument("--tail-start-seconds", type=float, default=None,
                    help="offset of the appended tail (default: lead+gap)")
    ap.add_argument("--tail-seconds", type=float, default=0.35)
    ap.add_argument("--force", action="store_true", help="overwrite output")
    ns = ap.parse_args()

    tail_start = (ns.tail_start_seconds if ns.tail_start_seconds is not None
                  else ns.lead_seconds + ns.gap_seconds)
    if min(ns.lead_seconds, ns.gap_seconds, ns.tail_start_seconds or 0.0,
           ns.tail_seconds) < 0.0:
        raise SystemExit("durations must be non-negative")
    if os.path.exists(ns.output) and not ns.force:
        raise SystemExit(f"{ns.output} exists; pass --force to overwrite")

    lead = samples_to_bytes(ns.lead_seconds, ns.rate)
    gap = samples_to_bytes(ns.gap_seconds, ns.rate)
    tail = samples_to_bytes(ns.tail_seconds, ns.rate)
    tail_start = samples_to_bytes(tail_start, ns.rate)
    size = os.path.getsize(ns.input)
    if tail_start + tail > size:
        raise SystemExit(f"tail slice ends at {tail_start + tail} > input {size}")

    with open(ns.input, "rb") as src, open(ns.output, "wb") as dst:
        copy_bytes(src, dst, lead)
        zero = bytes(CHUNK)
        left = gap
        while left:
            n = min(left, len(zero))
            dst.write(zero[:n])
            left -= n
        src.seek(tail_start)
        copy_bytes(src, dst, tail)

    out_size = os.path.getsize(ns.output)
    if out_size % 4:
        # SC16 is 4 bytes/sample; never hand a partial sample to the replay.
        raise SystemExit(f"output size {out_size} is not a multiple of 4")
    h = hashlib.sha256()
    with open(ns.output, "rb") as f:
        for block in iter(lambda: f.read(CHUNK), b""):
            h.update(block)
    print(f"{ns.output}\n  bytes={out_size} samples={out_size // 4} "
          f"sha256={h.hexdigest()}")
    print(f"  lead={ns.lead_seconds}s gap={ns.gap_seconds}s "
          f"tail_start={tail_start // 4 / ns.rate}s tail={ns.tail_seconds}s")
    print("  synthetic target-stop fixture: no RX_GAP marker; replay evidence only.")


if __name__ == "__main__":
    sys.exit(main())
