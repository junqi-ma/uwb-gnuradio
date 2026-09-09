#!/usr/bin/env python3
"""Receive UWB radar CIR taps sent by network.socket_pdu (UDP).

Each datagram is little-endian complex64 taps (no metadata).  The CG400
live chain and uwb_radar_cir_udp.grc send 116 taps = 928 bytes.
"""
from __future__ import annotations

import argparse
import socket
import time

import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--port", type=int, default=12345)
    p.add_argument("--expect-bytes", type=int, default=928,
                   help="Expected payload size; 0 accepts any")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="Stop after this many seconds (0 = until Ctrl-C)")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)
    print("listening %s:%d expect_bytes=%d" % (
        args.bind, args.port, args.expect_bytes), flush=True)

    n = 0
    bad = 0
    t0 = time.monotonic()
    try:
        while True:
            if args.seconds > 0 and (time.monotonic() - t0) >= args.seconds:
                break
            try:
                data, src = sock.recvfrom(65535)
            except socket.timeout:
                continue
            n += 1
            ok = (args.expect_bytes == 0) or (len(data) == args.expect_bytes)
            if not ok:
                bad += 1
            taps = np.frombuffer(data, dtype=np.complex64)
            peak = int(np.argmax(np.abs(taps))) if taps.size else -1
            metric = float(np.max(np.abs(taps))) if taps.size else 0.0
            if n <= 3 or n % 100 == 0:
                print("n=%d src=%s:%d bytes=%d taps=%d peak_tap=%d |peak|=%.6g ok=%s"
                      % (n, src[0], src[1], len(data), taps.size, peak, metric, ok),
                      flush=True)
    except KeyboardInterrupt:
        pass
    dt = time.monotonic() - t0
    rate = n / dt if dt > 0 else 0.0
    print("SUMMARY recv=%d bad_len=%d dt_s=%.3f pps=%.2f" % (n, bad, dt, rate),
          flush=True)
    raise SystemExit(0 if n > 0 and bad == 0 else 3)


if __name__ == "__main__":
    main()
