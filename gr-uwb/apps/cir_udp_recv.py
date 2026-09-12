#!/usr/bin/env python3
"""Receive UWB radar CIR datagrams.

x410_cg400_hrp_echo_cir.py sends a 28-byte UCR1 header plus 116
little-endian complex64 taps on every pulse, including sfd_failed (taps
are zeros).  The frequency-sweep app
(x410_cg400_hrp_echo_cir_sweep.py) sends a 44-byte UCR2 header that adds
the per-pulse centre frequency: two f64 fields ``freq_hz`` and
``freq_offset_hz``.  UCR1/UCR2 and legacy socket_pdu payloads (928 bytes
of taps, no header) are all accepted.
"""
from __future__ import annotations

import argparse
import socket
import struct
import time

import numpy as np

MAGIC = b"UCR1"
HDR = struct.Struct("<4sIHHffiI")
MAGIC_V2 = b"UCR2"
HDR_V2 = struct.Struct("<4sIHHffiIdd")
STATUS_NAME = {
    0: "ok",
    1: "sfd_failed",
    2: "timing_failed",
    3: "cir_failed",
    4: "other",
}


def parse_datagram(data):
    if len(data) >= HDR_V2.size and data[:4] == MAGIC_V2:
        (magic, pulse_id, status, tap_count, sfd, peak, peak_tap, est_us,
         freq_hz, freq_off) = HDR_V2.unpack_from(data)
        taps = np.frombuffer(data[HDR_V2.size:], dtype=np.complex64)
        return {
            "framed": True,
            "version": 2,
            "pulse_id": int(pulse_id),
            "status": STATUS_NAME.get(int(status), "other"),
            "status_code": int(status),
            "tap_count": int(tap_count),
            "sfd_metric": float(sfd),
            "peak_abs": float(peak),
            "peak_tap": int(peak_tap),
            "estimator_us": int(est_us),
            "freq_hz": float(freq_hz),
            "freq_offset_hz": float(freq_off),
            "taps": taps,
        }
    if len(data) >= HDR.size and data[:4] == MAGIC:
        magic, pulse_id, status, tap_count, sfd, peak, peak_tap, est_us = \
            HDR.unpack_from(data)
        taps = np.frombuffer(data[HDR.size:], dtype=np.complex64)
        return {
            "framed": True,
            "version": 1,
            "pulse_id": int(pulse_id),
            "status": STATUS_NAME.get(int(status), "other"),
            "status_code": int(status),
            "tap_count": int(tap_count),
            "sfd_metric": float(sfd),
            "peak_abs": float(peak),
            "peak_tap": int(peak_tap),
            "estimator_us": int(est_us),
            "freq_hz": None,
            "freq_offset_hz": None,
            "taps": taps,
        }
    taps = np.frombuffer(data, dtype=np.complex64)
    peak = int(np.argmax(np.abs(taps))) if taps.size else -1
    metric = float(np.max(np.abs(taps))) if taps.size else 0.0
    return {
        "framed": False,
        "version": 0,
        "pulse_id": -1,
        "status": "ok" if taps.size else "empty",
        "status_code": 0 if taps.size else 4,
        "tap_count": int(taps.size),
        "sfd_metric": 0.0,
        "peak_abs": metric,
        "peak_tap": peak,
        "estimator_us": 0,
        "freq_hz": None,
        "freq_offset_hz": None,
        "taps": taps,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--port", type=int, default=12345)
    p.add_argument("--expect-bytes", type=int, default=0,
                   help="0 accepts UCR1 or raw taps; >0 still checks length")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="Stop after this many seconds (0 = until Ctrl-C)")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)
    print("listening %s:%d expect_bytes=%d hdr=%d" % (
        args.bind, args.port, args.expect_bytes, HDR.size), flush=True)

    n = 0
    n_ok = 0
    n_fail = 0
    bad = 0
    t0 = time.monotonic()
    t_mark = t0
    mark_n = 0
    mark_ok = 0
    try:
        while True:
            now = time.monotonic()
            if args.seconds > 0 and (now - t0) >= args.seconds:
                break
            if now - t_mark >= 1.0:
                dt = now - t_mark
                print("rate dt=%.3f udp_hz=%.1f ok_hz=%.1f fail_hz=%.1f" % (
                    dt, (n - mark_n) / dt, (n_ok - mark_ok) / dt,
                    ((n - n_ok) - (mark_n - mark_ok)) / dt),
                      flush=True)
                t_mark = now
                mark_n = n
                mark_ok = n_ok
            try:
                data, src = sock.recvfrom(65535)
            except socket.timeout:
                continue
            n += 1
            if args.expect_bytes > 0 and len(data) != args.expect_bytes:
                bad += 1
            rec = parse_datagram(data)
            if rec["status"] == "ok":
                n_ok += 1
            else:
                n_fail += 1
            if n <= 3 or n % 100 == 0:
                freq_s = ("-" if rec["freq_hz"] is None
                          else "%.6fMHz(off%+.3fkHz)"
                          % (rec["freq_hz"] / 1e6,
                             rec["freq_offset_hz"] / 1e3))
                print("n=%d pulse=%d status=%s bytes=%d taps=%d peak_tap=%d "
                      "|peak|=%.6g sfd=%.4g freq=%s src=%s:%d framed=%s v=%d"
                      % (n, rec["pulse_id"], rec["status"], len(data),
                         rec["taps"].size, rec["peak_tap"], rec["peak_abs"],
                         rec["sfd_metric"], freq_s, src[0], src[1],
                         rec["framed"], rec["version"]),
                      flush=True)
    except KeyboardInterrupt:
        pass
    dt = time.monotonic() - t0
    rate = n / dt if dt > 0 else 0.0
    ok_hz = n_ok / dt if dt > 0 else 0.0
    print("SUMMARY recv=%d ok=%d fail=%d bad_len=%d dt_s=%.3f pps=%.2f "
          "ok_pps=%.2f" % (n, n_ok, n_fail, bad, dt, rate, ok_hz),
          flush=True)
    raise SystemExit(0 if n > 0 and bad == 0 else 3)


if __name__ == "__main__":
    main()
