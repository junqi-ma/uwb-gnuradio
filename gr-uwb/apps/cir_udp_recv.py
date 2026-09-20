#!/usr/bin/env python3
"""Receive UWB radar CIR datagrams, plot them live, and dump raw UCR4.

Both live scripts (x410_cg400_hrp_echo_cir.py and ..._jam.py) send the unified
UCR4 header followed by ``tap_count`` interleaved little-endian SC16 taps on
every CIR record.  A float32 ``cir_scale`` reconstructs FC32 as
``SC16 * cir_scale`` (per-record block-floating).  Legacy UCR3/UCR2/UCR1 and
bare raw socket_pdu payloads (unframed complex64 taps) are still accepted.

UCR4 header (little-endian, 52 bytes):
  magic "UCR4" | pulse_id u32 | status u16 | tap_count u16
  | repetition_index u16 | repetition_count u16
  | sfd_metric f32 | cir_peak_metric f32 | peak_tap i32 | estimator_us u32
  | freq_hz f64 | freq_offset_hz f64 | cir_scale f32

Two independent outputs:

1. Live matplotlib view (``--plot``, default on; ``--no-plot`` to disable):
   top    : |CIR| of all taps of the latest packet
   middle : |CIR| of the first N taps vs packet sequence number (rolling)
   bottom : rolling heatmap of |CIR| in dB normalized to the average
            first-peak power (tap index x packet sequence number)
   Reception runs in a background thread so the GUI never blocks the socket;
   the canvas is redrawn at a fixed ``--refresh`` rate.

2. Raw UCR4 dump (``--save-ucr4 PREFIX``): ``PREFIX.ucr4`` is the byte-exact
   concatenation of every received UCR4 datagram (header + SC16 payload), so
   MATLAB can re-parse ``FC32 = SC16 * cir_scale`` directly.  ``PREFIX.jsonl``
   carries one index object per record (byte_offset/byte_len + metadata).
   Non-UCR4 datagrams are counted and skipped (the stream stays self-describing).
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import struct
import threading
import time
from collections import deque

import numpy as np

MAGIC = b"UCR1"
HDR = struct.Struct("<4sIHHffiI")
MAGIC_V2 = b"UCR2"
HDR_V2 = struct.Struct("<4sIHHffiIdd")
MAGIC_V3 = b"UCR3"
HDR_V3 = struct.Struct("<4sIHHHHffiIdd")
MAGIC_V4 = b"UCR4"
HDR_V4 = struct.Struct("<4sIHHHHffiIddf")
STATUS_NAME = {
    0: "ok",
    1: "sfd_failed",
    2: "timing_failed",
    3: "cir_failed",
    4: "other",
}


def parse_datagram(data):
    if len(data) >= HDR_V4.size and data[:4] == MAGIC_V4:
        (magic, pulse_id, status, tap_count, rep_index, rep_count, sfd,
         peak, peak_tap, est_us, freq_hz, freq_off,
         cir_scale) = HDR_V4.unpack_from(data)
        payload = data[HDR_V4.size:]
        if len(payload) % 4:
            raise ValueError("UCR4 SC16 payload has an odd int16 count")
        taps_sc16 = np.frombuffer(payload, dtype="<i2")
        iq = taps_sc16.reshape(-1, 2)
        if iq.shape[0] != int(tap_count):
            raise ValueError("UCR4 tap_count does not match SC16 payload")
        taps = ((iq[:, 0].astype(np.float32) +
                 1j * iq[:, 1].astype(np.float32)) * np.float32(cir_scale))
        return {
            "framed": True, "version": 4, "sample_format": "sc16",
            "pulse_id": int(pulse_id),
            "status": STATUS_NAME.get(int(status), "other"),
            "status_code": int(status), "tap_count": int(tap_count),
            "repetition_index": (None if rep_index == 0xFFFF else int(rep_index)),
            "repetition_count": int(rep_count), "sfd_metric": float(sfd),
            "peak_abs": float(peak), "peak_tap": int(peak_tap),
            "estimator_us": int(est_us), "freq_hz": float(freq_hz),
            "freq_offset_hz": float(freq_off), "cir_scale": float(cir_scale),
            "taps_sc16": taps_sc16, "taps": taps.astype(np.complex64),
        }
    if len(data) >= HDR_V3.size and data[:4] == MAGIC_V3:
        (magic, pulse_id, status, tap_count, rep_index, rep_count, sfd,
         peak, peak_tap, est_us, freq_hz, freq_off) = HDR_V3.unpack_from(data)
        taps = np.frombuffer(data[HDR_V3.size:], dtype=np.complex64)
        return {
            "framed": True, "version": 3, "pulse_id": int(pulse_id),
            "sample_format": "fc32",
            "status": STATUS_NAME.get(int(status), "other"),
            "status_code": int(status), "tap_count": int(tap_count),
            "repetition_index": (None if rep_index == 0xFFFF else int(rep_index)),
            "repetition_count": int(rep_count), "sfd_metric": float(sfd),
            "peak_abs": float(peak), "peak_tap": int(peak_tap),
            "estimator_us": int(est_us), "freq_hz": float(freq_hz),
            "freq_offset_hz": float(freq_off), "taps": taps,
        }
    if len(data) >= HDR_V2.size and data[:4] == MAGIC_V2:
        (magic, pulse_id, status, tap_count, sfd, peak, peak_tap, est_us,
         freq_hz, freq_off) = HDR_V2.unpack_from(data)
        taps = np.frombuffer(data[HDR_V2.size:], dtype=np.complex64)
        return {
            "framed": True,
            "version": 2,
            "sample_format": "fc32",
            "pulse_id": int(pulse_id),
            "status": STATUS_NAME.get(int(status), "other"),
            "status_code": int(status),
            "tap_count": int(tap_count),
            "repetition_index": None,
            "repetition_count": 0,
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
            "sample_format": "fc32",
            "pulse_id": int(pulse_id),
            "status": STATUS_NAME.get(int(status), "other"),
            "status_code": int(status),
            "tap_count": int(tap_count),
            "repetition_index": None,
            "repetition_count": 0,
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
        "sample_format": "fc32",
        "pulse_id": -1,
        "status": "ok" if taps.size else "empty",
        "status_code": 0 if taps.size else 4,
        "tap_count": int(taps.size),
        "repetition_index": None,
        "repetition_count": 0,
        "sfd_metric": 0.0,
        "peak_abs": metric,
        "peak_tap": peak,
        "estimator_us": 0,
        "freq_hz": None,
        "freq_offset_hz": None,
        "taps": taps,
    }


def fmt_freq(rec):
    f = rec.get("freq_hz")
    if f is None or f != f:
        return "-"
    o = rec.get("freq_offset_hz")
    if o is None or o != o:
        return "%.6fMHz" % (f / 1e6)
    return "%.6fMHz(off%+.3fkHz)" % (f / 1e6, o / 1e3)


def _json_num(v):
    if v is None:
        return None
    v = float(v)
    return None if v != v else v


class Recorder:
    """Byte-exact raw UCR4 datagram dump + JSONL index.

    ``.ucr4`` is the concatenation of every received UCR4 datagram, header
    included, so the reader recovers ``FC32 = SC16 * cir_scale``.  ``.jsonl``
    records one index object per dumped datagram in the same order.  Writes
    are batched to keep the 20k+ records/s path off the syscall hot path.
    """

    FLUSH_EVERY = 256

    def __init__(self, prefix, append=False):
        self.prefix = prefix
        self.ucr4_path = prefix + ".ucr4"
        self.jsonl_path = prefix + ".jsonl"
        self.offset = 0
        self.n = 0
        self.skipped = 0
        self._since_flush = 0
        if append:
            if os.path.exists(self.ucr4_path):
                self.offset = os.path.getsize(self.ucr4_path)
            if os.path.exists(self.jsonl_path):
                with open(self.jsonl_path, "r", encoding="utf-8") as f:
                    self.n = sum(1 for line in f if line.strip())
            self.fh = open(self.ucr4_path, "ab")
            self.jh = open(self.jsonl_path, "a", encoding="utf-8")
        else:
            self.fh = open(self.ucr4_path, "wb")
            self.jh = open(self.jsonl_path, "w", encoding="utf-8")
        self.lock = threading.Lock()

    def add(self, data, rec, src, t0):
        if len(data) < 4 or data[:4] != MAGIC_V4:
            self.skipped += 1
            return
        meta = {
            "i": self.n,
            "byte_offset": self.offset,
            "byte_len": len(data),
            "t_unix": time.time(),
            "t_mono_s": time.monotonic() - t0,
            "src": "%s:%d" % (src[0], src[1]),
            "version": rec["version"],
            "pulse_id": rec["pulse_id"],
            "status": rec["status"],
            "status_code": rec["status_code"],
            "tap_count": rec["tap_count"],
            "repetition_index": rec["repetition_index"],
            "repetition_count": rec["repetition_count"],
            "sfd_metric": rec["sfd_metric"],
            "peak_abs": rec["peak_abs"],
            "peak_tap": rec["peak_tap"],
            "estimator_us": rec["estimator_us"],
            "freq_hz": _json_num(rec["freq_hz"]),
            "freq_offset_hz": _json_num(rec["freq_offset_hz"]),
            "cir_scale": rec["cir_scale"],
        }
        with self.lock:
            self.fh.write(data)
            self.jh.write(json.dumps(meta) + "\n")
            self.offset += len(data)
            self.n += 1
            self._since_flush += 1
            if self._since_flush >= self.FLUSH_EVERY:
                self.fh.flush()
                self.jh.flush()
                self._since_flush = 0

    def close(self):
        with self.lock:
            if not self.fh.closed:
                self.fh.flush()
                self.fh.close()
            if not self.jh.closed:
                self.jh.flush()
                self.jh.close()


class Stats:
    def __init__(self):
        self.n = 0
        self.bad = 0
        self.ok = 0
        self.fail = 0
        self.lock = threading.Lock()


class Hist:
    def __init__(self, maxlen):
        self.buf = deque(maxlen=maxlen)
        self.frames = 0
        self.expected_taps = None
        self.last = None


def first_peak_taps(arr, frac):
    """First significant local maximum per row (>= frac * row max)."""
    n = arr.shape[1]
    cand = ((arr >= np.roll(arr, 1, axis=1))
            & (arr >= np.roll(arr, -1, axis=1))
            & (arr >= frac * arr.max(axis=1, keepdims=True)))
    cand[:, 0] = False
    cand[:, -1] = False
    has = cand.any(axis=1)
    fp = np.argmax(cand, axis=1)
    fp[~has] = np.argmax(arr[~has], axis=1)
    return fp


def make_figure(head, db_min, db_max, plt):
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, num="CIR live view", figsize=(9, 10))
    fig.subplots_adjust(hspace=0.45)

    (line_all,) = ax1.plot([], [], lw=1.0)
    ax1.set_xlabel("tap index")
    ax1.set_ylabel("|CIR|")
    ax1.grid(True, alpha=0.3)

    head_lines = [ax2.plot([], [], lw=0.8, label="tap %d" % k)[0]
                  for k in range(max(head, 1))]
    ax2.set_xlabel("packet sequence number")
    ax2.set_ylabel("|CIR|")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", ncol=2, fontsize=6)

    img = ax3.imshow(np.full((1, 1), db_min), aspect="auto", origin="lower",
                     cmap="turbo", interpolation="nearest",
                     vmin=db_min, vmax=db_max)
    ax3.set_xlabel("tap index")
    ax3.set_ylabel("packet sequence number")
    fig.colorbar(img, ax=ax3, label="|CIR| (dB rel. first-peak avg power)")
    return fig, (ax1, ax2, ax3), line_all, head_lines, img


def recv_worker(sock, hist, stats, expect_bytes, show_failed, recorder,
                record_ok_only, t0):
    while True:
        try:
            data, src = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            rec = parse_datagram(data)
        except ValueError:
            with stats.lock:
                stats.n += 1
                stats.bad += 1
            continue
        taps = rec["taps"]
        if rec["framed"]:
            ok = taps.size > 0 and taps.size == rec["tap_count"]
        else:
            ok = taps.size > 0 and (expect_bytes == 0
                                    or len(data) == expect_bytes)
        if ok:
            if hist.expected_taps is None:
                hist.expected_taps = taps.size
            elif taps.size != hist.expected_taps:
                ok = False
        with stats.lock:
            stats.n += 1
            hist.last = rec
            if rec["status"] == "ok":
                stats.ok += 1
            else:
                stats.fail += 1
            if not ok:
                stats.bad += 1
                continue
        if recorder is not None and not (record_ok_only
                                         and rec["status"] != "ok"):
            recorder.add(data, rec, src, t0)
        if rec["status"] != "ok" and not show_failed:
            continue
        hist.buf.append(np.abs(taps).astype(np.float64))
        hist.frames += 1


def parse_args():
    p = argparse.ArgumentParser(
        description="Receive UWB radar CIR datagrams; plot live and/or dump "
                    "raw UCR4.")
    p.add_argument("--bind", default="0.0.0.0")
    p.add_argument("--port", type=int, default=12345)
    p.add_argument("--expect-bytes", type=int, default=0,
                   help="Raw-payload length check (0 = any); framed UCR4/3/2/1 "
                        "are validated against their tap_count field")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="Stop after this many seconds (0 = until Ctrl-C)")
    p.add_argument("--plot", dest="plot", action="store_true", default=True,
                   help="Live matplotlib view (default)")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="Disable the live view (headless dump)")
    p.add_argument("--history", type=int, default=200,
                   help="Rolling buffer length (packets) for figs 2/3")
    p.add_argument("--head", type=int, default=30,
                   help="Number of leading taps plotted in fig 2")
    p.add_argument("--refresh", type=float, default=0.05,
                   help="Seconds between GUI redraws (~20 fps)")
    p.add_argument("--first-peak-tap", type=int, default=-1,
                   help="Fixed first-peak tap for normalization; -1 = auto")
    p.add_argument("--first-peak-frac", type=float, default=0.5,
                   help="Auto-detect: first local max >= this fraction of max")
    p.add_argument("--show-failed", action="store_true",
                   help="Also plot frames whose status != ok (zero taps)")
    p.add_argument("--save-ucr4", default="",
                   help="Dump every raw UCR4 datagram to PREFIX.ucr4 plus a "
                        "PREFIX.jsonl index for offline MATLAB parsing")
    p.add_argument("--append", action="store_true",
                   help="Append to existing --save-ucr4 files (else overwrite)")
    p.add_argument("--record-ok-only", action="store_true",
                   help="With --save-ucr4, skip frames whose status != ok")
    p.add_argument("--db-min", type=float, default=-55.0,
                   help="Heatmap lower bound (dB rel. first-peak avg power)")
    p.add_argument("--db-max", type=float, default=0.0,
                   help="Heatmap upper bound (dB rel. first-peak avg power)")
    return p.parse_args()


def main():
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)
    print("listening %s:%d expect_bytes=%d hdr_ucr4=%d hdr_ucr3=%d "
          "hdr_ucr2=%d hdr_ucr1=%d plot=%s" % (
              args.bind, args.port, args.expect_bytes, HDR_V4.size,
              HDR_V3.size, HDR_V2.size, HDR.size, args.plot), flush=True)

    plt = None
    fig = axes = line_all = img = None
    head_lines = []
    if args.plot:
        try:
            import matplotlib.pyplot as plt  # noqa: F811
            plt.ion()
            fig, axes, line_all, head_lines, img = make_figure(
                args.head, args.db_min, args.db_max, plt)
        except Exception as e:  # missing matplotlib or no GUI backend
            print("plot disabled: %s" % e, flush=True)
            plt = None

    t0 = time.monotonic()
    rec = None
    if args.save_ucr4:
        rec = Recorder(args.save_ucr4, append=args.append)
        print("recording %s + %s%s" % (
            rec.ucr4_path, rec.jsonl_path,
            " [append]" if args.append else ""), flush=True)

    hist = Hist(args.history)
    stats = Stats()
    worker = threading.Thread(
        target=recv_worker,
        args=(sock, hist, stats, args.expect_bytes, args.show_failed, rec,
              args.record_ok_only, t0),
        daemon=True)
    worker.start()

    n_taps = None
    last_draw = 0.0
    last_report = t0
    try:
        while True:
            if args.seconds > 0 and (time.monotonic() - t0) >= args.seconds:
                break
            time.sleep(0.002)

            with stats.lock:
                n, bad, ok, fail = stats.n, stats.bad, stats.ok, stats.fail
                last = hist.last
            if n and (n <= 3 or n % 100 == 0) and n != last_report:
                last_report = n
                if last is None:
                    print("n=%d bad_len=%d ok=%d fail=%d (no frame)"
                          % (n, bad, ok, fail), flush=True)
                else:
                    lt = np.abs(last["taps"]) if last["taps"].size else None
                    peak_tap = int(np.argmax(lt)) if lt is not None else -1
                    peak = float(lt.max()) if lt is not None else 0.0
                    print("n=%d bad_len=%d ok=%d fail=%d taps=%d peak_tap=%d "
                          "|peak|=%.6g status=%s pulse=%s rep=%s/%d freq=%s"
                          % (n, bad, ok, fail, last["taps"].size, peak_tap,
                             peak, last["status"], last["pulse_id"],
                             last["repetition_index"],
                             last["repetition_count"], fmt_freq(last)),
                          flush=True)

            if plt is None or not hist.buf:
                continue
            now = time.monotonic()
            if (now - last_draw) < args.refresh:
                continue
            last_draw = now

            arr = np.asarray(hist.buf)              # (frames, n_taps)
            if n_taps is None:
                n_taps = arr.shape[1]
                axes[0].set_xlim(0, n_taps - 1)

            frame_last = hist.frames
            frame_first = frame_last - arr.shape[0] + 1
            t_axis = np.arange(frame_first, frame_last + 1)
            h = min(args.head, n_taps)
            x = np.arange(n_taps)

            latest = hist.buf[-1]
            y1max = max(float(latest.max()), 1e-9) * 1.05
            y2max = max(float(arr[:, :h].max()), 1e-9) * 1.05

            line_all.set_data(x, latest)
            axes[0].set_ylim(0, y1max)
            if last is not None:
                axes[0].set_title(
                    "pulse=%s rep=%s/%s status=%s peak_tap=%d sfd=%.4g freq=%s"
                    % (last["pulse_id"], last["repetition_index"],
                       last["repetition_count"], last["status"],
                       last["peak_tap"], last["sfd_metric"], fmt_freq(last)),
                    fontsize=9)
            for k in range(h):
                head_lines[k].set_data(t_axis, arr[:, k])
            axes[1].set_xlim(frame_first, max(frame_last, frame_first + 1))
            axes[1].set_ylim(0, y2max)
            if args.first_peak_tap >= 0:
                fp = args.first_peak_tap
            else:
                fp = first_peak_taps(arr, args.first_peak_frac)
            fp_pow = arr[np.arange(arr.shape[0]), fp] ** 2.0
            ref = max(float(np.sqrt(fp_pow.mean())), 1e-9)
            img.set_data(20.0 * np.log10(np.maximum(arr, 1e-9) / ref))
            img.set_extent((-0.5, n_taps - 0.5, frame_first, frame_last + 1))
            axes[2].set_ylim(frame_first, frame_last + 1)

            fig.canvas.draw_idle()
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        if plt is not None and fig is not None:
            plt.ioff()
            plt.close(fig)
        if rec is not None:
            rec.close()

    with stats.lock:
        n, bad, ok, fail = stats.n, stats.bad, stats.ok, stats.fail
    dt = time.monotonic() - t0
    rate = n / dt if dt > 0 else 0.0
    ok_hz = ok / dt if dt > 0 else 0.0
    print("SUMMARY recv=%d ok=%d fail=%d bad_len=%d dt_s=%.3f pps=%.2f "
          "ok_pps=%.2f" % (n, ok, fail, bad, dt, rate, ok_hz), flush=True)
    if rec is not None:
        print("RECORDED %d datagrams (%d bytes) -> %s + %s (skipped_non_ucr4=%d)"
              % (rec.n, rec.offset, rec.ucr4_path, rec.jsonl_path,
                 rec.skipped), flush=True)
    raise SystemExit(0 if n > 0 and bad == 0 else 3)


if __name__ == "__main__":
    main()
