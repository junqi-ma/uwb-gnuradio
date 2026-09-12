#!/usr/bin/env python3
"""Live HRP synth + timed X410 TX/RX + PDU 65/32 + radar CIR.

Pipeline:
  UwbHrpPacketSource.samples()     # 998.4 CF32, C++ IEEE 802.15.4a BPRF
    -> TimedUhdEcho                # 32/65 downsample once, SC16 timed burst
    -> PDU 65/32                   # 491.52 -> 998.4
    -> UwbRadarCirEstimator        # default: predicted TX time, no SFD gate
    -> UwbCirWriter + framed UDP

TX: UHD ch0 / TX/RX0 (front-panel ch1)
RX: UHD ch3 / RX1     (front-panel ch4)
Rate: CG400 491.52 MS/s

Timed echo locks SFD to a constant offset from the RX window
(predicted_sfd is identical every burst).  CIR uses that schedule, not
a detected SFD, so DW3000 collisions do not drop frames.  Pass
--require-sfd to restore the old search gate.

UDP is a non-blocking UCR1 header + 116 taps on every pulse.  Live
lines report echo_ok_hz vs cir_ok_hz vs udp_hz; radio ok is not CIR ok.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import struct
import sys
import threading
import time

import numpy as np
from scipy.signal import resample_poly

import glob
import importlib.util

from gnuradio import gr
import pmt


def find_repo_root():
    cur = os.path.abspath(os.path.dirname(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(cur, "testdata", "resampler_65_32")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise SystemExit("cannot find repo root from %s" % __file__)


def bootstrap_uhd_env():
    """Debian python3 skips site-packages; UHD 4.6 + DPDK AVX-512 SIGILL.

    Re-exec once with PYTHONPATH and the patched rte libs so `import uhd`
    works from a bare `python3` invocation.
    """
    if os.environ.get("UWB_UHD_BOOTSTRAPPED") == "1":
        return
    repo = find_repo_root()
    site = "/usr/local/lib/python3.10/site-packages"
    eal = "/tmp/uhd_eal_noret"
    build_lib = os.path.join(repo, "gr-uwb", "build", "lib")
    env = os.environ.copy()
    changed = False

    def prepend(key, path):
        nonlocal changed
        if not path or not os.path.isdir(path):
            return
        cur = env.get(key, "")
        parts = [p for p in cur.split(":") if p]
        if path in parts:
            return
        env[key] = path if not cur else (path + ":" + cur)
        changed = True

    prepend("LD_LIBRARY_PATH", build_lib)
    prepend("LD_LIBRARY_PATH", eal)
    prepend("PYTHONPATH", site)
    if not changed:
        return
    env["UWB_UHD_BOOTSTRAPPED"] = "1"
    os.execvpe(sys.executable, [sys.executable, "-u"] + sys.argv, env)


def load_uwb():
    repo = find_repo_root()
    bindir = os.path.join(repo, "gr-uwb", "build", "python", "uwb", "bindings")
    matches = sorted(glob.glob(os.path.join(bindir, "uwb_python*.so")))
    if not matches:
        raise SystemExit("missing %s/uwb_python*.so; rebuild gr-uwb python bindings"
                         % bindir)
    so = matches[-1]
    spec = importlib.util.spec_from_file_location("uwb_python", so)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


bootstrap_uhd_env()
uwb = load_uwb()

C_LIGHT = 299792458.0
CG400_HZ = 491520000.0
WORK_HZ = 998400000.0
SPS = 1016
SFD_SYMS_4Z2 = 8
SFD_MODE = "4z2"
# HRP BPRF preamble profiles.  The C++ blocks accept sync_repetitions in
# 32/64/128/256/512/1024/2048 and preamble code indices 9..12.  The app
# exposes the standard selectable preamble lengths and all four codes; TX
# waveform, CIR estimator and metadata must agree on both.
DEFAULT_CODE_INDEX = 9
CODE_INDEX_CHOICES = (9, 10, 11, 12)
DEFAULT_PREAMBLE_LENGTH = 64
PREAMBLE_LENGTH_CHOICES = (32, 64, 128, 256, 512, 1024)
# Legacy alias also accepts 2048 (supported by the C++ blocks but not part
# of the standard selectable preamble set exposed by --preamble-length).
SYNC_REPS_ALIAS_CHOICES = PREAMBLE_LENGTH_CHOICES + (2048,)


def llround(x):
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def ceildiv(a, b):
    return -(-a // b)


def hex_to_bytes(s):
    h = "".join(c for c in s if c in "0123456789abcdefABCDEF")
    if len(h) % 2:
        raise ValueError("psdu hex must have even length")
    return list(bytes.fromhex(h))


IQ_SCALE = 32768.0

# UDP CIR datagram: 28-byte header + 116 complex64 taps (always, zeros if fail).
# socket_pdu only forwarded the c32vector, so sfd_failed became a 0-byte
# datagram and the far end (expect 928) counted ~5 pkt/s while radio was 100 Hz.
CIR_UDP_MAGIC = b"UCR1"
CIR_UDP_TAPS = 116
CIR_UDP_HDR = struct.Struct("<4sIHHffiI")
# UCR2 appends the per-pulse centre frequency (f64 so kHz-level CFO is not
# lost to f32 rounding at 6.5 GHz).  Sent only when the sink is given a
# freq_lookup; otherwise the legacy UCR1 frame is unchanged.
CIR_UDP_MAGIC_V2 = b"UCR2"
CIR_UDP_HDR_V2 = struct.Struct("<4sIHHffiIdd")
CIR_UDP_STATUS = {
    "ok": 0,
    "sfd_failed": 1,
    "timing_failed": 2,
    "cir_failed": 3,
}


def fc32_to_sc16(iq):
    """UHD-style host float → interleaved little-endian int16 I/Q."""
    x = np.asarray(iq, dtype=np.complex64)
    interleaved = np.empty(x.size * 2, dtype=np.float64)
    interleaved[0::2] = np.real(x)
    interleaved[1::2] = np.imag(x)
    scaled = np.rint(interleaved * IQ_SCALE)
    return np.clip(scaled, -32768, 32767).astype(np.int16)


def _pmt_str(meta, key, default=""):
    v = pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)
    if pmt.is_symbol(v):
        return pmt.symbol_to_string(v)
    return default


def _pmt_int(meta, key, default=0):
    v = pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)
    if pmt.is_uint64(v):
        return int(pmt.to_uint64(v))
    if pmt.is_integer(v):
        return int(pmt.to_long(v))
    return default


def _pmt_float(meta, key, default=0.0):
    v = pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)
    if pmt.is_real(v):
        return float(pmt.to_double(v))
    if pmt.is_uint64(v):
        return float(pmt.to_uint64(v))
    if pmt.is_integer(v):
        return float(pmt.to_long(v))
    return default


class CirUdpSink(gr.basic_block):
    """Non-blocking UDP sink for CIR PDUs. Always sends header+116 taps.

    ``freq_lookup(pulse_id) -> (freq_hz, freq_offset_hz) | None`` makes the
    sink emit a UCR2 frame carrying the per-pulse centre frequency (used by
    the frequency-sweep app).  Without it the legacy UCR1 frame is sent.
    """

    def __init__(self, host, port, tap_count=CIR_UDP_TAPS, freq_lookup=None):
        gr.basic_block.__init__(self, name="cir_udp_sink",
                                in_sig=None, out_sig=None)
        self.tap_count = int(tap_count)
        self.freq_lookup = freq_lookup
        self._dst = (host, int(port))
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self.sent = 0
        self.sent_ok = 0
        self.sent_fail = 0
        self.sent_freq = 0
        self.dropped = 0
        self.message_port_register_in(pmt.intern("cir"))
        self.set_msg_handler(pmt.intern("cir"), self._on_cir)

    def _on_cir(self, msg):
        if not pmt.is_pair(msg):
            self.dropped += 1
            return
        meta = pmt.car(msg)
        vec = pmt.cdr(msg)
        taps = np.zeros(self.tap_count, dtype=np.complex64)
        if pmt.is_c32vector(vec):
            raw = np.asarray(pmt.c32vector_elements(vec), dtype=np.complex64)
            n = min(int(raw.size), self.tap_count)
            if n:
                taps[:n] = raw[:n]
        status_s = _pmt_str(meta, "status", "other")
        status = CIR_UDP_STATUS.get(status_s, 4)
        pulse_id = _pmt_int(meta, "pulse_id", 0) & 0xFFFFFFFF
        sfd_m = _pmt_float(meta, "sfd_metric", 0.0)
        peak_m = _pmt_float(meta, "cir_peak_metric", 0.0)
        peak_tap = _pmt_int(meta, "peak_tap", 0)
        est_us = _pmt_int(meta, "estimator_us", 0) & 0xFFFFFFFF
        hdr = None
        if self.freq_lookup is not None:
            fr = self.freq_lookup(pulse_id)
            if fr is not None:
                freq_hz, freq_off = fr
                hdr = CIR_UDP_HDR_V2.pack(
                    CIR_UDP_MAGIC_V2, pulse_id, status, self.tap_count,
                    sfd_m, peak_m, peak_tap, est_us, float(freq_hz),
                    float(freq_off))
                self.sent_freq += 1
        if hdr is None:
            hdr = CIR_UDP_HDR.pack(
                CIR_UDP_MAGIC, pulse_id, status, self.tap_count, sfd_m,
                peak_m, peak_tap, est_us)
        try:
            self._sock.sendto(hdr + taps.tobytes(), self._dst)
        except (BlockingIOError, InterruptedError, OSError):
            self.dropped += 1
            return
        self.sent += 1
        if status == 0:
            self.sent_ok += 1
        else:
            self.sent_fail += 1


def start_live_stats(echo, est, wr, udp, stop_evt, pri_s):
    def loop():
        t0 = time.monotonic()
        prev = (t0, echo._ok, est.pdus_completed(), est.pdus_failed(),
                est.pdus_dropped(), wr.frames_written(),
                0 if udp is None else udp.sent,
                0 if udp is None else udp.sent_ok)
        while not stop_evt.wait(1.0):
            t1 = time.monotonic()
            dt = t1 - prev[0]
            if dt <= 0:
                continue
            echo_ok = echo._ok
            cir_ok = est.pdus_completed()
            cir_fail = est.pdus_failed()
            est_drop = est.pdus_dropped()
            wr_ok = wr.frames_written()
            udp_n = 0 if udp is None else udp.sent
            udp_ok = 0 if udp is None else udp.sent_ok
            udp_eagain = 0 if udp is None else udp.dropped
            print(
                "live dt=%.3f echo_ok_hz=%.1f cir_ok_hz=%.1f cir_fail_hz=%.1f "
                "est_q=%d est_drop=%d wr_hz=%.1f udp_hz=%.1f udp_ok_hz=%.1f "
                "udp_eagain=%d service_us_mean=%d max=%d pri_hz=%.1f" % (
                    dt,
                    (echo_ok - prev[1]) / dt,
                    (cir_ok - prev[2]) / dt,
                    (cir_fail - prev[3]) / dt,
                    int(est.queue_depth()),
                    est_drop,
                    (wr_ok - prev[5]) / dt,
                    (udp_n - prev[6]) / dt,
                    (udp_ok - prev[7]) / dt,
                    udp_eagain,
                    int(est.service_mean_us()),
                    int(est.service_max_us()),
                    (1.0 / pri_s) if pri_s > 0 else 0.0),
                flush=True)
            if est_drop > prev[4]:
                print("NOTE estimator dropped %d frames (queue_full); "
                      "radio ok is not CIR/UDP ok" % (est_drop - prev[4]),
                      flush=True)
            prev = (t1, echo_ok, cir_ok, cir_fail, est_drop, wr_ok,
                    udp_n, udp_ok)
    th = threading.Thread(target=loop, name="cir_live", daemon=True)
    th.start()
    return th


def rx_geometry(rate, pre_us, sync_reps, range_m, tail_us,
                tx_native_samples=0, pad_us=8.0):
    """RX window must cover the full native TX burst, not just SYNC+SFD.

    With STS+PHR/PSDU the HRP packet is ~191 us; SYNC+SFD is only ~73 us.
    FPGA RX length is rounded up to a multiple of 4.
    """
    pre = llround(pre_us * 1e-6 * rate)
    sync = ceildiv(sync_reps * SPS * 32, 65)
    sfd = ceildiv(SFD_SYMS_4Z2 * SPS * 32, 65)
    rng = int(math.ceil(2.0 * range_m / C_LIGHT * rate))
    tail = llround(tail_us * 1e-6 * rate)
    pad = llround(float(pad_us) * 1e-6 * rate)
    body = int(tx_native_samples) if tx_native_samples else (sync + sfd)
    rx = pre + body + rng + pad + tail
    rx = (rx + 3) // 4 * 4
    return pre, sync, sfd, rng, tail, pad, rx


class TimedUhdEcho(gr.basic_block):
    """Downsample 998.4 TX PDU to 491.52, timed USRP burst, emit native RX PDU."""

    def __init__(self, args, rate, freq, tx_ch, rx_ch, tx_ant, rx_ant,
                 gain_tx, gain_rx, pre_us, range_m, tail_us, sync_reps,
                 cal_delay_native, arm_delay_s, pri_s, max_pulses,
                 rx_dump_dir="", min_lead_s=0.002, timing_path="",
                 sc16_dump_dir="", rx_pad_us=8.0,
                 code_index=DEFAULT_CODE_INDEX, sfd_mode=SFD_MODE):
        gr.basic_block.__init__(self, name="timed_uhd_echo",
                                in_sig=None, out_sig=None)
        self.rate = float(rate)
        self.freq = float(freq)
        self.code_index = int(code_index)
        self.sfd_mode = sfd_mode
        self.tx_ch = int(tx_ch)
        self.rx_ch = int(rx_ch)
        self.tx_ant = tx_ant
        self.rx_ant = rx_ant
        self.gain_tx = float(gain_tx)
        self.gain_rx = float(gain_rx)
        self.pre_us = float(pre_us)
        self.range_m = float(range_m)
        self.tail_us = float(tail_us)
        self.sync_reps = int(sync_reps)
        self.cal_delay_native = float(cal_delay_native)
        self.arm_delay_s = float(arm_delay_s)
        self.pri_s = float(pri_s)
        self.max_pulses = int(max_pulses)
        self.rx_dump_dir = rx_dump_dir
        self.sc16_dump_dir = sc16_dump_dir
        self.min_lead_s = float(min_lead_s)
        self.timing_path = timing_path
        self.rx_pad_us = float(rx_pad_us)
        self.pre, self.sync_n, self.sfd_n, self.rng_n, self.tail, self.pad_n, self.rx_len = \
            rx_geometry(rate, pre_us, sync_reps, range_m, tail_us, 0, self.rx_pad_us)

        self.message_port_register_in(pmt.intern("tx"))
        self.message_port_register_out(pmt.intern("rx"))
        self.message_port_register_out(pmt.intern("status"))
        self.set_msg_handler(pmt.intern("tx"), self._on_tx)

        import uhd
        self._uhd = uhd
        self._usrp = uhd.usrp.MultiUSRP(args)
        self._usrp.set_clock_source("internal")
        self._usrp.set_time_source("internal")
        self._usrp.set_time_now(uhd.types.TimeSpec(0.0))
        self._usrp.set_tx_rate(self.rate, self.tx_ch)
        self._usrp.set_rx_rate(self.rate, self.rx_ch)
        tx_rate = float(self._usrp.get_tx_rate(self.tx_ch))
        rx_rate = float(self._usrp.get_rx_rate(self.rx_ch))
        if abs(tx_rate - self.rate) > 1.0 or abs(rx_rate - self.rate) > 1.0:
            raise RuntimeError("rate coerced tx=%r rx=%r" % (tx_rate, rx_rate))
        self._usrp.set_tx_freq(uhd.types.TuneRequest(self.freq), self.tx_ch)
        self._usrp.set_rx_freq(uhd.types.TuneRequest(self.freq), self.rx_ch)
        self._usrp.set_tx_gain(self.gain_tx, self.tx_ch)
        self._usrp.set_rx_gain(self.gain_rx, self.rx_ch)
        self._usrp.set_tx_antenna(self.tx_ant, self.tx_ch)
        self._usrp.set_rx_antenna(self.rx_ant, self.rx_ch)
        sa_rx = uhd.usrp.StreamArgs("fc32", "sc16")
        sa_rx.channels = [self.rx_ch]
        sa_tx = uhd.usrp.StreamArgs("fc32", "sc16")
        sa_tx.channels = [self.tx_ch]
        self._rx_stream = self._usrp.get_rx_stream(sa_rx)
        self._tx_stream = self._usrp.get_tx_stream(sa_tx)
        time.sleep(0.15)
        self._done = 0
        self._ok = 0
        self._fail = 0
        self._late = 0
        self._pub_ok = 0
        self._t0 = None
        self._native = None
        self._stop_pub = threading.Event()
        self._pub_q = queue.Queue(maxsize=64)
        self._pub_thread = None
        self._timing = []
        self._sc16_iq = None
        self._sc16_jsonl = None
        self._sc16_offset = 0
        self._sc16_written = 0
        self.status = {
            "tx_rate": tx_rate, "rx_rate": rx_rate,
            "tx_ant": self._usrp.get_tx_antenna(self.tx_ch),
            "rx_ant": self._usrp.get_rx_antenna(self.rx_ch),
            "rx_window": self.rx_len, "pre": self.pre,
            "pri_s": self.pri_s, "max_pulses": self.max_pulses,
            "rx_pad_us": self.rx_pad_us,
            "code_index": self.code_index, "sfd_mode": self.sfd_mode,
            "sync_reps": self.sync_reps,
        }

    def set_tx_native(self, wave):
        peak = float(np.max(np.abs(wave))) or 1.0
        self._native = (np.asarray(wave, dtype=np.complex64) / peak * 0.8).astype(np.complex64)
        self.pre, self.sync_n, self.sfd_n, self.rng_n, self.tail, self.pad_n, self.rx_len = \
            rx_geometry(self.rate, self.pre_us, self.sync_reps, self.range_m,
                        self.tail_us, int(self._native.size), self.rx_pad_us)
        self.status["rx_window"] = self.rx_len
        self.status["tx_native"] = int(self._native.size)
        self.status["rx_window_us"] = self.rx_len / self.rate * 1e6
        self.status["tx_us"] = self._native.size / self.rate * 1e6
        self.status["pad"] = self.pad_n

    def open_sc16_dump(self):
        if not self.sc16_dump_dir:
            return
        os.makedirs(self.sc16_dump_dir, exist_ok=True)
        self._sc16_iq = open(os.path.join(self.sc16_dump_dir, "capture.iq"), "wb")
        self._sc16_jsonl = open(os.path.join(self.sc16_dump_dir, "capture.jsonl"),
                                "w", encoding="utf-8")
        self._sc16_offset = 0
        self._sc16_written = 0

    def close_sc16_dump(self):
        for fh in (self._sc16_iq, self._sc16_jsonl):
            if fh is not None:
                fh.flush()
                fh.close()
        self._sc16_iq = None
        self._sc16_jsonl = None

    def _write_sc16_packet(self, pulse_id, rx):
        if self._sc16_iq is None or self._sc16_jsonl is None:
            return
        sc16 = fc32_to_sc16(rx)
        n = int(rx.size)
        self._sc16_iq.write(sc16.tobytes())
        predicted = self._sc16_offset + self.pre
        rec = {
            "packet_id": int(pulse_id),
            "start_sample": int(predicted),
            "trigger_sample": int(predicted),
            "sample_rate": int(round(self.rate)),
            "sample_count": n,
            "file_offset_samples": int(self._sc16_offset),
            "detection_metric": 0.0,
            "pre_trigger_samples": int(self.pre),
            "sample_format": "sc16",
            "iq_scale": IQ_SCALE,
            "window_start_sample": int(self._sc16_offset),
            "predicted_start_sample": int(predicted),
            "pre_guard_samples": int(self.pre),
            "capture_samples": int(self.rx_len - self.pre - self.tail),
            "post_guard_samples": int(self.tail),
            "schedule_index": int(pulse_id),
            "capture_mode": "x410_echo",
            "lock_state": "timed",
        }
        self._sc16_jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._sc16_jsonl.flush()
        self._sc16_offset += n
        self._sc16_written += 1

    def start_publisher(self):
        if self._pub_thread is not None:
            return
        self._stop_pub.clear()
        self._pub_thread = threading.Thread(target=self._publisher, name="cir_pub",
                                            daemon=True)
        self._pub_thread.start()

    def stop_publisher(self, timeout=5.0):
        if self._pub_thread is None:
            return
        self._pub_q.put(None)
        self._stop_pub.set()
        self._pub_thread.join(timeout=timeout)
        self._pub_thread = None
        if self.timing_path:
            with open(self.timing_path, "w", encoding="utf-8") as f:
                for rec in self._timing:
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    def _tspec(self, seconds):
        full = int(math.floor(seconds))
        return self._uhd.types.TimeSpec(full, seconds - full)

    def _send(self, wave, t_tx, timeout):
        md = self._uhd.types.TXMetadata()
        md.has_time_spec = True
        md.start_of_burst = True
        md.end_of_burst = False
        md.time_spec = self._tspec(t_tx)
        sent = 0
        n = wave.size
        maxp = int(self._tx_stream.get_max_num_samps())
        buf = wave.reshape(1, -1)
        while sent < n:
            chunk = min(maxp, n - sent)
            if sent + chunk >= n:
                md.end_of_burst = True
            nsent = self._tx_stream.send(buf[:, sent:sent + chunk], md, timeout)
            if nsent <= 0:
                return sent, "send_stalled"
            sent += nsent
            md.has_time_spec = False
            md.start_of_burst = False
        return sent, ""

    def _recv(self, n, timeout):
        maxp = int(self._rx_stream.get_max_num_samps())
        out = np.zeros(n, dtype=np.complex64)
        md = self._uhd.types.RXMetadata()
        got = 0
        err = ""
        first = None
        deadline = time.monotonic() + timeout
        while got < n:
            left = deadline - time.monotonic()
            if left <= 0:
                err = "timeout after %d/%d" % (got, n)
                break
            want = min(maxp, n - got)
            buf = np.zeros((1, want), dtype=np.complex64)
            ngot = self._rx_stream.recv(buf, md, left)
            if md.error_code != self._uhd.types.RXMetadataErrorCode.none:
                err = str(md.error_code)
                if ngot <= 0:
                    break
            if ngot > 0:
                if first is None and md.has_time_spec:
                    first = md.time_spec.get_real_secs()
                out[got:got + ngot] = buf[0, :ngot]
                got += ngot
        status = "ok" if got == n and not err else "fail"
        return out, got, first, status, err

    def _cache_native_from_msg(self, msg):
        if self._native is not None:
            return True
        if not pmt.is_pair(msg):
            return False
        vec = pmt.cdr(msg)
        raw = pmt.c32vector_elements(vec)
        tx_work = np.asarray(raw, dtype=np.complex64)
        if tx_work.size == 0:
            return False
        native = resample_poly(tx_work.astype(np.complex128), 32, 65)
        self.set_tx_native(native.astype(np.complex64))
        return True

    def _make_rx_meta(self, pulse_id):
        cap = self.rx_len - self.pre - self.tail
        if cap < 0:
            cap = self.rx_len
        meta = pmt.make_dict()
        meta = pmt.dict_add(meta, pmt.intern("pulse_id"), pmt.from_uint64(pulse_id))
        meta = pmt.dict_add(meta, pmt.intern("packet_id"), pmt.from_uint64(pulse_id))
        meta = pmt.dict_add(meta, pmt.intern("schedule_index"),
                            pmt.from_uint64(pulse_id))
        meta = pmt.dict_add(meta, pmt.intern("sample_rate"),
                            pmt.from_double(self.rate))
        meta = pmt.dict_add(meta, pmt.intern("sample_format"), pmt.intern("fc32"))
        meta = pmt.dict_add(meta, pmt.intern("window_start_sample"), pmt.from_long(0))
        meta = pmt.dict_add(meta, pmt.intern("pre_guard_samples"),
                            pmt.from_long(self.pre))
        meta = pmt.dict_add(meta, pmt.intern("capture_samples"), pmt.from_long(cap))
        meta = pmt.dict_add(meta, pmt.intern("post_guard_samples"),
                            pmt.from_long(self.tail))
        meta = pmt.dict_add(meta, pmt.intern("sample_count"),
                            pmt.from_long(self.rx_len))
        meta = pmt.dict_add(meta, pmt.intern("sync_repetitions"),
                            pmt.from_long(self.sync_reps))
        meta = pmt.dict_add(meta, pmt.intern("sfd_mode"), pmt.intern(self.sfd_mode))
        meta = pmt.dict_add(meta, pmt.intern("code_index"),
                            pmt.from_long(self.code_index))
        meta = pmt.dict_add(meta, pmt.intern("sync_samples"),
                            pmt.from_long(self.sync_n))
        meta = pmt.dict_add(meta, pmt.intern("sfd_samples"),
                            pmt.from_long(self.sfd_n))
        meta = pmt.dict_add(meta, pmt.intern("tx_packet_samples"),
                            pmt.from_long(int(self._native.size)))
        meta = pmt.dict_add(meta, pmt.intern("rx_capture_samples"),
                            pmt.from_long(self.rx_len))
        meta = pmt.dict_add(meta, pmt.intern("calibration_delay_native_samples"),
                            pmt.from_double(self.cal_delay_native))
        meta = pmt.dict_add(meta, pmt.intern("num_delay_samps"),
                            pmt.from_long(int(round(self.cal_delay_native))))
        meta = pmt.dict_add(meta, pmt.intern("source"), pmt.intern("x410_echo"))
        return meta

    def _enqueue_rx(self, pulse_id, rx):
        try:
            self._pub_q.put_nowait((pulse_id, rx))
            return True
        except queue.Full:
            self._fail += 1
            extra = pmt.make_dict()
            extra = pmt.dict_add(extra, pmt.intern("event"), pmt.intern("burst_fail"))
            extra = pmt.dict_add(extra, pmt.intern("error"), pmt.intern("pub_queue_full"))
            extra = pmt.dict_add(extra, pmt.intern("pulse_id"),
                                 pmt.from_uint64(pulse_id))
            self.message_port_pub(pmt.intern("status"), extra)
            return False

    def _publisher(self):
        while True:
            item = self._pub_q.get()
            if item is None:
                break
            pulse_id, rx = item
            t0 = time.perf_counter()
            rx = np.ascontiguousarray(rx, dtype=np.complex64)
            rx_pmt = pmt.init_c32vector(int(rx.size), rx.tolist())
            meta = self._make_rx_meta(pulse_id)
            self.message_port_pub(pmt.intern("rx"), pmt.cons(meta, rx_pmt))
            self._pub_ok += 1
            dt = (time.perf_counter() - t0) * 1e3
            if pulse_id < 3 or pulse_id % 100 == 99:
                print("pub pulse=%d pmt_ms=%.2f q=%d" % (
                    pulse_id, dt, self._pub_q.qsize()), flush=True)

    def _one_burst(self, pulse_id):
        now = self._usrp.get_time_now().get_real_secs()
        if self._t0 is None:
            self._t0 = now + self.arm_delay_s
        t_tx = self._t0 + pulse_id * self.pri_s
        t_rx = t_tx - self.pre_us * 1e-6
        lead = t_rx - now
        rec = {
            "pulse_id": pulse_id,
            "now": now,
            "t_tx": t_tx,
            "t_rx": t_rx,
            "lead_s": lead,
        }
        if lead < self.min_lead_s:
            self._late += 1
            self._fail += 1
            rec["status"] = "late"
            rec["error"] = "lead_s=%.6f < min_lead_s=%.6f" % (lead, self.min_lead_s)
            self._timing.append(rec)
            extra = pmt.make_dict()
            extra = pmt.dict_add(extra, pmt.intern("event"), pmt.intern("burst_fail"))
            extra = pmt.dict_add(extra, pmt.intern("error"), pmt.intern("late"))
            extra = pmt.dict_add(extra, pmt.intern("pulse_id"),
                                 pmt.from_uint64(pulse_id))
            self.message_port_pub(pmt.intern("status"), extra)
            return False

        cmd = self._uhd.types.StreamCMD(self._uhd.types.StreamMode.num_done)
        cmd.num_samps = int(self.rx_len)
        cmd.stream_now = False
        cmd.time_spec = self._tspec(t_rx)
        t_issue = time.perf_counter()
        self._rx_stream.issue_stream_cmd(cmd)
        sent, send_err = self._send(self._native, t_tx, timeout=2.0)
        timeout = max(0.5, (t_tx - now) + self.rx_len / self.rate + 0.25)
        rx, got, first, status, err = self._recv(self.rx_len, timeout)
        rec["uhd_ms"] = (time.perf_counter() - t_issue) * 1e3
        rec["tx_sent"] = sent
        rec["rx_got"] = got
        rec["rx_first"] = first
        rec["status"] = status
        rec["error"] = err or send_err
        self._timing.append(rec)

        if self.rx_dump_dir and got > 0:
            os.makedirs(self.rx_dump_dir, exist_ok=True)
            rx[:got].tofile(os.path.join(self.rx_dump_dir,
                                         "pulse_%04d.cf32" % pulse_id))
        if status == "ok" and got == self.rx_len:
            self._write_sc16_packet(pulse_id, rx)
        if status != "ok":
            self._fail += 1
            extra = pmt.make_dict()
            extra = pmt.dict_add(extra, pmt.intern("event"), pmt.intern("burst_fail"))
            extra = pmt.dict_add(extra, pmt.intern("error"),
                                 pmt.intern(err or send_err or status))
            extra = pmt.dict_add(extra, pmt.intern("pulse_id"),
                                 pmt.from_uint64(pulse_id))
            extra = pmt.dict_add(extra, pmt.intern("rx_got"), pmt.from_long(got))
            extra = pmt.dict_add(extra, pmt.intern("tx_sent"), pmt.from_long(sent))
            self.message_port_pub(pmt.intern("status"), extra)
            return False
        self._ok += 1
        return self._enqueue_rx(pulse_id, rx)

    def run_schedule(self):
        if self._native is None:
            raise RuntimeError("TX waveform not set")
        self.open_sc16_dump()
        self.start_publisher()
        t_host0 = time.perf_counter()
        try:
            for pulse_id in range(self.max_pulses):
                self._one_burst(pulse_id)
                self._done = pulse_id + 1
                if pulse_id == 0 or (pulse_id + 1) % 100 == 0 or pulse_id + 1 == self.max_pulses:
                    dt = time.perf_counter() - t_host0
                    print("sched %d/%d ok=%d fail=%d late=%d sc16=%d host_s=%.3f" % (
                        pulse_id + 1, self.max_pulses, self._ok, self._fail,
                        self._late, self._sc16_written, dt), flush=True)
        finally:
            self.close_sc16_dump()
        return self._ok, self._fail, self._late

    def _on_tx(self, msg):
        if self._done >= self.max_pulses:
            return
        if not self._cache_native_from_msg(msg):
            return
        pulse_id = self._done
        self._one_burst(pulse_id)
        self._done += 1


def build_parser(add_help=True):
    p = argparse.ArgumentParser(add_help=add_help)
    p.add_argument("--args", default="addr=192.168.10.2")
    p.add_argument("--pulses", type=int, default=8)
    p.add_argument("--pri-s", type=float, default=0.05)
    p.add_argument("--rate-hz", type=float, default=0.0,
                   help="If set, PRI=1/rate and pulses=round(rate*duration)")
    p.add_argument("--duration-s", type=float, default=10.0)
    p.add_argument("--gain-tx", type=float, default=40.0)
    p.add_argument("--gain-rx", type=float, default=50.0)
    p.add_argument("--tx-channel", type=int, default=0)
    p.add_argument("--rx-channel", type=int, default=3)
    p.add_argument("--tx-antenna", default="TX/RX0")
    p.add_argument("--rx-antenna", default="RX1")
    p.add_argument("--freq", type=float, default=6489.6e6)
    p.add_argument("--cal-delay-native", type=float, default=334.0)
    p.add_argument("--sfd-search-margin", type=int, default=128,
                   help="SFD search half-window at 998.4 MS/s; 8192 is ~180 ms/frame")
    p.add_argument("--sfd-threshold", type=float, default=0.12)
    p.add_argument("--sync-refine-margin", type=int, default=32)
    p.add_argument("--sync-refine-threshold", type=float, default=0.02)
    p.add_argument("--tail-guard-us", type=float, default=20.0)
    p.add_argument("--pre-guard-us", type=float, default=2.0)
    p.add_argument("--rx-pad-us", type=float, default=8.0,
                   help="Extra native samples after TX burst before tail guard")
    p.add_argument("--psdu-hex",
                   default="47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221")
    p.add_argument("--code-index", type=int, default=DEFAULT_CODE_INDEX,
                   choices=list(CODE_INDEX_CHOICES),
                   help="HRP BPRF preamble code index (9/10/11/12); must "
                        "match TX waveform and CIR template")
    p.add_argument("--preamble-length", type=int, default=None,
                   choices=list(PREAMBLE_LENGTH_CHOICES),
                   help="HRP SYNC preamble length in repetitions "
                        "(32/64/128/256/512/1024, default %d)"
                        % DEFAULT_PREAMBLE_LENGTH)
    p.add_argument("--sync-reps", type=int, default=None,
                   choices=list(SYNC_REPS_ALIAS_CHOICES),
                   help="Deprecated alias for --preamble-length "
                        "(also accepts 2048)")
    p.add_argument("--insert-sts", action="store_true", default=True)
    p.add_argument("--no-sts", action="store_true")
    p.add_argument("--pulse-shape", default="linear",
                   choices=["linear", "minphase", "legacy", "gaussian",
                            "blackman", "external"],
                   help="TX pulse shaping on the 998.4 work grid; gaussian "
                        "(sigma=2.5 ns) removes the 491.52 MS/s band-edge "
                        "ringing of the legacy Butterworth-fit pulse")
    p.add_argument("--pulse-sigma-ns", type=float, default=2.5)
    p.add_argument("--pulse-bw-mhz", type=float, default=200.0)
    p.add_argument("--pulse-taps", default="",
                   help="Raw float32 causal FIR taps at 998.4 MS/s; overrides "
                        "--pulse-shape (external pulse), e.g. "
                        "testdata/uwb_hrp_tx/pulse_minphase_rc160_240.f32")
    p.add_argument("--output", required=True)
    p.add_argument("--taps", default="")
    p.add_argument("--template", default="")
    p.add_argument("--dump-rx", action="store_true")
    p.add_argument("--dump-sc16", action="store_true",
                   help="Write native RX windows as capture.iq + capture.jsonl (SC16)")
    p.add_argument("--min-lead-s", type=float, default=0.002)
    p.add_argument("--arm-delay-s", type=float, default=0.25)
    p.add_argument("--udp-host", default="133.133.133.132",
                   help="CIR taps UDP destination (empty disables)")
    p.add_argument("--udp-port", default="12345")
    p.add_argument("--no-udp", action="store_true",
                   help="Do not send CIR taps over UDP")
    p.add_argument("--est-queue", type=int, default=64,
                   help="CIR estimator job queue; overflow is a real drop. "
                        "Do not set this to pulses — that hides lag as 'no loss'")
    p.add_argument("--require-sfd", action="store_true",
                   help="Gate CIR on SFD search (default: use scheduled echo time)")
    return p


def parse_args():
    return build_parser().parse_args()


def analyze_cir(jsonl_path, pulses):
    if not os.path.isfile(jsonl_path):
        return {"exists": False, "lines": 0}
    ok = 0
    fail = 0
    ids = []
    peak_taps = []
    metrics = []
    est_us = []
    statuses = {}
    with open(jsonl_path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            rec = json.loads(ln)
            st = rec.get("status", "")
            statuses[st] = statuses.get(st, 0) + 1
            pid = rec.get("pulse_id")
            if pid is not None:
                ids.append(int(pid))
            if st == "ok":
                ok += 1
                if "peak_tap" in rec:
                    peak_taps.append(int(rec["peak_tap"]))
                if "cir_peak_metric" in rec:
                    metrics.append(float(rec["cir_peak_metric"]))
                if "estimator_us" in rec:
                    est_us.append(int(rec["estimator_us"]))
            else:
                fail += 1
    missing = []
    if ids:
        seen = set(ids)
        missing = [i for i in range(pulses) if i not in seen]
        dups = len(ids) - len(seen)
    else:
        dups = 0
    gaps = []
    if ids:
        s = sorted(ids)
        for a, b in zip(s, s[1:]):
            if b != a + 1:
                gaps.append((a, b))
    return {
        "exists": True,
        "lines": ok + fail,
        "ok": ok,
        "fail": fail,
        "status_hist": statuses,
        "unique_ids": len(set(ids)),
        "missing_count": len(missing),
        "missing_head": missing[:16],
        "dup_count": dups,
        "id_gaps": gaps[:16],
        "id_min": min(ids) if ids else None,
        "id_max": max(ids) if ids else None,
        "peak_tap_min": min(peak_taps) if peak_taps else None,
        "peak_tap_max": max(peak_taps) if peak_taps else None,
        "metric_min": min(metrics) if metrics else None,
        "metric_max": max(metrics) if metrics else None,
        "metric_mean": (sum(metrics) / len(metrics)) if metrics else None,
        "estimator_us_min": min(est_us) if est_us else None,
        "estimator_us_max": max(est_us) if est_us else None,
        "estimator_us_mean": (sum(est_us) / len(est_us)) if est_us else None,
    }


def analyze_timing(path):
    if not os.path.isfile(path):
        return {}
    recs = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                recs.append(json.loads(ln))
    if not recs:
        return {"n": 0}
    ok = [r for r in recs if r.get("status") == "ok"]
    dt = []
    for a, b in zip(ok, ok[1:]):
        dt.append(b["t_tx"] - a["t_tx"])
    leads = [r.get("lead_s") for r in recs if r.get("lead_s") is not None]
    uhd = [r.get("uhd_ms") for r in recs if r.get("uhd_ms") is not None]
    return {
        "n": len(recs),
        "ok": len(ok),
        "late": sum(1 for r in recs if r.get("status") == "late"),
        "dt_tx_min": min(dt) if dt else None,
        "dt_tx_max": max(dt) if dt else None,
        "dt_tx_mean": (sum(dt) / len(dt)) if dt else None,
        "lead_min": min(leads) if leads else None,
        "lead_max": max(leads) if leads else None,
        "uhd_ms_min": min(uhd) if uhd else None,
        "uhd_ms_max": max(uhd) if uhd else None,
        "uhd_ms_mean": (sum(uhd) / len(uhd)) if uhd else None,
        "span_s": (ok[-1]["t_tx"] - ok[0]["t_tx"]) if len(ok) >= 2 else 0.0,
    }


def main():
    bootstrap_uhd_env()
    a = parse_args()
    # --preamble-length is the primary knob; --sync-reps stays as a
    # backward-compatible alias.  Both must agree when given together.
    if a.preamble_length is not None and a.sync_reps is not None \
            and a.preamble_length != a.sync_reps:
        raise SystemExit("--preamble-length=%d conflicts with --sync-reps=%d"
                         % (a.preamble_length, a.sync_reps))
    if a.preamble_length is not None:
        a.sync_reps = a.preamble_length
    elif a.sync_reps is None:
        a.sync_reps = DEFAULT_PREAMBLE_LENGTH
    insert_sts = False if a.no_sts else True
    if a.rate_hz and a.rate_hz > 0:
        a.pri_s = 1.0 / a.rate_hz
        a.pulses = int(round(a.rate_hz * a.duration_s))
    repo = find_repo_root()
    os.makedirs(a.output, exist_ok=True)
    taps = a.taps or os.path.join(
        repo, "testdata", "resampler_65_32", "taps_quality_minorder.txt")
    tmpl_path = a.template or os.path.join(a.output, "sync_template_live.cf32")
    timing_path = os.path.join(a.output, "echo_timing.jsonl")

    psdu = hex_to_bytes(a.psdu_hex)
    if a.pulse_taps:
        pulse_shape, pulse_taps = "external", a.pulse_taps
    elif a.pulse_shape == "linear":
        pulse_shape = "external"
        pulse_taps = os.path.join(repo, "testdata", "uwb_hrp_tx",
                                  "pulse_trunc_linear_rc183_240.f32")
    elif a.pulse_shape == "minphase":
        pulse_shape = "external"
        pulse_taps = os.path.join(repo, "testdata", "uwb_hrp_tx",
                                  "pulse_minphase_rc160_240.f32")
    else:
        pulse_shape, pulse_taps = a.pulse_shape, ""
    src = uwb.hrp_packet_source(psdu, a.sync_reps, SFD_MODE, a.code_index,
                                0.8, a.pri_s, False, insert_sts, False,
                                pulse_shape, a.pulse_sigma_ns,
                                a.pulse_bw_mhz, pulse_taps)
    samples = np.array(src.samples(), dtype=np.complex64)
    if samples.size < SPS:
        raise SystemExit("HRP source produced %d samples" % samples.size)
    _tmpl_off = max(0, int(src.pulse_center_taps()) - 4)
    samples[_tmpl_off:_tmpl_off + SPS].tofile(tmpl_path)
    t_rs = time.perf_counter()
    native = resample_poly(samples.astype(np.complex128), 32, 65).astype(np.complex64)
    print("hrp_tx_998p4_samples=%d native_491p52=%d resample_ms=%.2f "
          "insert_sts=%s pulse_shape=%s pulse_taps=%d pulse_center=%d "
          "code_index=%d preamble_length=%d sfd_mode=%s"
          % (samples.size, native.size, (time.perf_counter() - t_rs) * 1e3,
             insert_sts, src.pulse_shape(), src.pulse_taps(),
             src.pulse_center_taps(), a.code_index, a.sync_reps, SFD_MODE),
          flush=True)
    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f" % (
        a.pulses, a.pri_s, (1.0 / a.pri_s), a.pulses * a.pri_s), flush=True)

    dump_dir = os.path.join(a.output, "rx_iq") if a.dump_rx else ""
    sc16_dir = a.output if a.dump_sc16 else ""
    echo = TimedUhdEcho(
        a.args, CG400_HZ, a.freq, a.tx_channel, a.rx_channel,
        a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx,
        a.pre_guard_us, 15.0, a.tail_guard_us, a.sync_reps, a.cal_delay_native,
        a.arm_delay_s, a.pri_s, a.pulses, dump_dir, a.min_lead_s, timing_path,
        sc16_dir, a.rx_pad_us, a.code_index, SFD_MODE)
    echo.set_tx_native(native)
    print("uhd_probe", echo.status, flush=True)
    if a.dump_sc16:
        tx_sc16_path = os.path.join(a.output, "tx_491p52.sc16")
        fc32_to_sc16(echo._native).tofile(tx_sc16_path)
        print("wrote_tx_sc16", tx_sc16_path, "samples=%d" % echo._native.size,
              flush=True)

    res = uwb.pdu_rational_resampler_ccf_65_32(taps, 998.4e6, True, 2097152)
    est_q = max(8, int(a.est_queue))
    use_pred = not a.require_sfd
    est = uwb.radar_cir_estimator(
        tmpl_path, a.sync_reps, SFD_MODE, a.code_index, 16, 100, 10, 0,
        a.sfd_search_margin, a.sync_refine_margin, a.sfd_threshold,
        a.sync_refine_threshold, True, est_q, use_pred)
    print("estimator code_index=%d preamble_length=%d sfd_search_margin=%d "
          "queue=%d use_predicted_timing=%s (overflow=drop)" % (
              a.code_index, a.sync_reps, a.sfd_search_margin, est_q, use_pred),
          flush=True)
    wr = uwb.cir_writer(a.output, "cir", True, 64)
    udp = None
    udp_on = (not a.no_udp) and bool(a.udp_host)
    if udp_on:
        udp = CirUdpSink(a.udp_host, int(a.udp_port), CIR_UDP_TAPS)
        print("udp_cir %s:%s framed=UCR1 always_send_taps=%d nonblock" % (
            a.udp_host, a.udp_port, CIR_UDP_TAPS), flush=True)

    tb = gr.top_block("x410_cg400_hrp_echo_cir")
    tb.msg_connect((echo, "rx"), (res, "packet"))
    tb.msg_connect((res, "packet"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if udp is not None:
        tb.msg_connect((est, "cir"), (udp, "cir"))
    # Do not attach message_debug on a 100 Hz soak: queue_full status
    # PDUs would flood the print block and stall the message system.

    tb.start()
    echo.start_publisher()
    live_stop = threading.Event()
    live_th = start_live_stats(echo, est, wr, udp, live_stop, a.pri_s)
    t_run = time.perf_counter()
    echo.run_schedule()
    sched_s = time.perf_counter() - t_run
    print("schedule_wall_s=%.3f" % sched_s, flush=True)
    live_stop.set()
    live_th.join(timeout=1.5)

    deadline = time.time() + 8.0
    while time.time() < deadline:
        written = wr.frames_written() + wr.frames_failed()
        if written >= echo._ok and est.drained() and echo._pub_q.empty():
            break
        time.sleep(0.05)
    echo.stop_publisher()
    tb.stop()
    tb.wait()
    try:
        wr.stop()
    except Exception:
        pass

    jsonl = os.path.join(a.output, "cir.jsonl")
    cir_stats = analyze_cir(jsonl, a.pulses)
    timing_stats = analyze_timing(timing_path)
    summary = {
        "hrp_samples": int(samples.size),
        "native_samples": int(native.size),
        "pulses": a.pulses,
        "pri_s": a.pri_s,
        "rate_hz": 1.0 / a.pri_s,
        "schedule_wall_s": sched_s,
        "echo_ok": echo._ok,
        "echo_fail": echo._fail,
        "echo_late": echo._late,
        "echo_pub": echo._pub_ok,
        "res_rx": res.pdus_received(),
        "res_tx": res.pdus_emitted(),
        "res_drop": res.pdus_dropped(),
        "est_rx": est.pdus_received(),
        "est_enq": est.pdus_enqueued(),
        "est_done": est.pdus_completed(),
        "est_fail": est.pdus_failed(),
        "est_drop": est.pdus_dropped(),
        "est_invalid": est.invalid_inputs(),
        "wr_ok": wr.frames_written(),
        "wr_fail": wr.frames_failed(),
        "wr_invalid": wr.frames_invalid(),
        "use_predicted_timing": use_pred,
        "est_queue_capacity": est_q,
        "est_queue_hwm": int(est.queue_high_watermark()),
        "est_service_us_mean": int(est.service_mean_us()),
        "est_service_us_max": int(est.service_max_us()),
        "udp_sent": 0 if udp is None else udp.sent,
        "udp_sent_ok": 0 if udp is None else udp.sent_ok,
        "udp_sent_fail": 0 if udp is None else udp.sent_fail,
        "udp_eagain": 0 if udp is None else udp.dropped,
        "cir": cir_stats,
        "timing": timing_stats,
        "output": a.output,
        "sc16_packets": echo._sc16_written,
        "sc16_samples": echo._sc16_offset,
        "dump_sc16": bool(a.dump_sc16),
        "freq_hz": a.freq,
        "native_rate_hz": CG400_HZ,
        "code_index": a.code_index,
        "preamble_length": a.sync_reps,
        "sfd_mode": SFD_MODE,
        "gain_tx": a.gain_tx,
        "gain_rx": a.gain_rx,
        "tx_channel": a.tx_channel,
        "rx_channel": a.rx_channel,
        "tx_antenna": a.tx_antenna,
        "rx_antenna": a.rx_antenna,
        "iq_scale": IQ_SCALE,
        "sample_format": "sc16",
        "rx_window": echo.rx_len,
        "rx_window_us": echo.rx_len / CG400_HZ * 1e6,
        "rx_pad_us": a.rx_pad_us,
    }
    with open(os.path.join(a.output, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    if a.dump_sc16:
        meta_path = os.path.join(a.output, "metadata.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump({
                "description": (
                    "X410 CG400 monostatic HRP echo, native SC16 RX windows"
                ),
                "sample_format": "sc16",
                "dtype": "int16",
                "byte_order": "little-endian",
                "layout": "interleaved_iq",
                "bytes_per_complex": 4,
                "iq_scale": IQ_SCALE,
                "sample_index_base": 0,
                "freq_hz": a.freq,
                "rate_native_hz": CG400_HZ,
                "rate_work_hz": WORK_HZ,
                "code_index": a.code_index,
                "sync_repetitions": a.sync_reps,
                "sfd_mode": SFD_MODE,
                "insert_sts": insert_sts,
                "packets": a.pulses,
                "pri_s": a.pri_s,
                "gain_tx": a.gain_tx,
                "gain_rx": a.gain_rx,
                "tx_channel": a.tx_channel,
                "rx_channel": a.rx_channel,
                "tx_antenna": a.tx_antenna,
                "rx_antenna": a.rx_antenna,
                "cal_delay_native_samples": a.cal_delay_native,
                "pre_guard_us": a.pre_guard_us,
                "tail_guard_us": a.tail_guard_us,
                "rx_pad_us": a.rx_pad_us,
                "rx_window_samples": echo.rx_len,
                "rx_window_us": echo.rx_len / CG400_HZ * 1e6,
                "files": {
                    "capture.iq": "concatenated native SC16 packets",
                    "capture.jsonl": "one JSON object per packet",
                    "tx_491p52.sc16": "timed TX burst (native 491.52 MS/s)",
                },
                "matlab": "[x, meta] = read_uwb_packet('capture.iq','capture.jsonl',id)",
                "echo": echo.status,
                "written_packets": echo._sc16_written,
                "written_samples": echo._sc16_offset,
            }, f, indent=2)
            f.write("\n")
    print("SUMMARY", json.dumps(summary), flush=True)
    print("radio_ok=%d cir_ok=%d cir_fail=%d est_drop=%d udp_sent=%d "
          "udp_ok=%d udp_fail=%d (radio ok is not CIR/UDP ok)" % (
              summary["echo_ok"],
              cir_stats.get("ok", 0),
              cir_stats.get("fail", 0),
              summary["est_drop"],
              summary["udp_sent"],
              summary["udp_sent_ok"],
              summary["udp_sent_fail"]),
          flush=True)
    if os.path.isfile(jsonl):
        print("cir.jsonl_lines=%d" % cir_stats.get("lines", 0), flush=True)
        with open(jsonl, "r", encoding="utf-8") as f:
            lines = [ln.strip() for ln in f if ln.strip()]
        for ln in lines[:2] + lines[-2:]:
            print(ln, flush=True)

    if a.dump_sc16:
        ok = (summary["echo_ok"] == a.pulses and
              summary["echo_late"] == 0 and
              summary["sc16_packets"] == a.pulses)
    else:
        ok = (summary["wr_ok"] == a.pulses and
              summary["echo_ok"] == a.pulses and
              summary["echo_late"] == 0 and
              cir_stats.get("ok") == a.pulses and
              cir_stats.get("missing_count", 1) == 0)
    raise SystemExit(0 if ok else 3)


if __name__ == "__main__":
    main()
