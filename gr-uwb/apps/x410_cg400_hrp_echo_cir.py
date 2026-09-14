#!/usr/bin/env python3
"""Live HRP synth + timed X410 TX/RX + PDU resample + radar CIR.

Pipeline:
  UwbHrpPacketSource.samples()     # 998.4 CF32, C++ IEEE 802.15.4a BPRF
    -> TimedUhdEcho                # work->native downsample once, SC16 burst
    -> PDU resampler               # 491.52/65-32 or 737.28/65-48 -> 998.4
    -> UwbRadarCirEstimator        # default: predicted TX time, no SFD gate
    -> UwbCirWriter + framed UDP

TX: UHD ch0 / TX/RX0 (front-panel ch1)
RX: UHD ch3 / RX1     (front-panel ch4)
Rate: --native-rate, default CG600 737.28 MS/s (CG400 491.52 still selectable)
Pulse: --pulse-shape, default 'legacy' standard full-band BPRF (CG600)

Timed echo locks SFD to a constant offset from the RX window
(predicted_sfd is identical every burst).  CIR uses that schedule, not
a detected SFD, so DW3000 collisions do not drop frames.  Pass
--require-sfd to restore the old search gate.

UDP is a non-blocking UCR2 header (28-byte base fields + freq_hz and
freq_offset_hz f64) + 116 taps on every pulse.  Live lines report
echo_ok_hz vs cir_ok_hz vs udp_hz; radio ok is not CIR ok.
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
    """Debian python3 does not add /usr/local site-packages by default.

    Re-exec once with PYTHONPATH (and the gr-uwb build lib) so `import uhd`
    works from a bare `python3` invocation.  The old /tmp/uhd_eal_noret
    patched-rte workaround is gone: the host DPDK libs and libuhd were
    rebuilt for this AVX2-only CPU, so no LD_LIBRARY_PATH override (and no
    DPDK AVX-512 SIGILL) remains.
    """
    if os.environ.get("UWB_UHD_BOOTSTRAPPED") == "1":
        return
    repo = find_repo_root()
    site = "/usr/local/lib/python3.10/site-packages"
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
CG600_HZ = 737280000.0
DEFAULT_NATIVE_HZ = CG600_HZ
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


class NativeRateProfile:
    """Native radio-rate contract for the two shipped X410 FPGA images.

    TX: the 998.4 MS/s work-grid waveform is resampled to the native device
    rate with ``resample_poly(work, tx_interp, tx_decim)``.
    RX: the native SC16 window is resampled back to 998.4 MS/s by the matching
    PDU block (65/32 for CG400, 65/48 for CG600).  Both ratios are exact:

        491.52e6 * 65/32 = 737.28e6 * 65/48 = 998.4e6

    Only these two images are accepted; anything else is a hard error so a
    wrong rate can never silently produce a mis-scaled CIR.
    """

    def __init__(self, native_hz):
        hz = float(native_hz)
        if abs(hz - CG600_HZ) < 1.0:
            self.name, self.label = "cg600", "737p28"
            self.tx_interp, self.tx_decim = 48, 65
            self.pdu, self.taps_dir = "65_48", "resampler_65_48"
        elif abs(hz - CG400_HZ) < 1.0:
            self.name, self.label = "cg400", "491p52"
            self.tx_interp, self.tx_decim = 32, 65
            self.pdu, self.taps_dir = "65_32", "resampler_65_32"
        else:
            raise SystemExit(
                "--native-rate must be 737.28e6 (CG600) or 491.52e6 (CG400); "
                "got %r" % native_hz)
        self.hz = hz
        self.work_per_native = WORK_HZ / hz

    def tx_native(self, work_iq):
        """998.4 MS/s CF32 work grid -> native CF32 (one-shot TX)."""
        return resample_poly(np.asarray(work_iq, dtype=np.complex128),
                             self.tx_interp, self.tx_decim).astype(np.complex64)

    def make_pdu_resampler(self, taps, res_workers, sc16_scale):
        """Build the native->998.4 PDU resampler matching this profile."""
        if self.pdu == "65_48":
            # The 65/48 PDU block exposes Sc16ScalePolicy but (yet) no
            # persistent worker pool; one-shot process per PDU.
            return uwb.pdu_rational_resampler_ccf_65_48(
                taps, WORK_HZ, True,
                uwb.pdu_resampler_emit_policy.FullWindow, 2097152, sc16_scale)
        return uwb.pdu_rational_resampler_ccf_65_32(
            taps, WORK_HZ, True, 2097152, int(res_workers), sc16_scale)

# UDP CIR datagram, unified for both live scripts (base and sweep): a 44-byte
# UCR2 header + 116 complex64 taps (always, zeros if fail).
#   magic "UCR2" | pulse_id u32 | status u16 | tap_count u16
#   | sfd_metric f32 | cir_peak_metric f32 | peak_tap i32 | estimator_us u32
#   | freq_hz f64 | freq_offset_hz f64
# f64 is required because f32 cannot resolve kHz-level CFO at 6.5 GHz.
# UCR1 (28-byte header, no frequency) is kept only for the receiver's legacy
# parse path.
CIR_UDP_MAGIC = b"UCR1"                       # legacy parse-only
CIR_UDP_HDR = struct.Struct("<4sIHHffiI")     # legacy parse-only
CIR_UDP_TAPS = 116
CIR_UDP_MAGIC_V2 = b"UCR2"
CIR_UDP_HDR_V2 = struct.Struct("<4sIHHffiIdd")
CIR_UDP_FREQ_UNKNOWN = float("nan")
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
    """Non-blocking UDP sink for CIR PDUs. Always sends a UCR2 datagram.

    ``freq_lookup(pulse_id) -> (freq_hz, freq_offset_hz) | None`` supplies the
    per-pulse centre frequency.  When it is missing or does not resolve, the
    two frequency fields are NaN.  The wire format is identical for the base
    and sweep apps.
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
        freq_hz = CIR_UDP_FREQ_UNKNOWN
        freq_off = CIR_UDP_FREQ_UNKNOWN
        if self.freq_lookup is not None:
            fr = self.freq_lookup(pulse_id)
            if fr is not None:
                freq_hz, freq_off = fr
                self.sent_freq += 1
        hdr = CIR_UDP_HDR_V2.pack(
            CIR_UDP_MAGIC_V2, pulse_id, status, self.tap_count, sfd_m,
            peak_m, peak_tap, est_us, float(freq_hz), float(freq_off))
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


def _mean_min_max(vals):
    if not vals:
        return 0.0, 0.0, 0.0
    return sum(vals) / len(vals), min(vals), max(vals)


def print_timing_detail(echo, est):
    """Per-burst host-side step times (ms) for the bursts just run."""
    tim = [r for r in echo._timing if r.get("status") == "ok"]
    pub = list(echo._pub_timing)
    print("[timing] step                 mean_ms   min_ms   max_ms   n", flush=True)
    rows = [
        ("get_time_ms", "usrp get_time_now"),
        ("issue_ms", "rx issue_stream_cmd"),
        ("send_ms", "tx send (fifo)"),
        ("recv_ms", "rx recv (rf+wait)"),
        ("sc16_ms", "sc16 dump"),
        ("enqueue_ms", "enqueue to publisher"),
        ("uhd_ms", "one burst uhd total"),
    ]
    for key, label in rows:
        m, lo, hi = _mean_min_max([r[key] for r in tim if key in r])
        print("[timing] %-20s %8.3f %8.3f %8.3f  %d" % (
            label, m, lo, hi, sum(key in r for r in tim)), flush=True)
    print("[timing] publisher (samples=%s)" % (
        pub[0]["samples"] if pub else "-"), flush=True)
    for key, label in (("contig_ms", "ascontiguousarray"), ("tolist_ms", "rx.tolist()"),
                       ("pmt_ms", "init_c32vector"), ("pub_ms", "message_port_pub"),
                       ("total_ms", "publisher total")):
        m, lo, hi = _mean_min_max([r[key] for r in pub])
        print("[timing] %-20s %8.3f %8.3f %8.3f  %d" % (
            label, m, lo, hi, len(pub)), flush=True)
    print("[timing] %-20s mean=%.1f us max=%.1f us (c++ resampler+estimator)" % (
        "est service", est.service_mean_us(), est.service_max_us()), flush=True)


def start_live_stats(echo, est, wr, udp, stop_evt, pri_s, res=None):
    def loop():
        t0 = time.monotonic()
        n_iter = 0
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
                "est_q=%d est_drop=%d tx_err=%d wr_hz=%.1f udp_hz=%.1f "
                "udp_ok_hz=%.1f udp_eagain=%d service_us_mean=%d max=%d "
                "pri_hz=%.1f" % (
                    dt,
                    (echo_ok - prev[1]) / dt,
                    (cir_ok - prev[2]) / dt,
                    (cir_fail - prev[3]) / dt,
                    int(est.queue_depth()),
                    est_drop,
                    echo._tx_send_error,
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
            # Backlog time series (rectification section 6.3): every 5 s
            # record the producer-consumer depth of each stage so a soak can
            # prove there is no long-term positive slope.
            n_iter += 1
            if res is not None and n_iter % 5 == 0:
                print("backlog res_in=%d res_out=%d est_in=%d est_done=%d "
                      "est_drop=%d est_q=%d wr=%d echo_pub=%d" % (
                          res.pdus_received(), res.pdus_emitted(),
                          est.pdus_received(), est.pdus_completed(),
                          est_drop, int(est.queue_depth()), wr_ok,
                          echo._pub_ok), flush=True)
            prev = (t1, echo_ok, cir_ok, cir_fail, est_drop, wr_ok,
                    udp_n, udp_ok)
    th = threading.Thread(target=loop, name="cir_live", daemon=True)
    th.start()
    return th


def rx_geometry(rate, pre_us, sync_reps, range_m, tail_us,
                tx_native_samples=0, pad_us=8.0,
                tx_interp=32, tx_decim=65):
    """RX window must cover the full native TX burst, not just SYNC+SFD.

    With STS+PHR/PSDU the HRP packet is ~191 us; SYNC+SFD is only ~73 us.
    FPGA RX length is rounded up to a multiple of 4.  ``tx_interp/tx_decim``
    convert work-grid counts (998.4 MS/s) to native samples: 32/65 for CG400,
    48/65 for CG600.
    """
    pre = llround(pre_us * 1e-6 * rate)
    sync = ceildiv(sync_reps * SPS * int(tx_interp), int(tx_decim))
    sfd = ceildiv(SFD_SYMS_4Z2 * SPS * int(tx_interp), int(tx_decim))
    rng = int(math.ceil(2.0 * range_m / C_LIGHT * rate))
    tail = llround(tail_us * 1e-6 * rate)
    pad = llround(float(pad_us) * 1e-6 * rate)
    body = int(tx_native_samples) if tx_native_samples else (sync + sfd)
    rx = pre + body + rng + pad + tail
    rx = (rx + 3) // 4 * 4
    return pre, sync, sfd, rng, tail, pad, rx


def cir_publish_native(pre_native, sync_reps, cal_native, native_hz=CG600_HZ,
                       cir_pre=16, cir_post=100, cir_skip=10,
                       margin_native=4096):
    """Upper bound on the native samples the CIR estimator can read.

    ``estimate_radar_cir`` reads windows of ``SPS + pre + post - 1`` work
    samples at repetition offsets ``k*sps`` for ``k = skip .. reps-1``,
    starting at ``origin = pre_guard + cal``.  The full RX window is sized
    for the whole TX burst (hundreds of us); publishing all of it to a PMT
    every pulse costs milliseconds of GIL time and can starve the timed TX
    loop.  This returns a conservative publish length covering every read
    the estimator can make, plus a native-sample margin.
    """
    ratio = WORK_HZ / float(native_hz)
    count = max(0, int(sync_reps) - int(cir_skip))
    wlen = SPS + int(cir_pre) + int(cir_post) - 1
    need_work = ((float(pre_native) + float(cal_native)) * ratio
                 + count * SPS + wlen)
    return int(math.ceil(need_work / ratio)) + int(margin_native)


def default_cal_delay_native(native_hz):
    """Firmware/rate starting calibration delay (the CIR zero-delay anchor).

    CG400: 334 native @491.52 MS/s measured on that image.

    CG600: the naive rate scale of that CG400 value is 501 native, but that
    is WRONG by +591.5 native (0.80 us).  The CG600 DDC/decimation front end
    (the CG400 image has no DDC; the CG600 is a /4 DDC at 2949.12 MS/s) delays
    the TX->RX loopback by that much, so the scaled default placed the entire
    SYNC preamble outside the 116-tap CIR window and every live CIR was noise
    (peak_tap uniform-random 0..115, metric ~1e-3; the CIR origin sat ~801
    work samples before the preamble).  Measured 1092.5 native by aligning
    the code phase of a captured 737.28 RX burst: with it, peak_tap pins to
    cir_pre (16) and metric_mean rises ~90x to ~0.07 (> the CG400 baseline).

    The sweep app's peak servo still refines this anchor at runtime.
    """
    if abs(float(native_hz) - CG600_HZ) < 1.0:
        return 1092.5
    return float(round(334.0 * float(native_hz) / CG400_HZ))


class TimedUhdEcho(gr.basic_block):
    """Downsample 998.4 TX PDU to the native rate, timed USRP burst, emit
    native RX PDU.  The native rate (CG400 491.52 / CG600 737.28 MS/s) comes
    from the ``NativeRateProfile`` so the TX/RX ratios and window geometry
    cannot drift apart."""

    def __init__(self, args, profile, freq, tx_ch, rx_ch, tx_ant, rx_ant,
                 gain_tx, gain_rx, pre_us, range_m, tail_us, sync_reps,
                 cal_delay_native, arm_delay_s, pri_s, max_pulses,
                 rx_dump_dir="", min_lead_s=0.002, timing_path="",
                 sc16_dump_dir="", rx_pad_us=8.0,
                 code_index=DEFAULT_CODE_INDEX, sfd_mode=SFD_MODE,
                 publish_native=0):
        gr.basic_block.__init__(self, name="timed_uhd_echo",
                                in_sig=None, out_sig=None)
        self._profile = profile
        self.rate = float(profile.hz)
        self.tx_interp = int(profile.tx_interp)
        self.tx_decim = int(profile.tx_decim)
        self.native_label = profile.label
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
            rx_geometry(self.rate, pre_us, sync_reps, range_m, tail_us, 0,
                        self.rx_pad_us, self.tx_interp, self.tx_decim)
        if int(publish_native) < 0:
            publish_native = cir_publish_native(
                self.pre, self.sync_reps, self.cal_delay_native, self.rate)
        self.publish_native = int(publish_native)

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
        self._pub_timing = []
        self._sc16_iq = None
        self._sc16_jsonl = None
        self._sc16_offset = 0
        self._sc16_written = 0
        self._tx_send_error = 0
        self.status = {
            "tx_rate": tx_rate, "rx_rate": rx_rate,
            "tx_ant": self._usrp.get_tx_antenna(self.tx_ch),
            "rx_ant": self._usrp.get_rx_antenna(self.rx_ch),
            "rx_window": self.rx_len, "pre": self.pre,
            "publish_native": self.publish_native,
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
                        self.tail_us, int(self._native.size), self.rx_pad_us,
                        self.tx_interp, self.tx_decim)
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
        problems = []
        while sent < n:
            chunk = min(maxp, n - sent)
            if sent + chunk >= n:
                md.end_of_burst = True
            nsent = self._tx_stream.send(buf[:, sent:sent + chunk], md, timeout)
            if nsent <= 0:
                problems.append("send_stalled")
                break
            if nsent < chunk:
                # Short send: the radio drained the FIFO before we refilled
                # it (TX underflow shows up here; uhd's TXMetadata binding
                # does not expose error_code).
                problems.append("short_send")
            sent += nsent
            md.has_time_spec = False
            md.start_of_burst = False
        return sent, ";".join(problems)

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
        native = self._profile.tx_native(tx_work)
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
            t_contig = time.perf_counter()
            # The CIR estimator only reads the preamble region; publishing
            # the whole TX-burst-sized window costs ~ms of GIL time per pulse
            # and can starve the timed TX loop (uhd reports TX underflow).
            if 0 < self.publish_native < rx.size:
                rx = rx[:self.publish_native]
            lst = rx.tolist()
            t_tolist = time.perf_counter()
            rx_pmt = pmt.init_c32vector(int(rx.size), lst)
            t_pmt = time.perf_counter()
            meta = self._make_rx_meta(pulse_id)
            self.message_port_pub(pmt.intern("rx"), pmt.cons(meta, rx_pmt))
            t_pub = time.perf_counter()
            self._pub_ok += 1
            self._pub_timing.append({
                "pulse_id": int(pulse_id),
                "samples": int(rx.size),
                "contig_ms": (t_contig - t0) * 1e3,
                "tolist_ms": (t_tolist - t_contig) * 1e3,
                "pmt_ms": (t_pmt - t_tolist) * 1e3,
                "pub_ms": (t_pub - t_pmt) * 1e3,
                "total_ms": (t_pub - t0) * 1e3,
            })
            if pulse_id < 3 or pulse_id % 100 == 99:
                r = self._pub_timing[-1]
                print("pub pulse=%d samples=%d contig=%.3fms tolist=%.2fms "
                      "pmt=%.2fms pub=%.3fms total=%.2fms q=%d" % (
                          pulse_id, r["samples"], r["contig_ms"],
                          r["tolist_ms"], r["pmt_ms"], r["pub_ms"],
                          r["total_ms"], self._pub_q.qsize()), flush=True)

    def _one_burst(self, pulse_id):
        t_in = time.perf_counter()
        now = self._usrp.get_time_now().get_real_secs()
        t_now = time.perf_counter()
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
            "get_time_ms": (t_now - t_in) * 1e3,
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
        t_after_issue = time.perf_counter()
        sent, send_err = self._send(self._native, t_tx, timeout=2.0)
        t_after_send = time.perf_counter()
        if send_err:
            self._tx_send_error += 1
        timeout = max(0.5, (t_tx - now) + self.rx_len / self.rate + 0.25)
        rx, got, first, status, err = self._recv(self.rx_len, timeout)
        t_after_recv = time.perf_counter()
        rec["issue_ms"] = (t_after_issue - t_issue) * 1e3
        rec["send_ms"] = (t_after_send - t_after_issue) * 1e3
        rec["recv_ms"] = (t_after_recv - t_after_send) * 1e3
        rec["uhd_ms"] = (t_after_recv - t_issue) * 1e3
        rec["tx_sent"] = sent
        rec["tx_error"] = send_err
        rec["rx_got"] = got
        rec["rx_first"] = first
        rec["status"] = status
        rec["error"] = err or send_err
        self._timing.append(rec)

        if self.rx_dump_dir and got > 0:
            os.makedirs(self.rx_dump_dir, exist_ok=True)
            rx[:got].tofile(os.path.join(self.rx_dump_dir,
                                         "pulse_%04d.cf32" % pulse_id))
        t_sc16 = time.perf_counter()
        if status == "ok" and got == self.rx_len:
            self._write_sc16_packet(pulse_id, rx)
        rec["sc16_ms"] = (time.perf_counter() - t_sc16) * 1e3
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
        t_enq = time.perf_counter()
        ok = self._enqueue_rx(pulse_id, rx)
        rec["enqueue_ms"] = (time.perf_counter() - t_enq) * 1e3
        return ok

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


class _EmptyQueue:
    """Stands in for TimedUhdEcho._pub_q when there is no Python publisher."""

    def empty(self):
        return True


class CppPduEcho:
    """Adapter exposing the TimedUhdEcho attribute surface over the C++
    UwbRealtimeEchoTimer PDU block (``--echo-backend cpp-pdu``).

    The C++ block is message-only: ONE native SC16 schedule PDU arms a
    device-time grid and the dedicated radio worker produces one RX burst
    PDU per slot.  All UHD I/O and PMT construction happen in C++/on the
    worker, so there is no Python timed path and no per-pulse
    ``tolist()/init_c32vector`` GIL cost on the critical path.  Retune and
    calibration are pushed through ``set_freq``/``set_cal_delay_native``
    and applied at a burst boundary by the worker.
    """

    def __init__(self, a, native, profile):
        self._profile = profile
        self.rate = float(profile.hz)
        self.tx_interp = int(profile.tx_interp)
        self.tx_decim = int(profile.tx_decim)
        self.native_label = profile.label
        self.freq = float(a.freq)
        self.code_index = int(a.code_index)
        self.sfd_mode = SFD_MODE
        self.tx_ch = int(a.tx_channel)
        self.rx_ch = int(a.rx_channel)
        self.pri_s = float(a.pri_s)
        self.sync_reps = int(a.sync_reps)
        self.cal_delay_native = float(a.cal_delay_native)
        self.pre_us = float(a.pre_guard_us)
        self.range_m = 15.0
        self.tail_us = float(a.tail_guard_us)
        self.rx_pad_us = float(a.rx_pad_us)
        self.arm_delay_s = float(a.arm_delay_s)
        self.max_pulses = int(a.pulses)

        # Same peak-normalisation + native geometry as TimedUhdEcho.
        wave = np.asarray(native, dtype=np.complex64)
        peak = float(np.max(np.abs(wave))) or 1.0
        self._native = (wave / peak * 0.8).astype(np.complex64)
        (self.pre, self.sync_n, self.sfd_n, self.rng_n, self.tail,
         self.pad_n, self.rx_len) = rx_geometry(
            self.rate, self.pre_us, self.sync_reps, self.range_m,
            self.tail_us, int(self._native.size), self.rx_pad_us,
            self.tx_interp, self.tx_decim)
        if int(a.publish_native) < 0:
            pub = cir_publish_native(self.pre, self.sync_reps,
                                     self.cal_delay_native, self.rate)
        else:
            pub = int(a.publish_native)
        self.publish_native = int(pub)

        # Python-compat attributes read by main()/live stats.
        self._pub_q = _EmptyQueue()
        self._timing = []
        self._pub_timing = []
        self._sc16_written = 0
        self._sc16_offset = 0

        self._arm_ticks = int(round(self.arm_delay_s * self.rate))
        self._tx_samples = int(self._native.size)
        pri_num = int(round(self.pri_s * self.rate))
        if pri_num <= 0:
            raise SystemExit("pri_s too small for the device tick grid")
        pre_guard_ticks = int(round(self.pre_us * 1e-6 * self.rate))

        self.blk = uwb.realtime_echo_timer_uhd(
            a.args, self.rate, a.tx_channel, a.rx_channel,
            a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx, self.freq,
            "internal", "internal",
            pri_num, 1, pre_guard_ticks, 65536, 1 << 20,
            4, 1000, 1 << 21, 1 << 21)
        self.status = {
            "backend": "cpp-pdu",
            "tx_rate": self.rate, "rx_rate": self.rate,
            "tx_ant": a.tx_antenna, "rx_ant": a.rx_antenna,
            "rx_window": self.rx_len, "pre": self.pre,
            "publish_native": self.publish_native,
            "pri_s": self.pri_s, "max_pulses": self.max_pulses,
            "rx_pad_us": self.rx_pad_us,
            "code_index": self.code_index, "sfd_mode": self.sfd_mode,
            "sync_reps": self.sync_reps,
            "tx_native": self._tx_samples,
            "rx_window_us": self.rx_len / self.rate * 1e6,
            "tx_us": self._tx_samples / self.rate * 1e6,
            "pad": self.pad_n,
        }

    # Live counters (properties so start_live_stats sees fresh values).
    @property
    def _ok(self):
        return int(self.blk.bursts_ok())

    @property
    def _fail(self):
        return int(self.blk.bursts_failed())

    @property
    def _late(self):
        return int(self.blk.late_slot_skips())

    @property
    def _pub_ok(self):
        return int(self.blk.bursts_published())

    @property
    def _tx_send_error(self):
        return 0

    def set_tx_native(self, wave):  # native is captured in __init__
        pass

    def start_publisher(self):
        pass

    def stop_publisher(self):
        pass

    def _schedule_meta(self):
        meta = pmt.make_dict()
        meta = pmt.dict_add(meta, pmt.intern("tx_samples"),
                            pmt.from_uint64(self._tx_samples))
        meta = pmt.dict_add(meta, pmt.intern("rx_samples"),
                            pmt.from_uint64(self.rx_len))
        meta = pmt.dict_add(meta, pmt.intern("schedule_index"),
                            pmt.from_uint64(0))
        meta = pmt.dict_add(meta, pmt.intern("pulse_id"), pmt.from_uint64(0))
        meta = pmt.dict_add(meta, pmt.intern("pulse_id_increment"),
                            pmt.from_uint64(1))
        meta = pmt.dict_add(meta, pmt.intern("burst_count"),
                            pmt.from_uint64(self.max_pulses))
        meta = pmt.dict_add(meta, pmt.intern("publish_native"),
                            pmt.from_uint64(self.publish_native))
        meta = pmt.dict_add(meta, pmt.intern("sample_rate"),
                            pmt.from_double(self.rate))
        # Same radar geometry the Python _make_rx_meta() sends.
        cap = self.rx_len - self.pre - self.tail
        if cap < 0:
            cap = self.rx_len
        meta = pmt.dict_add(meta, pmt.intern("window_start_sample"),
                            pmt.from_long(0))
        meta = pmt.dict_add(meta, pmt.intern("pre_guard_samples"),
                            pmt.from_long(self.pre))
        meta = pmt.dict_add(meta, pmt.intern("capture_samples"),
                            pmt.from_long(cap))
        meta = pmt.dict_add(meta, pmt.intern("post_guard_samples"),
                            pmt.from_long(self.tail))
        # sample_count is the PHYSICAL window (ROI only truncates payload).
        meta = pmt.dict_add(meta, pmt.intern("sample_count"),
                            pmt.from_long(self.rx_len))
        meta = pmt.dict_add(meta, pmt.intern("rx_capture_samples"),
                            pmt.from_long(self.rx_len))
        meta = pmt.dict_add(meta, pmt.intern("sync_repetitions"),
                            pmt.from_long(self.sync_reps))
        meta = pmt.dict_add(meta, pmt.intern("sfd_mode"),
                            pmt.intern(self.sfd_mode))
        meta = pmt.dict_add(meta, pmt.intern("code_index"),
                            pmt.from_long(self.code_index))
        meta = pmt.dict_add(meta, pmt.intern("sync_samples"),
                            pmt.from_long(self.sync_n))
        meta = pmt.dict_add(meta, pmt.intern("sfd_samples"),
                            pmt.from_long(self.sfd_n))
        meta = pmt.dict_add(meta, pmt.intern("tx_packet_samples"),
                            pmt.from_long(self._tx_samples))
        meta = pmt.dict_add(meta, pmt.intern("calibration_delay_native_samples"),
                            pmt.from_double(self.cal_delay_native))
        meta = pmt.dict_add(meta, pmt.intern("num_delay_samps"),
                            pmt.from_long(int(round(self.cal_delay_native))))
        meta = pmt.dict_add(meta, pmt.intern("source"),
                            pmt.intern("x410_echo"))
        return meta

    def run_schedule(self):
        # Must be called AFTER tb.start(): the block's start() prepares the
        # UHD backend, after which the device clock is readable.
        t0 = self.blk.device_time_ticks() + self._arm_ticks
        meta = self._schedule_meta()
        meta = pmt.dict_add(meta, pmt.intern("t0_ticks"), pmt.from_long(t0))
        payload = fc32_to_sc16(self._native)
        sched = pmt.cons(
            meta, pmt.init_s16vector(int(payload.size), payload.tolist()))
        self.blk._post(pmt.intern("schedule"), sched)
        print("cpp_pdu schedule t0_ticks=%d tx_samples=%d rx_samples=%d "
              "publish_native=%d bursts=%d" % (
                  t0, self._tx_samples, self.rx_len, self.publish_native,
                  self.max_pulses), flush=True)
        deadline = time.monotonic() + self.max_pulses * self.pri_s + 15.0
        while time.monotonic() < deadline:
            if self.blk.bursts_published() >= self.max_pulses:
                break
            time.sleep(0.02)
        if self.blk.bursts_published() < self.max_pulses:
            print("WARN cpp_pdu schedule timeout published=%d/%d last_error=%s"
                  % (self.blk.bursts_published(), self.max_pulses,
                     self.blk.last_error()), flush=True)
        return (int(self.blk.bursts_ok()), int(self.blk.bursts_failed()),
                int(self.blk.late_slot_skips()))


def build_parser(add_help=True):
    p = argparse.ArgumentParser(
        add_help=add_help,
        description="X410 CG600 (737.28 MS/s, default) / CG400 (491.52 MS/s) "
                    "monostatic HRP echo CIR.  Native rate and pulse shape "
                    "default to the CG600 chain (--native-rate 737.28e6, "
                    "--pulse-shape legacy); the sweep app reuses this parser.")
    p.add_argument("--args", default="addr=192.168.10.2")
    p.add_argument("--use-dpdk", action="store_true",
                   help="Use the DPDK transport for the QSFP data link "
                        "(adds use_dpdk=1 to --args).  Requires root (mlx5 "
                        "DevX + hugetlbfs) and a separate management link via "
                        "--mgmt-addr; without it the kernel-UDP path is used.")
    p.add_argument("--mgmt-addr", default=None,
                   help="X410 management (RJ45) IP for DPDK runs, e.g. "
                        "--mgmt-addr=192.168.20.133.  UHD's MPM RPC cannot "
                        "ride the DPDK link, so this must be a link DPDK does "
                        "not occupy.")
    p.add_argument("--native-rate", type=float, default=DEFAULT_NATIVE_HZ,
                   help="X410 native sample rate: 737.28e6 (CG600, default) or "
                        "491.52e6 (CG400). Selects the TX work->native ratio "
                        "(48/65 or 32/65) and the RX PDU resampler (65/48 or "
                        "65/32); both are exact (x65/.. = 998.4e6).")
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
    p.add_argument("--cal-delay-native", type=float, default=None,
                   help="Native-sample calibration delay (CIR zero-delay "
                        "anchor). Default None picks the firmware default: 334 "
                        "native @491.52 (CG400) or 1092.5 native @737.28 "
                        "(CG600, which has a DDC; a naive rate scale of 501 is "
                        "off by ~0.80 us and puts the SYNC outside the CIR "
                        "window). The sweep app's peak servo refines it.")
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
    p.add_argument("--pulse-shape", default="legacy",
                   choices=["legacy", "linear", "minphase", "gaussian",
                            "blackman", "external"],
                   help="TX pulse shaping on the 998.4 work grid.  Default "
                        "'legacy' is the standard full-band BPRF pulse (the "
                        "48-tap core fitted to the 737.28 MS/s reference); it "
                        "needs the CG600 737.28 MS/s front end -- the CG400 "
                        "491.52 limit rings near +/-245.76 MHz, for which the "
                        "narrowband 'linear'/'minphase'/'gaussian'/'blackman' "
                        "low-tail shapes exist.")
    p.add_argument("--pulse-sigma-ns", type=float, default=2.5)
    p.add_argument("--pulse-bw-mhz", type=float, default=200.0)
    p.add_argument("--pulse-taps", default="",
                   help="Raw float32 causal FIR taps at 998.4 MS/s; overrides "
                        "--pulse-shape (external pulse), e.g. "
                        "testdata/uwb_hrp_tx/pulse_minphase_rc160_240.f32")
    p.add_argument("--output", required=True)
    p.add_argument("--taps", default="")
    p.add_argument("--echo-backend", default=None,
                   choices=["python", "cpp-pdu"],
                   help="cpp-pdu (default) = C++ UwbRealtimeEchoTimer "
                        "message-only grid: no Python timed path, no per-pulse "
                        "PMT/GIL, retune at burst boundaries; python = legacy "
                        "TimedUhdEcho per-pulse PMT publisher (fallback, "
                        "emits TX underflow 'U' when retuning/servoed). "
                        "When unset and --dump-rx/--dump-sc16 is given, it "
                        "auto-selects python (cpp-pdu has no dump sink yet).")
    p.add_argument("--res-workers", type=int, default=1,
                   help="Native->998.4 PDU resampler persistent FIR worker "
                        "threads (65/32 CG400 only; 1 keeps single-thread "
                        "results).  The 65/48 CG600 block has no worker pool "
                        "yet and ignores this.")
    p.add_argument("--res-sc16-scale", choices=["auto", "raw", "unit"],
                   default="auto",
                   help="PDU resampler SC16 input amplitude contract: 'unit' = "
                        "float(int16)/32768 (matches the legacy Python FC32 "
                        "radar chain), 'raw' = float(int16) (legacy "
                        "scheduled-capture chain), 'auto' = unit for "
                        "cpp-pdu and raw otherwise")
    p.add_argument("--template", default="")
    p.add_argument("--publish-native", type=int, default=-1,
                   help="native samples published to the CIR estimator each "
                        "pulse: -1 auto (preamble + CIR span), 0 = full RX "
                        "window, >0 explicit; smaller cuts host GIL time and "
                        "timed-TX underflow")
    p.add_argument("--timing-detail", action="store_true",
                   help="after the run, print per-burst host step times "
                        "(get_time, issue, tx send, rx recv, sc16, publisher, "
                        "estimator); run with --pulses 1..3 for one packet")
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


def resolve_echo_backend(a):
    """Resolve an unset --echo-backend.

    cpp-pdu is the default (no Python timed path).  --dump-rx/--dump-sc16
    are only implemented by the python backend, so an unset backend with a
    dump request falls back to python with a note, keeping the documented
    dump commands working.  An explicitly requested cpp-pdu + dump still
    warns and disables the dump in main().
    """
    if a.echo_backend is None:
        if a.dump_rx or a.dump_sc16:
            a.echo_backend = "python"
            print("NOTE --echo-backend unset with --dump-rx/--dump-sc16: "
                  "using the python backend (cpp-pdu has no dump sink yet)",
                  flush=True)
        else:
            a.echo_backend = "cpp-pdu"
    return a.echo_backend


def _split_device_args(text):
    """Parse a UHD device-args string into an ordered list of (key, value)."""
    pairs = []
    for item in (text or "").split(","):
        item = item.strip()
        if not item:
            continue
        key, sep, value = item.partition("=")
        pairs.append((key.strip() if sep else item, value.strip() if sep else ""))
    return pairs


def _join_device_args(pairs):
    return ",".join(k if v == "" else "%s=%s" % (k, v) for k, v in pairs)


def resolve_dpdk_args(a):
    """Fold --use-dpdk / --mgmt-addr into a.args (before the USRP is built).

    UHD's MPM control (RPC) is kernel-UDP only -- there is no RPC-over-DPDK
    path -- so mgmt_addr must name a link DPDK does not occupy, while addr is
    the QSFP link that DPDK takes over.  This host routes management over the
    X410 RJ45 and data over the ConnectX QSFP.
    """
    if not getattr(a, "use_dpdk", False):
        return a.args
    pairs = _split_device_args(a.args)
    keys = dict(pairs)
    if getattr(a, "mgmt_addr", None):
        keys["mgmt_addr"] = a.mgmt_addr
    if not keys.get("mgmt_addr"):
        raise SystemExit(
            "--use-dpdk needs the X410 management link: pass "
            "--mgmt-addr=<RJ45 IP>, or put mgmt_addr=<IP> in --args.  It "
            "must be a link DPDK does not use (UHD has no RPC-over-DPDK).")
    if not keys.get("addr"):
        raise SystemExit(
            "--use-dpdk needs the QSFP data address: --args must contain "
            "addr=<IP> (default addr=192.168.10.2).")
    keys["use_dpdk"] = "1"
    head = [(k, keys[k]) for k in ("mgmt_addr", "addr", "use_dpdk")]
    rest = [(k, v) for k, v in pairs
            if k not in ("mgmt_addr", "addr", "use_dpdk")]
    a.args = _join_device_args(head + rest)
    return a.args


def dpdk_preflight(a):
    """Fail early, with guidance, if the host cannot do a DPDK run.

    The mlx5 PMD needs DevX (CAP_NET_ADMIN) and DPDK needs hugetlbfs, so a
    DPDK run must be root; report it before the GNU Radio graph is built.
    """
    if not getattr(a, "use_dpdk", False):
        return
    if os.geteuid() != 0:
        script = os.path.relpath(sys.argv[0]) if sys.argv and sys.argv[0] \
            else "x410_cg400_hrp_echo_cir.py"
        raise SystemExit(
            "--use-dpdk requires root (mlx5 DevX + hugetlbfs).  Re-run as:\n"
            "  sudo -E python3 %s <same args>\n"
            "  (add -E so the user env/PYTHONPATH survives sudo)" % script)
    free = None
    try:
        with open("/proc/meminfo") as fh:
            for line in fh:
                if line.startswith("HugePages_Free:"):
                    free = int(line.split()[1])
                    break
    except OSError:
        free = None
    if free is not None and free <= 0:
        raise SystemExit(
            "--use-dpdk found no free hugepages (/proc/meminfo "
            "HugePages_Free=0); reserve 2M pages, e.g. add "
            "vm.nr_hugepages=512 to /etc/sysctl.d and reboot.")
    print("dpdk_transport args=[%s] root=yes hugepages_free=%s" % (
        a.args, "?" if free is None else free), flush=True)


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
    resolve_echo_backend(a)
    resolve_dpdk_args(a)
    dpdk_preflight(a)
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
    profile = NativeRateProfile(a.native_rate)
    if a.cal_delay_native is None:
        a.cal_delay_native = default_cal_delay_native(profile.hz)
    print("native_rate=%.1f MS/s (%s) tx=%d/%d pdu=%s work_per_native=%.6f "
          "cal_delay_native=%.1f" % (
              profile.hz / 1e6, profile.name, profile.tx_interp,
              profile.tx_decim, profile.pdu, profile.work_per_native,
              a.cal_delay_native), flush=True)
    taps = a.taps or os.path.join(
        repo, "testdata", profile.taps_dir, "taps_quality_minorder.txt")
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
    native = profile.tx_native(samples)
    print("hrp_tx_998p4_samples=%d native_%s=%d resample_ms=%.2f "
          "insert_sts=%s pulse_shape=%s pulse_taps=%d pulse_center=%d "
          "code_index=%d preamble_length=%d sfd_mode=%s"
          % (samples.size, profile.label, native.size,
             (time.perf_counter() - t_rs) * 1e3,
             insert_sts, src.pulse_shape(), src.pulse_taps(),
             src.pulse_center_taps(), a.code_index, a.sync_reps, SFD_MODE),
          flush=True)
    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f" % (
        a.pulses, a.pri_s, (1.0 / a.pri_s), a.pulses * a.pri_s), flush=True)

    if a.echo_backend == "cpp-pdu" and (a.dump_rx or a.dump_sc16):
        print("WARN --dump-rx/--dump-sc16 are unsupported with "
              "--echo-backend cpp-pdu; disabling dumps", flush=True)
        a.dump_rx = False
        a.dump_sc16 = False
    # require-SFD searches a window around the predicted origin; the auto
    # ROI formula only bounds the predicted-timing CIR reads.  Until a
    # proven SFD-search upper bound exists, fall back to the full physical
    # window for auto ROI.
    if a.require_sfd and a.publish_native < 0:
        print("WARN --require-sfd with --publish-native -1 (auto ROI): no "
              "proven SFD-search upper bound -> falling back to full window "
              "(0)", flush=True)
        a.publish_native = 0
    dump_dir = os.path.join(a.output, "rx_iq") if a.dump_rx else ""
    sc16_dir = a.output if a.dump_sc16 else ""
    if a.echo_backend == "cpp-pdu":
        echo = CppPduEcho(a, native, profile)
        echo_out_port = "burst"
    else:
        echo = TimedUhdEcho(
            a.args, profile, a.freq, a.tx_channel, a.rx_channel,
            a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx,
            a.pre_guard_us, 15.0, a.tail_guard_us, a.sync_reps,
            a.cal_delay_native, a.arm_delay_s, a.pri_s, a.pulses, dump_dir,
            a.min_lead_s, timing_path, sc16_dir, a.rx_pad_us, a.code_index,
            SFD_MODE, publish_native=a.publish_native)
        echo.set_tx_native(native)
        echo_out_port = "rx"
    print("echo_backend=%s" % a.echo_backend, flush=True)
    print("uhd_probe", echo.status, flush=True)
    if a.dump_sc16:
        tx_sc16_path = os.path.join(a.output, "tx_%s.sc16" % profile.label)
        fc32_to_sc16(echo._native).tofile(tx_sc16_path)
        print("wrote_tx_sc16", tx_sc16_path, "samples=%d" % echo._native.size,
              flush=True)

    if a.res_sc16_scale == "unit":
        sc16_scale = uwb.Sc16ScalePolicy.UnitRange
    elif a.res_sc16_scale == "raw":
        sc16_scale = uwb.Sc16ScalePolicy.RawInteger
    else:  # auto: cpp-pdu publishes native SC16 -> normalize like UHD FC32
        sc16_scale = (uwb.Sc16ScalePolicy.UnitRange
                      if a.echo_backend == "cpp-pdu"
                      else uwb.Sc16ScalePolicy.RawInteger)
    res = profile.make_pdu_resampler(taps, a.res_workers, sc16_scale)
    print("resampler pdu=%s workers=%d sc16_scale=%s" % (
        profile.pdu, int(a.res_workers), a.res_sc16_scale), flush=True)
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
        udp = CirUdpSink(
            a.udp_host, int(a.udp_port), CIR_UDP_TAPS,
            freq_lookup=lambda pid: (echo.freq, 0.0))
        print("udp_cir %s:%s framed=UCR2(+freq_hz,freq_offset_hz) "
              "always_send_taps=%d nonblock" % (
                  a.udp_host, a.udp_port, CIR_UDP_TAPS), flush=True)

    tb = gr.top_block("x410_cg400_hrp_echo_cir")
    # The cpp-pdu path is the C++ block itself; the wrapper only carries the
    # Python-compat attribute surface.
    echo_block = echo if a.echo_backend == "python" else echo.blk
    tb.msg_connect((echo_block, echo_out_port), (res, "packet"))
    tb.msg_connect((res, "packet"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if udp is not None:
        tb.msg_connect((est, "cir"), (udp, "cir"))
    # Do not attach message_debug on a 100 Hz soak: queue_full status
    # PDUs would flood the print block and stall the message system.

    tb.start()
    echo.start_publisher()
    live_stop = threading.Event()
    live_th = start_live_stats(echo, est, wr, udp, live_stop, a.pri_s, res)
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

    if a.timing_detail:
        print_timing_detail(echo, est)

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
        "tx_send_error": echo._tx_send_error,
        "publish_native": echo.publish_native,
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
        "udp_sent_freq": 0 if udp is None else udp.sent_freq,
        "udp_eagain": 0 if udp is None else udp.dropped,
        "cir": cir_stats,
        "timing": timing_stats,
        "output": a.output,
        "sc16_packets": echo._sc16_written,
        "sc16_samples": echo._sc16_offset,
        "dump_sc16": bool(a.dump_sc16),
        "freq_hz": a.freq,
        "native_rate_hz": profile.hz,
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
        "echo_backend": a.echo_backend,
        "res_workers": int(a.res_workers),
        "echo_queue_hwm": (int(echo.blk.queue_high_watermark())
                           if a.echo_backend == "cpp-pdu" else 0),
        "echo_worker_us_mean": (int(echo.blk.mean_worker_us())
                                if a.echo_backend == "cpp-pdu" else 0),
        "echo_worker_us_max": (int(echo.blk.max_worker_us())
                               if a.echo_backend == "cpp-pdu" else 0),
        "echo_slot_lead_us_last": (int(echo.blk.slot_lead_us_last())
                                   if a.echo_backend == "cpp-pdu" else 0),
        "echo_issue_rx_us_mean": (int(echo.blk.issue_rx_us_mean())
                                  if a.echo_backend == "cpp-pdu" else 0),
        "echo_tx_send_us_mean": (int(echo.blk.tx_send_us_mean())
                                 if a.echo_backend == "cpp-pdu" else 0),
        "echo_rx_collect_us_mean": (int(echo.blk.rx_collect_us_mean())
                                    if a.echo_backend == "cpp-pdu" else 0),
        "echo_pdu_build_us_mean": (int(echo.blk.pdu_build_us_mean())
                                   if a.echo_backend == "cpp-pdu" else 0),
        "echo_pdu_publish_us_mean": (int(echo.blk.pdu_publish_us_mean())
                                     if a.echo_backend == "cpp-pdu" else 0),
        "rx_window": echo.rx_len,
        "rx_window_us": echo.rx_len / profile.hz * 1e6,
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
                    "X410 monostatic HRP echo, native SC16 RX windows "
                    "(%s)" % profile.name
                ),
                "sample_format": "sc16",
                "dtype": "int16",
                "byte_order": "little-endian",
                "layout": "interleaved_iq",
                "bytes_per_complex": 4,
                "iq_scale": IQ_SCALE,
                "sample_index_base": 0,
                "freq_hz": a.freq,
                "rate_native_hz": profile.hz,
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
                "rx_window_us": echo.rx_len / profile.hz * 1e6,
                "files": {
                    "capture.iq": "concatenated native SC16 packets",
                    "capture.jsonl": "one JSON object per packet",
                    "tx_%s.sc16" % profile.label:
                        "timed TX burst (native %.2f MS/s)"
                        % (profile.hz / 1e6),
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
