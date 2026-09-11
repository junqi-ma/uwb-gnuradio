#!/usr/bin/env python3
"""CG400 X410 echo-chain simulation: HRP synth, raw SC16 dump, PDU 65/32, CIR.

Mirrors gr-uwb/apps/x410_cg400_hrp_echo_cir.py without UHD or hardware:

  UwbHrpPacketSource.samples()      # 998.4 CF32, IEEE 802.15.4a BPRF
    -> 32/65 downsample (scipy)     # 491.52 MS/s CG400 native
    -> SimTimedEcho                 # fullwin RX window, wall-clock paced
    -> PDU 65/32                    # 491.52 -> 998.4
    -> UwbRadarCirEstimator
    -> UwbCirWriter

With --dump-rx the raw RX windows are written through UwbPacketWriter in the
same three-file contract as the CG400 fullwin dumps:

  <output>/capture.iq            concatenated native SC16 windows
  <output>/capture.jsonl         one JSON object per pulse
  <output>/capture_metadata.json run-level metadata
  <output>/tx_491p52.sc16        simulated native TX waveform

Window geometry matches the recorded fullwin dump:

  rx_len = pre_guard + tx_native + rx_pad + range_guard + tail_guard
           (rounded up to a multiple of 4 native samples)

Defaults (491.52 MS/s): pre 983 + tx 93988 + pad 3932 + range 50 + tail
11796 -> 110752 samples.  Echo paths are placed at native index
pre_guard + echo_delay, so the default delay (== --cal-delay-native)
reproduces the calibrated zero delay.  The CIR chain still consumes the
truncated echo window (pre + SYNC + SFD + range + tail).
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import json
import math
import os
import queue
import threading
import time

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.signal import resample_poly

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


uwb = load_uwb()

C_LIGHT = 299792458.0
CG400_HZ = 491520000.0
WORK_HZ = 998400000.0
SPS = 1016
SFD_SYMS_4Z2 = 8
NATIVE_DECIM = 65
NATIVE_INTERP = 32
IQ_SCALE = 32768.0


def llround(x):
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def ceildiv(a, b):
    return -(-a // b)


def hex_to_bytes(s):
    h = "".join(c for c in s if c in "0123456789abcdefABCDEF")
    if len(h) % 2:
        raise ValueError("psdu hex must have even length")
    return list(bytes.fromhex(h))


def rx_geometry(rate, pre_us, sync_reps, range_m, tail_us):
    pre = llround(pre_us * 1e-6 * rate)
    sync = ceildiv(sync_reps * SPS * NATIVE_INTERP, NATIVE_DECIM)
    sfd = ceildiv(SFD_SYMS_4Z2 * SPS * NATIVE_INTERP, NATIVE_DECIM)
    rng = int(math.ceil(2.0 * range_m / C_LIGHT * rate))
    tail = llround(tail_us * 1e-6 * rate)
    rx = pre + sync + sfd + rng + tail
    rx = (rx + 3) // 4 * 4
    return pre, sync, sfd, rng, tail, rx


def fullwin_geometry(rate, pre, rng, tail, pad_us, tx_native):
    pad = llround(pad_us * 1e-6 * rate)
    rx = pre + tx_native + pad + rng + tail
    rx = (rx + 3) // 4 * 4
    return pad, rx


def to_sc16_interleaved(x, scale=IQ_SCALE):
    x = np.asarray(x)
    i = np.clip(np.rint(x.real * scale), -32768, 32767).astype(np.int16)
    q = np.clip(np.rint(x.imag * scale), -32768, 32767).astype(np.int16)
    out = np.empty(2 * x.size, dtype=np.int16)
    out[0::2] = i
    out[1::2] = q
    return out


def parse_delays(text, default):
    if not text.strip():
        return [float(default)]
    return [float(v) for v in text.split(",") if v.strip()]


def parse_gains(text):
    if not text.strip():
        return [complex(1.0, 0.0)]
    return [complex(v) for v in text.split(",") if v.strip()]


class SimTimedEcho(gr.basic_block):
    """Simulated X410 timed echo: numpy channel, fullwin SC16 raw + echo PDU."""

    def __init__(self, rate, pre_us, range_m, tail_us, sync_reps,
                 cal_delay_native, arm_delay_s, pri_s, max_pulses,
                 echo_delays, echo_gains, noise_std, noise_seed,
                 pad_us=8.0, dump_raw=False):
        gr.basic_block.__init__(self, name="sim_timed_echo",
                                in_sig=None, out_sig=None)
        self.rate = float(rate)
        self.pre_us = float(pre_us)
        self.range_m = float(range_m)
        self.tail_us = float(tail_us)
        self.sync_reps = int(sync_reps)
        self.cal_delay_native = float(cal_delay_native)
        self.arm_delay_s = float(arm_delay_s)
        self.pri_s = float(pri_s)
        self.max_pulses = int(max_pulses)
        self.echo_delays = [float(d) for d in echo_delays]
        self.echo_gains = [complex(g) for g in echo_gains]
        if len(self.echo_delays) != len(self.echo_gains):
            raise ValueError("echo_delays and echo_gains must have equal length")
        if not self.echo_delays:
            raise ValueError("at least one echo path is required")
        self.noise_std = float(noise_std)
        self.pad_us = float(pad_us)
        self.dump_raw = bool(dump_raw)
        self.pre, self.sync_n, self.sfd_n, self.rng_n, self.tail, self.rx_len = \
            rx_geometry(rate, pre_us, sync_reps, range_m, tail_us)
        self.pad, _ = fullwin_geometry(
            rate, self.pre, self.rng_n, self.tail, pad_us, 0)
        self.fullwin_len = 0

        self.message_port_register_in(pmt.intern("tx"))
        self.message_port_register_out(pmt.intern("rx"))
        self.message_port_register_out(pmt.intern("status"))
        if self.dump_raw:
            self.message_port_register_out(pmt.intern("raw"))
        self.set_msg_handler(pmt.intern("tx"), self._on_tx)

        self._rng = np.random.default_rng(int(noise_seed))
        self._native = None
        self._done = 0
        self._ok = 0
        self._fail = 0
        self._overrun = 0
        self._pub_ok = 0
        self._host_times = []
        self._pub_q = queue.Queue(maxsize=64)
        self._stop_pub = threading.Event()
        self._pub_thread = None
        self.status = {
            "rate": self.rate,
            "rx_window": self.rx_len,
            "pre": self.pre,
            "pri_s": self.pri_s,
            "max_pulses": self.max_pulses,
        }

    def set_tx_native(self, wave):
        peak = float(np.max(np.abs(wave))) or 1.0
        self._native = (np.asarray(wave, dtype=np.complex64) / peak
                        * 0.8).astype(np.complex64)
        self.pad, self.fullwin_len = fullwin_geometry(
            self.rate, self.pre, self.rng_n, self.tail, self.pad_us,
            int(self._native.size))

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

    def _make_raw_meta(self, pulse_id):
        ws = pulse_id * self.fullwin_len
        meta = pmt.make_dict()
        meta = pmt.dict_add(meta, pmt.intern("packet_id"),
                            pmt.from_uint64(pulse_id))
        meta = pmt.dict_add(meta, pmt.intern("schedule_index"),
                            pmt.from_uint64(pulse_id))
        meta = pmt.dict_add(meta, pmt.intern("sample_rate"),
                            pmt.from_double(self.rate))
        meta = pmt.dict_add(meta, pmt.intern("window_start_sample"),
                            pmt.from_long(ws))
        meta = pmt.dict_add(meta, pmt.intern("start_sample"),
                            pmt.from_long(ws + self.pre))
        meta = pmt.dict_add(meta, pmt.intern("predicted_start_sample"),
                            pmt.from_long(ws + self.pre))
        meta = pmt.dict_add(meta, pmt.intern("trigger_sample"),
                            pmt.from_long(ws + self.pre))
        meta = pmt.dict_add(meta, pmt.intern("detection_metric"),
                            pmt.from_double(0.0))
        meta = pmt.dict_add(meta, pmt.intern("pre_trigger_samples"),
                            pmt.from_long(self.pre))
        meta = pmt.dict_add(meta, pmt.intern("pre_guard_samples"),
                            pmt.from_long(self.pre))
        meta = pmt.dict_add(meta, pmt.intern("capture_samples"),
                            pmt.from_long(self.fullwin_len - self.pre - self.tail))
        meta = pmt.dict_add(meta, pmt.intern("post_guard_samples"),
                            pmt.from_long(self.tail))
        meta = pmt.dict_add(meta, pmt.intern("iq_scale"),
                            pmt.from_double(IQ_SCALE))
        meta = pmt.dict_add(meta, pmt.intern("capture_mode"),
                            pmt.intern("x410_echo"))
        meta = pmt.dict_add(meta, pmt.intern("lock_state"), pmt.intern("timed"))
        return meta

    def _make_rx_meta(self, pulse_id):
        cap = self.rx_len - self.pre - self.tail
        if cap < 0:
            cap = self.rx_len
        meta = pmt.make_dict()
        meta = pmt.dict_add(meta, pmt.intern("pulse_id"), pmt.from_uint64(pulse_id))
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
        meta = pmt.dict_add(meta, pmt.intern("sfd_mode"), pmt.intern("4z2"))
        meta = pmt.dict_add(meta, pmt.intern("code_index"), pmt.from_long(9))
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
        meta = pmt.dict_add(meta, pmt.intern("source"), pmt.intern("sim_echo"))
        return meta

    def _enqueue_rx(self, pulse_id, full, echo):
        try:
            self._pub_q.put_nowait((pulse_id, full, echo))
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
            pulse_id, full, echo = item
            t0 = time.perf_counter()
            if full is not None:
                sc16 = to_sc16_interleaved(full)
                vec = pmt.init_s16vector(int(sc16.size), sc16.tolist())
                self.message_port_pub(pmt.intern("raw"),
                                      pmt.cons(self._make_raw_meta(pulse_id), vec))
            rx_pmt = pmt.init_c32vector(int(echo.size), echo.tolist())
            self.message_port_pub(pmt.intern("rx"),
                                  pmt.cons(self._make_rx_meta(pulse_id), rx_pmt))
            self._pub_ok += 1
            dt = (time.perf_counter() - t0) * 1e3
            if pulse_id < 3 or pulse_id % 100 == 99:
                print("pub pulse=%d pmt_ms=%.2f q=%d" % (
                    pulse_id, dt, self._pub_q.qsize()), flush=True)

    def _apply_path(self, rx, tx, delay, gain):
        offset = self.pre + delay
        if abs(delay - round(delay)) < 1e-9:
            start = int(round(offset))
            k0 = max(0, -start)
            k1 = min(tx.size, rx.size - start)
            if k1 > k0:
                rx[start + k0:start + k1] += gain * tx[k0:k1]
            return
        base = self.pre
        k0 = max(0, -base)
        k1 = min(tx.size, rx.size - base)
        y = np.zeros(rx.size, dtype=np.complex128)
        if k1 > k0:
            y[base + k0:base + k1] = tx[k0:k1]
        xi = np.arange(rx.size, dtype=np.float64)
        xq = xi - delay
        idx = (xq >= 0.0) & (xq <= rx.size - 1)
        vals = np.zeros(rx.size, dtype=np.complex128)
        if np.any(idx):
            re = PchipInterpolator(xi, y.real)(xq[idx])
            im = PchipInterpolator(xi, y.imag)(xq[idx])
            vals[idx] = re + 1j * im
        rx += gain * vals

    def _sim_window(self):
        n = self.fullwin_len if self.dump_raw else self.rx_len
        rx = np.zeros(n, dtype=np.complex128)
        for delay, gain in zip(self.echo_delays, self.echo_gains):
            self._apply_path(rx, self._native, delay, gain)
        if self.noise_std > 0.0:
            noise = (self._rng.standard_normal(n)
                     + 1j * self._rng.standard_normal(n)) * self.noise_std
            rx += noise
        return rx.astype(np.complex64)

    def _one_burst(self, pulse_id):
        if self._native is None:
            self._fail += 1
            return False
        full = self._sim_window()
        echo = full[:self.rx_len] if self.dump_raw else full
        self._ok += 1
        self._host_times.append(time.monotonic())
        return self._enqueue_rx(pulse_id, full if self.dump_raw else None, echo)

    def run_schedule(self):
        if self._native is None:
            raise RuntimeError("TX waveform not set")
        self.start_publisher()
        t_host0 = time.monotonic() + self.arm_delay_s
        t_wall0 = time.perf_counter()
        for pulse_id in range(self.max_pulses):
            target = t_host0 + pulse_id * self.pri_s
            now = time.monotonic()
            if now < target:
                time.sleep(target - now)
            elif now - target > self.pri_s:
                self._overrun += 1
            self._one_burst(pulse_id)
            self._done = pulse_id + 1
            if pulse_id == 0 or (pulse_id + 1) % 100 == 0 \
                    or pulse_id + 1 == self.max_pulses:
                print("sched %d/%d ok=%d fail=%d overrun=%d host_s=%.3f" % (
                    pulse_id + 1, self.max_pulses, self._ok, self._fail,
                    self._overrun, time.perf_counter() - t_wall0), flush=True)
        return self._ok, self._fail, self._overrun

    def _on_tx(self, msg):
        if self._done >= self.max_pulses:
            return
        if not pmt.is_pair(msg):
            return
        raw = pmt.c32vector_elements(pmt.cdr(msg))
        tx_work = np.asarray(raw, dtype=np.complex64)
        if tx_work.size == 0:
            return
        native = resample_poly(tx_work.astype(np.complex128),
                               NATIVE_INTERP, NATIVE_DECIM)
        self.set_tx_native(native.astype(np.complex64))
        pulse_id = self._done
        self._one_burst(pulse_id)
        self._done += 1

    def host_times(self):
        return list(self._host_times)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--pulses", type=int, default=8)
    p.add_argument("--pri-s", type=float, default=0.01)
    p.add_argument("--rate-hz", type=float, default=0.0,
                   help="If set, PRI=1/rate and pulses=round(rate*duration)")
    p.add_argument("--duration-s", type=float, default=10.0)
    p.add_argument("--cal-delay-native", type=float, default=334.0)
    p.add_argument("--echo-delays", default="",
                   help="Comma-separated native-sample delays; default = cal delay")
    p.add_argument("--echo-gains", default="",
                   help="Comma-separated complex gains; default = 1.0")
    p.add_argument("--noise-std", type=float, default=0.0)
    p.add_argument("--noise-seed", type=int, default=1)
    p.add_argument("--sfd-search-margin", type=int, default=128)
    p.add_argument("--sfd-threshold", type=float, default=0.12)
    p.add_argument("--sync-refine-margin", type=int, default=32)
    p.add_argument("--sync-refine-threshold", type=float, default=0.02)
    p.add_argument("--tail-guard-us", type=float, default=24.0)
    p.add_argument("--pre-guard-us", type=float, default=2.0)
    p.add_argument("--rx-pad-us", type=float, default=8.0)
    p.add_argument("--range-m", type=float, default=15.0)
    p.add_argument("--psdu-hex",
                   default="47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221")
    p.add_argument("--sync-reps", type=int, default=64)
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
    p.add_argument("--dump-rx", action="store_true",
                   help="Write capture.iq/capture.jsonl/capture_metadata.json "
                        "(SC16 fullwin, same contract as the X410 CG400 dumps)")
    p.add_argument("--arm-delay-s", type=float, default=0.05)
    return p.parse_args()


def analyze_cir(jsonl_path, pulses):
    if not os.path.isfile(jsonl_path):
        return {"exists": False, "lines": 0}
    ok = 0
    fail = 0
    ids = []
    peak_taps = []
    metrics = []
    statuses = {}
    pre_sidelobes = []
    post_sidelobes = []
    cir_path = os.path.join(os.path.dirname(jsonl_path), "cir.cf32")
    cir_taps = None
    if os.path.isfile(cir_path):
        cir_taps = np.fromfile(cir_path, dtype=np.complex64)
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
                if cir_taps is not None:
                    off = int(rec.get("file_offset_taps", 0))
                    tc = int(rec.get("tap_count", 0))
                    if tc > 0 and off + tc <= cir_taps.size:
                        seg = np.abs(cir_taps[off:off + tc])
                        pt = int(rec.get("peak_tap", int(np.argmax(seg))))
                        if 0 <= pt < tc:
                            pk = max(float(seg[pt]), 1e-30)
                            if pt > 3:
                                pre_sidelobes.append(20 * np.log10(
                                    max(float(seg[:pt - 3].max()), 1e-30) / pk))
                            if pt + 3 < tc:
                                post_sidelobes.append(20 * np.log10(
                                    max(float(seg[pt + 3:].max()), 1e-30) / pk))
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
        "pre_sidelobe_db_max": max(pre_sidelobes) if pre_sidelobes else None,
        "post_sidelobe_db_max": max(post_sidelobes) if post_sidelobes else None,
    }


def summarize_pacing(times, pri_s):
    if len(times) < 2:
        return {"n": len(times)}
    dt = [b - a for a, b in zip(times, times[1:])]
    return {
        "n": len(times),
        "target_s": pri_s,
        "dt_min": min(dt),
        "dt_max": max(dt),
        "dt_mean": sum(dt) / len(dt),
        "late_over_1p5x": sum(1 for d in dt if d > 1.5 * pri_s),
    }


def write_capture_metadata(path, a, insert_sts, native, echoes, gains,
                           fullwin_len, pad, rng_n, pre, tail, raw_stats,
                           pulse_meta):
    meta = {
        "description": "CG400 monostatic HRP echo simulation, native SC16 "
                       "RX windows",
        "simulated": True,
        "sample_format": "sc16",
        "dtype": "int16",
        "byte_order": "little-endian",
        "layout": "interleaved_iq",
        "bytes_per_complex": 4,
        "iq_scale": IQ_SCALE,
        "sample_index_base": 0,
        "freq_hz": 6489600000.0,
        "rate_native_hz": CG400_HZ,
        "rate_work_hz": WORK_HZ,
        "code_index": 9,
        "sync_repetitions": a.sync_reps,
        "sfd_mode": "4z2",
        "insert_sts": bool(insert_sts),
        "pulse_shape": pulse_meta["shape"],
        "pulse_sigma_ns": pulse_meta["sigma_ns"],
        "pulse_bw_mhz": pulse_meta["bw_mhz"],
        "pulse_taps": pulse_meta["taps"],
        "pulse_center_taps": pulse_meta["center_taps"],
        "pulse_taps_file": pulse_meta["taps_file"],
        "packets": a.pulses,
        "pri_s": a.pri_s,
        "gain_tx": None,
        "gain_rx": None,
        "tx_channel": None,
        "rx_channel": None,
        "tx_antenna": None,
        "rx_antenna": None,
        "cal_delay_native_samples": a.cal_delay_native,
        "pre_guard_us": a.pre_guard_us,
        "tail_guard_us": a.tail_guard_us,
        "rx_pad_us": a.rx_pad_us,
        "rx_window_samples": fullwin_len,
        "rx_window_us": fullwin_len / CG400_HZ * 1e6,
        "files": {
            "capture.iq": "concatenated native SC16 packets",
            "capture.jsonl": "one JSON object per packet",
            "tx_491p52.sc16": "simulated TX waveform (native 491.52 MS/s)",
        },
        "matlab": "[x, meta] = read_uwb_packet('capture.iq','capture.jsonl',id)",
        "echo": {
            "tx_rate": CG400_HZ,
            "rx_rate": CG400_HZ,
            "tx_ant": None,
            "rx_ant": None,
            "rx_window": fullwin_len,
            "pre": pre,
            "pri_s": a.pri_s,
            "max_pulses": a.pulses,
            "rx_pad_us": a.rx_pad_us,
            "tx_native": int(native.size),
            "rx_window_us": fullwin_len / CG400_HZ * 1e6,
            "tx_us": native.size / CG400_HZ * 1e6,
            "pad": pad,
            "range_guard_native": rng_n,
            "tail_guard_native": tail,
            "echo_delays_native": list(echoes),
            "echo_gains": [[g.real, g.imag] for g in gains],
            "noise_std": a.noise_std,
        },
        "written_packets": raw_stats["packets"],
        "written_samples": raw_stats["samples"],
        "raw_dropped": raw_stats["dropped"],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
        f.write("\n")


def main():
    a = parse_args()
    insert_sts = False if a.no_sts else True
    if a.rate_hz and a.rate_hz > 0:
        a.pri_s = 1.0 / a.rate_hz
        a.pulses = int(round(a.rate_hz * a.duration_s))
    repo = find_repo_root()
    os.makedirs(a.output, exist_ok=True)
    taps = a.taps or os.path.join(
        repo, "testdata", "resampler_65_32", "taps_quality_minorder.txt")
    tmpl_path = a.template or os.path.join(a.output, "sync_template_live.cf32")

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
    src = uwb.hrp_packet_source(psdu, a.sync_reps, "4z2", 9, 0.8, a.pri_s,
                                False, insert_sts, False, pulse_shape,
                                a.pulse_sigma_ns, a.pulse_bw_mhz, pulse_taps)
    samples = np.array(src.samples(), dtype=np.complex64)
    if samples.size < SPS:
        raise SystemExit("HRP source produced %d samples" % samples.size)
    _tmpl_off = max(0, int(src.pulse_center_taps()) - 4)
    samples[_tmpl_off:_tmpl_off + SPS].tofile(tmpl_path)
    t_rs = time.perf_counter()
    native = resample_poly(samples.astype(np.complex128),
                           NATIVE_INTERP, NATIVE_DECIM).astype(np.complex64)
    print("hrp_tx_998p4_samples=%d native_491p52=%d resample_ms=%.2f "
          "insert_sts=%s pulse_shape=%s pulse_taps=%d pulse_center=%d"
          % (samples.size, native.size, (time.perf_counter() - t_rs) * 1e3,
             insert_sts, src.pulse_shape(), src.pulse_taps(),
             src.pulse_center_taps()), flush=True)
    pulse_meta = {
        "shape": src.pulse_shape(),
        "sigma_ns": src.pulse_sigma_ns(),
        "bw_mhz": src.pulse_bw_mhz(),
        "taps": src.pulse_taps(),
        "center_taps": src.pulse_center_taps(),
        "taps_file": src.pulse_taps_file(),
    }

    echoes = parse_delays(a.echo_delays, a.cal_delay_native)
    gains = parse_gains(a.echo_gains)
    if len(echoes) != len(gains):
        raise SystemExit("--echo-delays and --echo-gains must have equal length")

    echo = SimTimedEcho(
        CG400_HZ, a.pre_guard_us, a.range_m, a.tail_guard_us, a.sync_reps,
        a.cal_delay_native, a.arm_delay_s, a.pri_s, a.pulses,
        echoes, gains, a.noise_std, a.noise_seed, a.rx_pad_us, a.dump_rx)
    echo.set_tx_native(native)
    print("uhd_free_probe", echo.status, flush=True)
    print("echo_paths delays=%s gains=%s noise_std=%s" % (
        echoes, gains, a.noise_std), flush=True)
    if max(echoes) - a.cal_delay_native > a.sfd_search_margin:
        print("warning: echo delay exceeds sfd-search-margin %d; raise "
              "--sfd-search-margin or the estimator will report sfd_failed"
              % a.sfd_search_margin, flush=True)
    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f dump_rx=%s"
          % (a.pulses, a.pri_s, 1.0 / a.pri_s, a.pulses * a.pri_s,
             bool(a.dump_rx)), flush=True)

    raw_stats = {"packets": 0, "samples": 0, "dropped": 0}
    if a.dump_rx:
        tx_sc16 = to_sc16_interleaved(echo._native)
        tx_sc16.tofile(os.path.join(a.output, "tx_491p52.sc16"))
        print("dump_contract file=capture.iq sc16 window_native=%d "
              "bytes_per_pulse=%d total_estimate_mb=%.1f rx_window_us=%.3f"
              % (echo.fullwin_len, echo.fullwin_len * 4,
                 a.pulses * echo.fullwin_len * 4 / 1e6,
                 echo.fullwin_len / CG400_HZ * 1e6), flush=True)

    res = uwb.pdu_rational_resampler_ccf_65_32(taps, WORK_HZ, True, 2097152)
    est_q = max(64, min(1024, a.pulses + 8))
    est = uwb.radar_cir_estimator(
        tmpl_path, a.sync_reps, "4z2", 9, 16, 100, 10, 0,
        a.sfd_search_margin, a.sync_refine_margin, a.sfd_threshold,
        a.sync_refine_threshold, True, est_q)
    print("estimator sfd_search_margin=%d queue=%d" % (
        a.sfd_search_margin, est_q), flush=True)
    wr = uwb.cir_writer(a.output, "cir", True, 64)
    wr_iq = uwb.packet_writer(a.output, "capture", False) if a.dump_rx else None

    tb = gr.top_block("uwb_sim_cg400_echo_cir")
    tb.msg_connect((echo, "rx"), (res, "packet"))
    tb.msg_connect((res, "packet"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if wr_iq is not None:
        tb.msg_connect((echo, "raw"), (wr_iq, "packet"))

    tb.start()
    t_run = time.perf_counter()
    echo.run_schedule()
    sched_s = time.perf_counter() - t_run
    print("schedule_wall_s=%.3f" % sched_s, flush=True)

    deadline = time.time() + 8.0
    while time.time() < deadline:
        written = wr.frames_written() + wr.frames_failed()
        raw_ok = True if wr_iq is None else \
            (wr_iq.packets_written() + wr_iq.packets_dropped() >= echo._ok)
        if written >= echo._ok and est.drained() and echo._pub_q.empty() \
                and raw_ok:
            break
        time.sleep(0.05)
    echo.stop_publisher()
    tb.stop()
    tb.wait()
    try:
        wr.stop()
    except Exception:
        pass
    if wr_iq is not None:
        try:
            wr_iq.stop()
        except Exception:
            pass

    if wr_iq is not None:
        raw_stats = {
            "packets": wr_iq.packets_written(),
            "samples": int(wr_iq.samples_written()),
            "dropped": wr_iq.packets_dropped(),
        }
        write_capture_metadata(
            os.path.join(a.output, "capture_metadata.json"), a, insert_sts,
            native, echoes, gains, echo.fullwin_len, echo.pad, echo.rng_n,
            echo.pre, echo.tail, raw_stats, pulse_meta)

    jsonl = os.path.join(a.output, "cir.jsonl")
    cir_stats = analyze_cir(jsonl, a.pulses)
    summary = {
        "hrp_samples": int(samples.size),
        "native_samples": int(native.size),
        "native_rate": CG400_HZ,
        "work_rate": WORK_HZ,
        "pulses": a.pulses,
        "pri_s": a.pri_s,
        "rate_hz": 1.0 / a.pri_s,
        "schedule_wall_s": sched_s,
        "echo_ok": echo._ok,
        "echo_fail": echo._fail,
        "echo_overrun": echo._overrun,
        "echo_pub": echo._pub_ok,
        "echo_paths": [{"delay_native": d, "gain": [g.real, g.imag]}
                       for d, g in zip(echoes, gains)],
        "noise_std": a.noise_std,
        "rx_window_native": echo.rx_len,
        "dump_rx": bool(a.dump_rx),
        "raw": raw_stats,
        "raw_window_native": echo.fullwin_len if a.dump_rx else 0,
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
        "cir": cir_stats,
        "pacing": summarize_pacing(echo.host_times(), a.pri_s),
        "output": a.output,
    }
    with open(os.path.join(a.output, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    print("SUMMARY", json.dumps(summary), flush=True)

    ok = (summary["wr_ok"] == a.pulses and
          summary["echo_ok"] == a.pulses and
          summary["est_fail"] == 0 and
          summary["est_drop"] == 0 and
          cir_stats.get("ok") == a.pulses and
          cir_stats.get("missing_count", 1) == 0 and
          (not a.dump_rx or
           (raw_stats["packets"] == a.pulses and raw_stats["dropped"] == 0)))
    raise SystemExit(0 if ok else 3)


if __name__ == "__main__":
    main()
