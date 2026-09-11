#!/usr/bin/env python3
"""Live HRP synth + timed X410 TX/RX + PDU 65/32 + radar CIR.

Pipeline:
  UwbHrpPacketSource.samples()     # 998.4 CF32, C++ IEEE 802.15.4a BPRF
    -> TimedUhdEcho                # 32/65 downsample once, SC16 timed burst
    -> PDU 65/32                   # 491.52 -> 998.4
    -> UwbRadarCirEstimator
    -> UwbCirWriter

TX: UHD ch0 / TX/RX0 (front-panel ch1)
RX: UHD ch3 / RX1     (front-panel ch4)
Rate: CG400 491.52 MS/s

100 Hz soak uses a host schedule thread (UHD timed bursts) and an
async PMT publisher so numpy->PMT conversion does not steal TX lead time.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time

import numpy as np
from scipy.signal import resample_poly

import glob
import importlib.util

from gnuradio import gr, network
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
    sync = ceildiv(sync_reps * SPS * 32, 65)
    sfd = ceildiv(SFD_SYMS_4Z2 * SPS * 32, 65)
    rng = int(math.ceil(2.0 * range_m / C_LIGHT * rate))
    tail = llround(tail_us * 1e-6 * rate)
    rx = pre + sync + sfd + rng + tail
    rx = (rx + 3) // 4 * 4
    return pre, sync, sfd, rng, tail, rx


class TimedUhdEcho(gr.basic_block):
    """Downsample 998.4 TX PDU to 491.52, timed USRP burst, emit native RX PDU."""

    def __init__(self, args, rate, freq, tx_ch, rx_ch, tx_ant, rx_ant,
                 gain_tx, gain_rx, pre_us, range_m, tail_us, sync_reps,
                 cal_delay_native, arm_delay_s, pri_s, max_pulses,
                 rx_dump_dir="", min_lead_s=0.002, timing_path=""):
        gr.basic_block.__init__(self, name="timed_uhd_echo",
                                in_sig=None, out_sig=None)
        self.rate = float(rate)
        self.freq = float(freq)
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
        self.min_lead_s = float(min_lead_s)
        self.timing_path = timing_path
        self.pre, self.sync_n, self.sfd_n, self.rng_n, self.tail, self.rx_len = \
            rx_geometry(rate, pre_us, sync_reps, range_m, tail_us)

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
        self.status = {
            "tx_rate": tx_rate, "rx_rate": rx_rate,
            "tx_ant": self._usrp.get_tx_antenna(self.tx_ch),
            "rx_ant": self._usrp.get_rx_antenna(self.rx_ch),
            "rx_window": self.rx_len, "pre": self.pre,
            "pri_s": self.pri_s, "max_pulses": self.max_pulses,
        }

    def set_tx_native(self, wave):
        peak = float(np.max(np.abs(wave))) or 1.0
        self._native = (np.asarray(wave, dtype=np.complex64) / peak * 0.8).astype(np.complex64)

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
        self.start_publisher()
        t_host0 = time.perf_counter()
        for pulse_id in range(self.max_pulses):
            self._one_burst(pulse_id)
            self._done = pulse_id + 1
            if pulse_id == 0 or (pulse_id + 1) % 100 == 0 or pulse_id + 1 == self.max_pulses:
                dt = time.perf_counter() - t_host0
                print("sched %d/%d ok=%d fail=%d late=%d host_s=%.3f" % (
                    pulse_id + 1, self.max_pulses, self._ok, self._fail,
                    self._late, dt), flush=True)
        return self._ok, self._fail, self._late

    def _on_tx(self, msg):
        if self._done >= self.max_pulses:
            return
        if not self._cache_native_from_msg(msg):
            return
        pulse_id = self._done
        self._one_burst(pulse_id)
        self._done += 1


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--args", default="addr=192.168.20.2")
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
    p.add_argument("--sfd-search-margin", type=int, default=8192)
    p.add_argument("--sfd-threshold", type=float, default=0.12)
    p.add_argument("--sync-refine-margin", type=int, default=32)
    p.add_argument("--sync-refine-threshold", type=float, default=0.02)
    p.add_argument("--tail-guard-us", type=float, default=20.0)
    p.add_argument("--pre-guard-us", type=float, default=2.0)
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
    p.add_argument("--dump-rx", action="store_true")
    p.add_argument("--min-lead-s", type=float, default=0.002)
    p.add_argument("--arm-delay-s", type=float, default=0.25)
    p.add_argument("--udp-host", default="133.133.133.132",
                   help="CIR taps UDP destination (empty disables)")
    p.add_argument("--udp-port", default="12345")
    p.add_argument("--no-udp", action="store_true",
                   help="Do not send CIR taps over UDP")
    return p.parse_args()


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
    src = uwb.hrp_packet_source(psdu, a.sync_reps, "4z2", 9, 0.8, a.pri_s,
                                False, insert_sts, False, pulse_shape,
                                a.pulse_sigma_ns, a.pulse_bw_mhz, pulse_taps)
    samples = np.array(src.samples(), dtype=np.complex64)
    if samples.size < SPS:
        raise SystemExit("HRP source produced %d samples" % samples.size)
    _tmpl_off = max(0, int(src.pulse_center_taps()) - 4)
    samples[_tmpl_off:_tmpl_off + SPS].tofile(tmpl_path)
    t_rs = time.perf_counter()
    native = resample_poly(samples.astype(np.complex128), 32, 65).astype(np.complex64)
    print("hrp_tx_998p4_samples=%d native_491p52=%d resample_ms=%.2f "
          "insert_sts=%s pulse_shape=%s pulse_taps=%d pulse_center=%d"
          % (samples.size, native.size, (time.perf_counter() - t_rs) * 1e3,
             insert_sts, src.pulse_shape(), src.pulse_taps(),
             src.pulse_center_taps()), flush=True)
    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f" % (
        a.pulses, a.pri_s, (1.0 / a.pri_s), a.pulses * a.pri_s), flush=True)

    dump_dir = os.path.join(a.output, "rx_iq") if a.dump_rx else ""
    echo = TimedUhdEcho(
        a.args, CG400_HZ, a.freq, a.tx_channel, a.rx_channel,
        a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx,
        a.pre_guard_us, 15.0, a.tail_guard_us, a.sync_reps, a.cal_delay_native,
        a.arm_delay_s, a.pri_s, a.pulses, dump_dir, a.min_lead_s, timing_path)
    echo.set_tx_native(native)
    print("uhd_probe", echo.status, flush=True)

    res = uwb.pdu_rational_resampler_ccf_65_32(taps, 998.4e6, True, 2097152)
    est_q = max(64, min(1024, a.pulses + 8))
    est = uwb.radar_cir_estimator(
        tmpl_path, a.sync_reps, "4z2", 9, 16, 100, 10, 0,
        a.sfd_search_margin, a.sync_refine_margin, a.sfd_threshold,
        a.sync_refine_threshold, True, est_q)
    print("estimator sfd_search_margin=%d queue=%d" % (
        a.sfd_search_margin, est_q), flush=True)
    wr = uwb.cir_writer(a.output, "cir", True, 64)
    sock = None
    udp_on = (not a.no_udp) and bool(a.udp_host)
    if udp_on:
        sock = network.socket_pdu("UDP_CLIENT", a.udp_host, str(a.udp_port), 1472)
        print("udp_cir %s:%s mtu=1472 taps_only" % (a.udp_host, a.udp_port),
              flush=True)

    tb = gr.top_block("x410_cg400_hrp_echo_cir")
    tb.msg_connect((echo, "rx"), (res, "packet"))
    tb.msg_connect((res, "packet"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if sock is not None:
        tb.msg_connect((est, "cir"), (sock, "pdus"))
    # Do not attach message_debug on a 100 Hz soak: queue_full status
    # PDUs would flood the print block and stall the message system.

    tb.start()
    echo.start_publisher()
    t_run = time.perf_counter()
    echo.run_schedule()
    sched_s = time.perf_counter() - t_run
    print("schedule_wall_s=%.3f" % sched_s, flush=True)

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
        "cir": cir_stats,
        "timing": timing_stats,
        "output": a.output,
    }
    with open(os.path.join(a.output, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    print("SUMMARY", json.dumps(summary), flush=True)
    if os.path.isfile(jsonl):
        print("cir.jsonl_lines=%d" % cir_stats.get("lines", 0), flush=True)
        with open(jsonl, "r", encoding="utf-8") as f:
            lines = [ln.strip() for ln in f if ln.strip()]
        for ln in lines[:2] + lines[-2:]:
            print(ln, flush=True)

    ok = (summary["wr_ok"] == a.pulses and
          summary["echo_ok"] == a.pulses and
          summary["echo_late"] == 0 and
          cir_stats.get("ok") == a.pulses and
          cir_stats.get("missing_count", 1) == 0)
    raise SystemExit(0 if ok else 3)


if __name__ == "__main__":
    main()
