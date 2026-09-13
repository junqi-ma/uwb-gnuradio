#!/usr/bin/env python3
"""Streaming (gr-radar-style) X410 CG400 HRP echo CIR radar RX chain.

This app is the stream / tagged-stream sibling of the validated PDU chain in
``x410_cg400_hrp_echo_cir.py`` (imported as ``base``).  It replaces the
per-pulse PMT/PDU conversion of the whole native RX window with a fully
streamed chain, keeping only the CIR product on message/PDU ports:

    vector_source_c(TX native, repeat -> one packet per PRI)
      -> stream_to_tagged_stream(complex, 1, tx_samples, "packet_len")
      -> uwb.echo_timer_stream_uhd         (C++ tagged_stream_block; one timed
                                            burst per work() call; owns the USRP)
      -> uwb.rational_resampler_ccf_65_32  (tagged-stream, 491.52 -> 998.4,
                                            maps the radar tags to the work grid)
      -> blocks.tagged_stream_to_pdu(complex_t, "packet_len")
      -> uwb.radar_cir_estimator           (msg "rx" -> "cir")
      -> uwb.cir_writer / base.CirUdpSink / PeakAlignSink

No RX window is turned into a PMT c32vector until ``tagged_stream_to_pdu``;
the echo, the resampler and the bridge all move data on GNU Radio stream
buffers.  This removes the Python publisher / GIL hot spot of the base app.

TX geometry (native 491.52 MS/s)
--------------------------------
The HRP waveform is synthesised once on the 998.4 MS/s work grid
(``base.uwb.hrp_packet_source``) and decimated 32/65 to the native rate with
``base.resample_poly`` exactly like the base app.  ``tx_samples`` is that
native length.  ``rx_samples`` is the 7th return value of
``base.rx_geometry(CG400_HZ, pre_guard_us, sync_reps, 15.0, tail_guard_us,
tx_samples, rx_pad_us)``; the echo block re-derives the same window from
``tx_samples`` / ``rx_samples`` and tags every burst.

Buffer plan (items)
-------------------
``buffer_plan(tx_samples, rx_samples)`` documents the per-step minimum output
buffers (GNU Radio sizes a connection from the *upstream* block's
``set_min_output_buffer``).  A stream output is connected 1:1 here, so the
sender and receiver of each connection share one buffer:

    key          value                         shared with
    -----------  ----------------------------  ----------------------------
    tx_ts_out    2 * tx_samples                echo TX input
    echo_out     4 * rx_samples                resampler input
    res_in       2 * rx_samples                same buffer as echo_out
    res_out      2 * ceil(rx_samples * 65/32)  bridge input
    bridge_in    1 * ceil(rx_samples * 65/32)  same buffer as res_out

Because a connection has a single buffer, the app applies the larger value:
echo output = ``max(echo_out, res_in)`` = 4*rx and resampler output =
``max(res_out, bridge_in)`` = 2*work.  The numbers are logged in
``summary.json`` (``buffer_plan`` + ``buffer_applied``) and printed at
startup.  For the reference window (tx=94492, rx=109288 native samples) the
plan is::

    tx_ts_out = 188984
    echo_out  = 437152
    res_in    = 218576
    work      = 221992
    res_out   = 443984
    bridge_in = 221992

Frequency handling (``--freq-mode fixed|scan|manual``)
------------------------------------------------------
``fixed`` (default) behaves like a fixed base run.  The echo is stream-driven,
so ``scan`` / ``manual`` are applied by a small background thread
(``StreamFreqRetuner``) that watches ``echo.frames()``, resolves the target
with ``echo_cir_freq_plan.FreqPlan`` and calls ``echo.set_freq(hz)`` at each
dwell boundary.  The C++ block applies the retune at the next burst start, so
the boundary pulse itself may still have been acquired at the previous
frequency (documented limitation; the plan is recorded per pulse in
``freq_sweep.jsonl`` and in ``summary.json``).

Peak alignment
--------------
When ``--peak-target-tap > 0`` a ``PeakAlignSink``-style ``gr.basic_block``
subscribes to ``est.cir``, drives ``fp.PeakAlignController`` and writes the
updated calibration back with ``echo.set_cal_delay_native(...)`` (the PDU
sweep app writes the Python attribute instead).  ``--peak-*`` arguments match
the sweep app.

The app is import-safe when ``uwb.echo_timer_stream_uhd`` is not built yet:
the symbol is only referenced in ``main``, which raises a clear runtime error.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import x410_cg400_hrp_echo_cir as base          # noqa: E402
import echo_cir_freq_plan as fp                 # noqa: E402

import pmt                                      # noqa: E402
from gnuradio import gr, blocks                 # noqa: E402


# --------------------------------------------------------------------------
# Buffer plan (pure-Python; unit-tested by test_echo_stream_buffers.py without
# importing GNU Radio via AST extraction of this function).
# --------------------------------------------------------------------------
def buffer_plan(tx_samples, rx_samples):
    """Minimum per-step output buffers in items (see the module docstring).

    ``work = ceil(rx_samples * 65 / 32)`` is the resampler's output window
    length on the 998.4 MS/s work grid for one native RX window.  Integer
    arithmetic only, so this stays importable without math/numpy.
    """
    tx = int(tx_samples)
    rx = int(rx_samples)
    work = (rx * 65 + 31) // 32
    return {
        "tx_ts_out": 2 * tx,
        "echo_out": 4 * rx,
        "res_in": 2 * rx,
        "res_out": 2 * work,
        "bridge_in": work,
    }


def apply_buffer_plan(tx_ts, echo, res, plan):
    """Apply ``buffer_plan`` to the blocks that own the shared buffers.

    GNU Radio has no ``set_min_input_buffer``: the resampler input buffer is
    the echo output buffer, and the bridge input buffer is the resampler
    output buffer.  The plan values are therefore collapsed with ``max``.
    """
    tx_ts.set_min_output_buffer(0, int(plan["tx_ts_out"]))
    echo.set_min_output_buffer(0, max(int(plan["echo_out"]),
                                      int(plan["res_in"])))
    res.set_min_output_buffer(0, max(int(plan["res_out"]),
                                     int(plan["bridge_in"])))
    return {
        "tx_ts_out": int(plan["tx_ts_out"]),
        "echo_out": max(int(plan["echo_out"]), int(plan["res_in"])),
        "res_out": max(int(plan["res_out"]), int(plan["bridge_in"])),
    }


def _counter(obj, name, default=0):
    """Read an int counter method/attribute defensively."""
    try:
        v = getattr(obj, name)
        if callable(v):
            v = v()
        return int(v)
    except Exception:
        return int(default)


# --------------------------------------------------------------------------
# Live stats / summary helpers (stream counters instead of the base block's
# Python `_ok` / `_pub_q` attributes, which do not exist on the C++ block).
# --------------------------------------------------------------------------
def start_stream_live_stats(echo, res, est, wr, udp, stop_evt, pri_s):
    def snap():
        return {
            "bursts_ok": _counter(echo, "bursts_ok"),
            "bursts_failed": _counter(echo, "bursts_failed"),
            "late": _counter(echo, "late_slot_skips"),
            "frames": _counter(echo, "frames"),
            "windows": _counter(res, "windows"),
            "cir_ok": _counter(est, "pdus_completed"),
            "cir_fail": _counter(est, "pdus_failed"),
            "est_drop": _counter(est, "pdus_dropped"),
            "wr_ok": _counter(wr, "frames_written"),
            "udp_n": 0 if udp is None else udp.sent,
            "udp_ok": 0 if udp is None else udp.sent_ok,
        }

    def loop():
        t0 = time.monotonic()
        prev = snap()
        while not stop_evt.wait(1.0):
            t1 = time.monotonic()
            dt = t1 - t0
            if dt <= 0:
                continue
            cur = snap()
            print(
                "live dt=%.3f echo_ok_hz=%.1f echo_fail_hz=%.1f late=%d "
                "frames=%d res_wins=%d cir_ok_hz=%.1f cir_fail_hz=%.1f "
                "est_q=%d est_drop=%d wr_hz=%.1f udp_hz=%.1f udp_ok_hz=%.1f "
                "udp_eagain=%d service_us_mean=%d max=%d pri_hz=%.1f" % (
                    dt,
                    (cur["bursts_ok"] - prev["bursts_ok"]) / dt,
                    (cur["bursts_failed"] - prev["bursts_failed"]) / dt,
                    cur["late"] - prev["late"],
                    cur["frames"],
                    (cur["windows"] - prev["windows"]) / dt,
                    (cur["cir_ok"] - prev["cir_ok"]) / dt,
                    (cur["cir_fail"] - prev["cir_fail"]) / dt,
                    int(est.queue_depth()),
                    cur["est_drop"],
                    (cur["wr_ok"] - prev["wr_ok"]) / dt,
                    (cur["udp_n"] - prev["udp_n"]) / dt,
                    (cur["udp_ok"] - prev["udp_ok"]) / dt,
                    0 if udp is None else udp.dropped,
                    int(est.service_mean_us()),
                    int(est.service_max_us()),
                    (1.0 / pri_s) if pri_s > 0 else 0.0),
                flush=True)
            if cur["est_drop"] > prev["est_drop"]:
                print("NOTE estimator dropped %d frames (queue_full); "
                      "radio ok is not CIR/UDP ok"
                      % (cur["est_drop"] - prev["est_drop"]), flush=True)
            prev = cur
            t0 = t1

    th = threading.Thread(target=loop, name="stream_live", daemon=True)
    th.start()
    return th


def print_stream_counters(echo, res, est, wr, udp):
    """``--timing-detail`` for the stream path: the C++ echo has no per-burst
    host timing log, so print its counters plus the downstream stage stats."""
    print("[stream] echo bursts_ok=%d bursts_failed=%d late_slot_skips=%d "
          "frames=%d tx_samples=%d rx_samples=%d" % (
              _counter(echo, "bursts_ok"), _counter(echo, "bursts_failed"),
              _counter(echo, "late_slot_skips"), _counter(echo, "frames"),
              _counter(echo, "tx_samples"), _counter(echo, "rx_samples")),
          flush=True)
    print("[stream] res windows=%d input_items=%d output_items=%d "
          "tag_errors=%d" % (
              _counter(res, "windows"), _counter(res, "input_items"),
              _counter(res, "output_items"), _counter(res, "tag_errors")),
          flush=True)
    print("[stream] est rx=%d enq=%d done=%d fail=%d drop=%d invalid=%d "
          "q_hwm=%d service_us_mean=%d max=%d" % (
              _counter(est, "pdus_received"), _counter(est, "pdus_enqueued"),
              _counter(est, "pdus_completed"), _counter(est, "pdus_failed"),
              _counter(est, "pdus_dropped"), _counter(est, "invalid_inputs"),
              _counter(est, "queue_high_watermark"),
              _counter(est, "service_mean_us"),
              _counter(est, "service_max_us")), flush=True)
    print("[stream] writer ok=%d fail=%d invalid=%d udp_sent=%d ok=%d "
          "fail=%d eagain=%d" % (
              _counter(wr, "frames_written"), _counter(wr, "frames_failed"),
              _counter(wr, "frames_invalid"),
              0 if udp is None else udp.sent,
              0 if udp is None else udp.sent_ok,
              0 if udp is None else udp.sent_fail,
              0 if udp is None else udp.dropped), flush=True)


# --------------------------------------------------------------------------
# Frequency retune thread
# --------------------------------------------------------------------------
class StreamFreqRetuner:
    """Apply an ``fp.FreqPlan`` to the stream echo at burst boundaries.

    Because the echo is stream-driven there is no host schedule loop to
    hook, so a daemon thread polls ``echo.frames()`` and calls
    ``echo.set_freq(hz)`` whenever the frequency for the next pulse differs
    from the last one applied.  ``set_freq`` is expected to be thread-safe and
    to take effect at the next burst start (C++ contract).
    """

    def __init__(self, echo, plan, nominal_hz, stop_evt, settle_s=0.0,
                 poll_s=0.001):
        self.echo = echo
        self.plan = plan
        self.nominal = float(nominal_hz)
        self.stop_evt = stop_evt
        self.settle_s = max(0.0, float(settle_s))
        self.poll_s = max(0.0002, float(poll_s))
        self.freq_by_pulse = {}
        self.records = []
        self.retune_count = 0
        self.retune_fail = 0
        self.stop_requested = False
        self.last_target = None
        self._th = None

    def start(self):
        if self._th is not None:
            return
        self._th = threading.Thread(target=self._loop, name="stream_freq",
                                    daemon=True)
        self._th.start()

    def stop(self, timeout=2.0):
        self.stop_evt.set()
        if self._th is not None:
            self._th.join(timeout)
            self._th = None

    def _loop(self):
        while not self.stop_evt.is_set():
            if self.plan is not None and getattr(self.plan, "stop_requested",
                                                 False):
                self.stop_requested = True
                break
            try:
                pid = int(self.echo.frames())
            except Exception:
                break
            target = (self.plan.freq_for(pid)
                      if self.plan is not None else self.nominal)
            if target is None:
                break
            target = float(target)
            if self.last_target is None:
                # The echo constructor already tuned to the nominal centre;
                # treat the first observed target as the baseline.
                self.last_target = target
            elif abs(target - self.last_target) >= 1.0:
                try:
                    self.echo.set_freq(target)
                    self.retune_count += 1
                except Exception as exc:
                    self.retune_fail += 1
                    print("[freq] set_freq %.3f MHz failed, keep %.3f MHz: %s"
                          % (target / 1e6,
                             self._current() / 1e6, exc), flush=True)
                self.last_target = target
                if self.settle_s > 0.0:
                    self.stop_evt.wait(self.settle_s)
            cur = self._current()
            self.freq_by_pulse[pid] = (cur, cur - self.nominal)
            self.records.append({
                "pulse_id": pid,
                "target_hz": target,
                "freq_hz": cur,
                "freq_offset_hz": cur - self.nominal,
            })
            self.stop_evt.wait(self.poll_s)

    def _current(self):
        try:
            return float(self.echo.freq())
        except Exception:
            return float(self.last_target if self.last_target is not None
                         else self.nominal)


def make_freq_lookup(a, plan, retuner):
    """Per-pulse ``(freq_hz, freq_offset_hz)`` for the UDP sink.

    ``fixed``/``scan`` are deterministic in ``pulse_id``; ``manual`` can only
    be read back from the retune thread's observations.
    """
    if a.freq_mode == "manual":
        return lambda pid: retuner.freq_by_pulse.get(int(pid))

    nominal = float(a.freq)

    def lookup(pid):
        f = float(plan.freq_for(int(pid)))
        return (f, f - nominal)

    return lookup


def build_freq_records(a, plan, retuner, frames_done):
    """Per-pulse frequency table for the summary / CIR-by-frequency join."""
    if a.freq_mode == "manual":
        return list(retuner.records)
    nominal = float(a.freq)
    out = []
    for pid in range(int(frames_done)):
        f = float(plan.freq_for(pid))
        out.append({
            "pulse_id": pid,
            "target_hz": f,
            "freq_hz": f,
            "freq_offset_hz": f - nominal,
        })
    return out


# --------------------------------------------------------------------------
# Optional SC16 RX recorder (--dump-sc16)
# --------------------------------------------------------------------------
class StreamSc16Recorder(gr.sync_block):
    """Debug tee: write the native RX stream as SC16 ``capture.iq``.

    Connected to the echo output in parallel with the resampler.  Every burst
    is exactly ``window_samples`` long, so the file is a concatenation of
    fixed-size windows; ``capture.jsonl`` records the constant geometry and a
    pulse id derived from the running sample count.

    This is a Python sink at the native rate; it is intended only for short
    dump runs and is off by default (``--dump-sc16``).
    """

    def __init__(self, path, window_samples, pre_guard, capture, post_guard,
                 rate, freq_lookup=None):
        gr.sync_block.__init__(self, name="stream_sc16_recorder",
                               in_sig=[np.complex64], out_sig=None)
        self.window_samples = int(window_samples)
        self.pre_guard = int(pre_guard)
        self.capture_samples = int(capture)
        self.post_guard = int(post_guard)
        self.rate = float(rate)
        self.freq_lookup = freq_lookup
        self._iq = open(path, "wb")
        self._jsonl = open(os.path.join(os.path.dirname(path),
                                        "capture.jsonl"), "w", encoding="utf-8")
        self._offset = 0
        self.windows = 0

    def work(self, input_items, output_items):
        x = input_items[0]
        if x.size:
            self._iq.write(base.fc32_to_sc16(x).tobytes())
            self._offset += int(x.size)
            while (self.windows + 1) * self.window_samples <= self._offset:
                pid = self.windows
                rec = {
                    "packet_id": pid,
                    "start_sample": pid * self.window_samples,
                    "sample_rate": int(round(self.rate)),
                    "sample_count": self.window_samples,
                    "file_offset_samples": pid * self.window_samples,
                    "pre_trigger_samples": self.pre_guard,
                    "sample_format": "sc16",
                    "iq_scale": base.IQ_SCALE,
                    "window_start_sample": pid * self.window_samples,
                    "pre_guard_samples": self.pre_guard,
                    "capture_samples": self.capture_samples,
                    "post_guard_samples": self.post_guard,
                    "schedule_index": pid,
                    "capture_mode": "x410_echo_stream",
                    "lock_state": "timed",
                }
                if self.freq_lookup is not None:
                    fr = self.freq_lookup(pid)
                    if fr is not None:
                        rec["freq_hz"] = float(fr[0])
                        rec["freq_offset_hz"] = float(fr[1])
                self._jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")
                self.windows += 1
            self._jsonl.flush()
        return len(x)

    def stop(self):
        for fh in (self._iq, self._jsonl):
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass


# --------------------------------------------------------------------------
# Peak-align servo sink
# --------------------------------------------------------------------------
class PeakAlignSink(gr.basic_block):
    """Closed loop: read CIR taps and steer ``echo`` calibration delay.

    Same controller as the PDU sweep app (``fp.PeakAlignController``) but the
    stream echo is a C++ block, so the updated axis is pushed with
    ``echo.set_cal_delay_native(...)`` instead of assigning an attribute.
    Message-only, no streaming ports.
    """

    def __init__(self, controller, echo):
        gr.basic_block.__init__(self, name="stream_peak_align_sink",
                                in_sig=None, out_sig=None)
        self.controller = controller
        self.echo = echo
        self.message_port_register_in(pmt.intern("cir"))
        self.set_msg_handler(pmt.intern("cir"), self._on_cir)

    def _on_cir(self, msg):
        if not pmt.is_pair(msg):
            return
        meta = pmt.car(msg)
        vec = pmt.cdr(msg)
        if not pmt.is_dict(meta) or not pmt.is_c32vector(vec):
            return
        if base._pmt_str(meta, "status", "other") != "ok":
            return
        raw = pmt.c32vector_elements(vec)
        if not raw:
            return
        info = self.controller.on_frame(
            raw, base._pmt_int(meta, "zero_delay_tap", 0))
        if info is None or not self.controller.enabled:
            return
        self.echo.set_cal_delay_native(
            float(self.controller.cal_delay_native))


# --------------------------------------------------------------------------
# Parser
# --------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(
        parents=[base.build_parser(add_help=False)],
        description="Streaming (gr-radar-style) X410 CG400 HRP echo CIR radar")
    p.add_argument("--clock-source", default="internal")
    p.add_argument("--time-source", default="internal")
    p.add_argument("--tx-repeat", dest="tx_repeat", action="store_true",
                   default=True,
                   help="repeat the TX waveform so the echo sees one packet "
                        "per PRI (default)")
    p.add_argument("--no-tx-repeat", dest="tx_repeat", action="store_false",
                   help="send a single TX burst then stop")
    p.add_argument("--freq-mode", choices=["fixed", "scan", "manual"],
                   default="fixed",
                   help="fixed = nominal only; scan = step sweep; manual = "
                        "stdin commands applied at burst boundaries")
    p.add_argument("--freq-start", type=float, default=None,
                   help="scan start in Hz (default: --freq)")
    p.add_argument("--freq-stop", type=float, default=None,
                   help="scan stop in Hz (single sweep, inclusive)")
    p.add_argument("--freq-step", type=float, default=0.0,
                   help="scan step magnitude in Hz (>0); direction follows "
                        "--freq-start/--freq-stop")
    p.add_argument("--freq-dwell", type=int, default=20,
                   help="stream pulses per scan frequency (>=1)")
    p.add_argument("--freq-scan", choices=["once", "cycle"], default="once",
                   help="scan once then stop, or cycle until pulses")
    p.add_argument("--freq-settle-s", type=float, default=0.02,
                   help="sleep after set_freq before watching the next burst")
    p.add_argument("--freq-unit", choices=list(fp.FREQ_UNIT_CHOICES),
                   default=fp.DEFAULT_FREQ_UNIT,
                   help="default unit for bare manual-mode numbers")
    p.add_argument("--peak-target-tap", type=int, default=0,
                   help="lock the first CIR peak to this tap via a calibration "
                        "servo (0 = disabled, keep --cal-delay-native)")
    p.add_argument("--peak-first-rel", type=float, default=0.5,
                   help="first peak = first tap >= this fraction of max(|CIR|)")
    p.add_argument("--peak-search-start", type=int, default=0,
                   help="ignore CIR taps below this index")
    p.add_argument("--peak-search-stop", type=int, default=None,
                   help="ignore CIR taps above this index")
    p.add_argument("--peak-cal-skip", type=int, default=5,
                   help="ignore the first N bursts before calibrating")
    p.add_argument("--peak-cal-pulses", type=int, default=5,
                   help="calibrate on the first N bursts after the skip")
    p.add_argument("--peak-lock-frames", type=int, default=2,
                   help="lock early after this many in-deadband bursts")
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan and exit without touching UHD")
    return p.parse_args()


def build_freq_plan(a):
    manual_q = None
    if a.freq_mode == "manual":
        import queue as _queue
        manual_q = _queue.Queue(maxsize=256)
        fp.start_manual_reader(manual_q)
    return fp.FreqPlan(
        a.freq_mode, a.freq, start_hz=a.freq_start, stop_hz=a.freq_stop,
        step_hz=a.freq_step, dwell=a.freq_dwell,
        once=(a.freq_scan == "once"), manual_q=manual_q,
        freq_unit=a.freq_unit)


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main():
    base.bootstrap_uhd_env()
    a = parse_args()

    # preamble length / legacy alias resolution (same as base/sweep)
    if a.preamble_length is not None and a.sync_reps is not None \
            and a.preamble_length != a.sync_reps:
        raise SystemExit("--preamble-length=%d conflicts with --sync-reps=%d"
                         % (a.preamble_length, a.sync_reps))
    if a.preamble_length is not None:
        a.sync_reps = a.preamble_length
    elif a.sync_reps is None:
        a.sync_reps = base.DEFAULT_PREAMBLE_LENGTH
    insert_sts = False if a.no_sts else True
    if a.rate_hz and a.rate_hz > 0:
        a.pri_s = 1.0 / a.rate_hz
        a.pulses = int(round(a.rate_hz * a.duration_s))

    plan = build_freq_plan(a)
    if a.freq_mode == "scan":
        total = plan.total_pulses()
        if total is not None:
            if a.pulses != total:
                print("[freq] scan once: overriding pulses %d -> %d "
                      "(n_freq=%d dwell=%d)"
                      % (a.pulses, total, plan.n_points, plan.dwell),
                      flush=True)
            a.pulses = total
    target_frames = 0 if (a.freq_mode == "manual" and a.pulses <= 0) \
        else int(a.pulses)
    # The echo runs `target_frames + slack` bursts (not exactly target): when
    # the echo returns WORK_DONE the GNU Radio stream subgraph finishes and
    # the executor tears message delivery down before the *last* CIR PDU is
    # delivered (one frame lost).  A little slack keeps the stream alive long
    # enough for every target frame to be written; the app stops at target and
    # truncates the analysis to it.
    max_frames = (target_frames + 8) if target_frames > 0 else 0
    if target_frames == 0:
        print("[freq] manual: run until 'q' / stdin EOF", flush=True)

    print("[freq] mode=%s nominal=%.6fMHz points=%d dwell=%d target_frames=%d"
          % (a.freq_mode, a.freq / 1e6, plan.n_points, plan.dwell,
             target_frames),
          flush=True)
    if a.freq_mode == "scan":
        print("[freq] sweep %.3f -> %.3f MHz step %+.3f MHz"
              % (plan.freqs[0] / 1e6, plan.freqs[-1] / 1e6,
                 plan.step_hz / 1e6), flush=True)

    if a.peak_target_tap > 0:
        print("[align] first peak -> tap %d (base cal_delay_native=%.3f, "
              "first_rel=%.2f, search=%d..%s, cal_skip=%d, cal_pulses=%d, "
              "lock_frames=%d)"
              % (a.peak_target_tap, a.cal_delay_native, a.peak_first_rel,
                 a.peak_search_start,
                 "-" if a.peak_search_stop is None else a.peak_search_stop,
                 a.peak_cal_skip, a.peak_cal_pulses, a.peak_lock_frames),
              flush=True)

    repo = base.find_repo_root()
    os.makedirs(a.output, exist_ok=True)
    taps = a.taps or os.path.join(
        repo, "testdata", "resampler_65_32", "taps_quality_minorder.txt")
    tmpl_path = a.template or os.path.join(a.output, "sync_template_live.cf32")
    timing_path = os.path.join(a.output, "echo_timing.jsonl")

    # --- TX native waveform (once) ---------------------------------------
    psdu = base.hex_to_bytes(a.psdu_hex)
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
    src = base.uwb.hrp_packet_source(
        psdu, a.sync_reps, base.SFD_MODE, a.code_index, 0.8, a.pri_s, False,
        insert_sts, False, pulse_shape, a.pulse_sigma_ns, a.pulse_bw_mhz,
        pulse_taps)
    samples = np.array(src.samples(), dtype=np.complex64)
    if samples.size < base.SPS:
        raise SystemExit("HRP source produced %d samples" % samples.size)
    _tmpl_off = max(0, int(src.pulse_center_taps()) - 4)
    samples[_tmpl_off:_tmpl_off + base.SPS].tofile(tmpl_path)
    t_rs = time.perf_counter()
    native = base.resample_poly(samples.astype(np.complex128),
                                32, 65).astype(np.complex64)
    tx_samples = int(native.size)
    print("hrp_tx_998p4_samples=%d native_491p52=%d resample_ms=%.2f "
          "insert_sts=%s pulse_shape=%s code_index=%d preamble_length=%d "
          "sfd_mode=%s"
          % (samples.size, tx_samples, (time.perf_counter() - t_rs) * 1e3,
             insert_sts, src.pulse_shape(), a.code_index, a.sync_reps,
             base.SFD_MODE), flush=True)

    # --- RX window geometry (7th return value) ---------------------------
    pre, sync_n, sfd_n, rng_n, tail, pad_n, rx_samples = base.rx_geometry(
        base.CG400_HZ, a.pre_guard_us, a.sync_reps, 15.0, a.tail_guard_us,
        tx_samples, a.rx_pad_us)
    capture = rx_samples - pre - tail
    if capture < 0:
        capture = rx_samples
    bplan = buffer_plan(tx_samples, rx_samples)
    bapplied = None
    print("rx_geometry pre=%d capture=%d post=%d rx_samples=%d "
          "rx_window_us=%.3f" % (
              pre, capture, tail, rx_samples,
              rx_samples / base.CG400_HZ * 1e6), flush=True)
    print("buffer_plan tx_samples=%d rx_samples=%d plan=%s"
          % (tx_samples, rx_samples, json.dumps(bplan, sort_keys=True)),
          flush=True)

    align = fp.PeakAlignController(
        a.cal_delay_native, a.peak_target_tap,
        work_per_native=(65.0 / 32.0),
        first_peak_rel=a.peak_first_rel,
        search_start=a.peak_search_start,
        search_stop=a.peak_search_stop,
        cal_skip=a.peak_cal_skip,
        cal_pulses=a.peak_cal_pulses,
        lock_frames=a.peak_lock_frames)

    if a.dry_run:
        print("[dry-run] tx_samples=%d rx_samples=%d capture=%d pre=%d "
              "post=%d buffer_plan=%s"
              % (tx_samples, rx_samples, capture, pre, tail,
                 json.dumps(bplan, sort_keys=True)), flush=True)
        raise SystemExit(0)

    # --- blocks ----------------------------------------------------------
    if not hasattr(base.uwb, "echo_timer_stream_uhd"):
        raise SystemExit(
            "uwb.echo_timer_stream_uhd is not available in the loaded "
            "bindings; rebuild gr-uwb (M2 streaming echo block) first")
    estimate_use_pred = not a.require_sfd
    est_q = max(8, int(a.est_queue))

    tx_src = blocks.vector_source_c(native.tolist(), bool(a.tx_repeat), 1, [])
    tx_ts = blocks.stream_to_tagged_stream(
        gr.sizeof_gr_complex, 1, tx_samples, "packet_len")
    echo = base.uwb.echo_timer_stream_uhd(
        a.args, base.CG400_HZ, a.tx_channel, a.rx_channel,
        a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx, a.freq,
        a.clock_source, a.time_source,
        int(base.llround(a.pri_s * base.CG400_HZ)), 1,
        pre, 0, a.arm_delay_s,
        tx_samples, rx_samples, pre, capture, tail,
        a.sync_reps, base.SFD_MODE, a.code_index, a.cal_delay_native,
        max_frames, 1000, 1 << 21, 1 << 21, "packet_len")
    res = base.uwb.rational_resampler_ccf_65_32(
        taps, True, 1, "packet_len")
    bridge = blocks.tagged_stream_to_pdu(gr.types.complex_t, "packet_len")
    est = base.uwb.radar_cir_estimator(
        tmpl_path, a.sync_reps, base.SFD_MODE, a.code_index, 16, 100, 10, 0,
        a.sfd_search_margin, a.sync_refine_margin, a.sfd_threshold,
        a.sync_refine_threshold, True, est_q, estimate_use_pred)
    wr = base.uwb.cir_writer(a.output, "cir", True, 64)

    run_stop = threading.Event()
    retuner = StreamFreqRetuner(echo, plan, a.freq, run_stop,
                                settle_s=a.freq_settle_s)
    freq_lookup = make_freq_lookup(a, plan, retuner)

    udp = None
    if (not a.no_udp) and bool(a.udp_host):
        udp = base.CirUdpSink(a.udp_host, int(a.udp_port),
                              base.CIR_UDP_TAPS, freq_lookup=freq_lookup)
        print("udp_cir %s:%s framed=UCR2(+freq_hz,freq_offset_hz) "
              "always_send_taps=%d nonblock"
              % (a.udp_host, a.udp_port, base.CIR_UDP_TAPS), flush=True)

    align_sink = PeakAlignSink(align, echo) if align.enabled else None

    recorder = None
    if a.dump_sc16:
        tx_sc16_path = os.path.join(a.output, "tx_491p52.sc16")
        base.fc32_to_sc16(native).tofile(tx_sc16_path)
        print("wrote_tx_sc16 %s samples=%d" % (tx_sc16_path, tx_samples),
              flush=True)
        recorder = StreamSc16Recorder(
            os.path.join(a.output, "capture.iq"), rx_samples, pre, capture,
            tail, base.CG400_HZ, freq_lookup=freq_lookup)
        print("dump_sc16 capture.iq (Python tee; short runs only)", flush=True)

    # --- flowgraph -------------------------------------------------------
    tb = gr.top_block("x410_cg400_hrp_echo_cir_stream")
    tb.connect((tx_src, 0), (tx_ts, 0))
    tb.connect((tx_ts, 0), (echo, 0))
    tb.connect((echo, 0), (res, 0))
    tb.connect((res, 0), (bridge, 0))
    if recorder is not None:
        tb.connect((echo, 0), (recorder, 0))
    tb.msg_connect((bridge, "pdus"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if udp is not None:
        tb.msg_connect((est, "cir"), (udp, "cir"))
    if align_sink is not None:
        tb.msg_connect((est, "cir"), (align_sink, "cir"))

    bapplied = apply_buffer_plan(tx_ts, echo, res, bplan)
    print("buffer_applied %s" % json.dumps(bapplied, sort_keys=True),
          flush=True)

    # --- run -------------------------------------------------------------
    tb.start()
    retuner.start()
    live_stop = threading.Event()
    live_th = start_stream_live_stats(echo, res, est, wr, udp, live_stop,
                                      a.pri_s)
    t_run = time.perf_counter()
    while not run_stop.wait(0.05):
        done = _counter(echo, "bursts_ok") + _counter(echo, "bursts_failed")
        if target_frames > 0 and done >= target_frames:
            break
        if retuner.stop_requested:
            run_stop.set()
            break
    sched_s = time.perf_counter() - t_run
    print("stream_wall_s=%.3f" % sched_s, flush=True)
    live_stop.set()
    live_th.join(timeout=1.5)

    # Drain: let the resampler / bridge / estimator / writer finish.  The
    # estimator queue may still hold work after the echo stops, so wait for
    # drained() plus a short settle window (bounded by `deadline`).
    deadline = time.time() + 12.0
    drained_since = None
    while time.time() < deadline:
        done = _counter(echo, "frames")
        written = wr.frames_written() + wr.frames_failed()
        est_pub = est.pdus_completed() + est.pdus_failed()
        want = target_frames if target_frames > 0 else est_pub
        rec_ok = (recorder is None) or (recorder.windows >= done)
        if est.drained() and written >= want and rec_ok:
            if drained_since is None:
                drained_since = time.time()
            elif time.time() - drained_since > 0.1:
                break
        else:
            drained_since = None
        time.sleep(0.05)

    run_stop.set()
    retuner.stop()
    tb.stop()
    tb.wait()
    if recorder is not None:
        recorder.stop()
    try:
        wr.stop()
    except Exception:
        pass

    if a.timing_detail:
        print_stream_counters(echo, res, est, wr, udp)

    # --- output ----------------------------------------------------------
    frames_done = int(_counter(echo, "frames"))
    jsonl = os.path.join(a.output, "cir.jsonl")
    # The echo runs `target + slack` bursts to keep the stream alive through
    # the drain; drop the few slack frames so the output is exactly target.
    if target_frames > 0 and frames_done > target_frames:
        try:
            with open(jsonl, "r", encoding="utf-8") as f:
                lines = f.read().splitlines()
            with open(jsonl, "w", encoding="utf-8") as f:
                f.write("\n".join(lines[:target_frames]) + "\n")
            for name in ("cir.cf32", "cir_norm.cf32"):
                p = os.path.join(a.output, name)
                if os.path.isfile(p):
                    with open(p, "r+b") as f:
                        f.truncate(target_frames * base.CIR_UDP_TAPS * 8)
            frames_done = target_frames
            print("[stream] truncated %d slack CIR frames -> %d"
                  % (len(lines) - target_frames, frames_done), flush=True)
        except OSError as exc:
            print("[stream] slack truncate skipped: %s" % exc, flush=True)
    cir_stats = base.analyze_cir(jsonl, frames_done)
    timing_stats = base.analyze_timing(timing_path)
    freq_records = build_freq_records(a, plan, retuner, frames_done)
    freq_stats = fp.analyze_cir_by_freq(jsonl, freq_records)

    freq_sweep_path = os.path.join(a.output, "freq_sweep.jsonl")
    with open(freq_sweep_path, "w", encoding="utf-8") as f:
        for rec in freq_records:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    echo_ok = _counter(echo, "bursts_ok")
    echo_fail = _counter(echo, "bursts_failed")
    echo_late = _counter(echo, "late_slot_skips")
    summary = {
        "app": "x410_cg400_hrp_echo_cir_stream",
        "chain": "stream",
        "hrp_samples": int(samples.size),
        "tx_samples": tx_samples,
        "rx_samples": rx_samples,
        "rx_window_us": rx_samples / base.CG400_HZ * 1e6,
        "pre_guard_samples": pre,
        "capture_samples": capture,
        "post_guard_samples": tail,
        "pulses": a.pulses,
        "target_frames": target_frames,
        "echo_max_frames": max_frames,
        "pulses_done": frames_done,
        "pri_s": a.pri_s,
        "pri_num": int(base.llround(a.pri_s * base.CG400_HZ)),
        "pri_den": 1,
        "rate_hz": 1.0 / a.pri_s,
        "schedule_wall_s": sched_s,
        "buffer_plan": bplan,
        "buffer_applied": bapplied,
        "echo_bursts_ok": echo_ok,
        "echo_bursts_failed": echo_fail,
        "echo_late_slot_skips": echo_late,
        "echo_frames": _counter(echo, "frames"),
        "echo_tx_samples": _counter(echo, "tx_samples"),
        "echo_rx_samples": _counter(echo, "rx_samples"),
        "res_windows": _counter(res, "windows"),
        "res_input_items": _counter(res, "input_items"),
        "res_output_items": _counter(res, "output_items"),
        "res_tag_errors": _counter(res, "tag_errors"),
        "est_rx": _counter(est, "pdus_received"),
        "est_enq": _counter(est, "pdus_enqueued"),
        "est_done": _counter(est, "pdus_completed"),
        "est_fail": _counter(est, "pdus_failed"),
        "est_drop": _counter(est, "pdus_dropped"),
        "est_invalid": _counter(est, "invalid_inputs"),
        "est_queue_capacity": est_q,
        "est_queue_hwm": _counter(est, "queue_high_watermark"),
        "est_service_us_mean": _counter(est, "service_mean_us"),
        "est_service_us_max": _counter(est, "service_max_us"),
        "wr_ok": _counter(wr, "frames_written"),
        "wr_fail": _counter(wr, "frames_failed"),
        "wr_invalid": _counter(wr, "frames_invalid"),
        "use_predicted_timing": estimate_use_pred,
        "udp_sent": 0 if udp is None else udp.sent,
        "udp_sent_ok": 0 if udp is None else udp.sent_ok,
        "udp_sent_fail": 0 if udp is None else udp.sent_fail,
        "udp_sent_freq": 0 if udp is None else udp.sent_freq,
        "udp_eagain": 0 if udp is None else udp.dropped,
        "cir": cir_stats,
        "timing": timing_stats,
        "output": a.output,
        "sc16_packets": 0 if recorder is None else recorder.windows,
        "sc16_samples": 0 if recorder is None else recorder._offset,
        "dump_sc16": bool(a.dump_sc16),
        "freq_hz": a.freq,
        "freq_mode": a.freq_mode,
        "freq_nominal_hz": a.freq,
        "freq_start_hz": plan.start,
        "freq_stop_hz": plan.stop,
        "freq_step_hz": plan.step_hz,
        "freq_dwell": a.freq_dwell,
        "freq_scan": a.freq_scan,
        "freq_settle_s": a.freq_settle_s,
        "freq_unit": a.freq_unit,
        "freq_points": plan.n_points,
        "freq_plan_hz": plan.freqs,
        "freq_retune_count": retuner.retune_count,
        "freq_retune_fail": retuner.retune_fail,
        "peak_target_tap": a.peak_target_tap,
        "peak_first_rel": a.peak_first_rel,
        "peak_search_start": a.peak_search_start,
        "peak_search_stop": a.peak_search_stop,
        "peak_cal_skip": a.peak_cal_skip,
        "peak_cal_pulses": a.peak_cal_pulses,
        "peak_lock_frames": a.peak_lock_frames,
        "align_enabled": align.enabled,
        "align_frames": align.frames,
        "align_skipped": align.skipped,
        "align_adapt_frames": align.adapt_frames,
        "align_applied": align.applied,
        "align_locked": align.locked,
        "align_last_first_peak_tap": align.last_first_peak,
        "align_last_peak_tap": align.last_peak_tap,
        "align_last_error": align.last_error,
        "align_cal_delay_native": align.cal_delay_native,
        "align_cal_delay_base_native": align.base_cal_native,
        "cir_by_freq": freq_stats,
        "native_rate_hz": base.CG400_HZ,
        "work_rate_hz": base.WORK_HZ,
        "code_index": a.code_index,
        "preamble_length": a.sync_reps,
        "sfd_mode": base.SFD_MODE,
        "gain_tx": a.gain_tx,
        "gain_rx": a.gain_rx,
        "tx_channel": a.tx_channel,
        "rx_channel": a.rx_channel,
        "tx_antenna": a.tx_antenna,
        "rx_antenna": a.rx_antenna,
        "clock_source": a.clock_source,
        "time_source": a.time_source,
        "iq_scale": base.IQ_SCALE,
        "sample_format": "sc16",
        "rx_pad_us": a.rx_pad_us,
    }
    with open(os.path.join(a.output, "summary.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")

    print("SUMMARY", json.dumps(summary), flush=True)
    print("radio_ok=%d radio_fail=%d late=%d frames=%d res_windows=%d "
          "cir_ok=%d cir_fail=%d est_drop=%d udp_sent=%d udp_ok=%d "
          "retune=%d retune_fail=%d buffer_plan=%s "
          "(radio ok is not CIR/UDP ok)" % (
              echo_ok, echo_fail, echo_late, summary["echo_frames"],
              summary["res_windows"], cir_stats.get("ok", 0),
              cir_stats.get("fail", 0), summary["est_drop"],
              summary["udp_sent"], summary["udp_sent_ok"],
              retuner.retune_count, retuner.retune_fail,
              json.dumps(bplan, sort_keys=True)),
          flush=True)

    expected = frames_done
    if expected <= 0:
        ok = False
    elif a.dump_sc16:
        ok = (echo_ok + echo_fail == expected and echo_late == 0
              and summary["sc16_packets"] == expected)
    else:
        ok = (summary["wr_ok"] == expected and echo_ok == expected
              and echo_late == 0
              and cir_stats.get("ok") == expected
              and cir_stats.get("missing_count", 1) == 0
              and retuner.retune_fail == 0)
    raise SystemExit(0 if ok else 3)


if __name__ == "__main__":
    main()
