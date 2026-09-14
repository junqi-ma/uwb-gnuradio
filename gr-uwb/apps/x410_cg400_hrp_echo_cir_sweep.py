#!/usr/bin/env python3
"""X410 CG600 / CG400 HRP echo CIR with runtime frequency tuning.

Built on the validated chain in ``x410_cg400_hrp_echo_cir.py`` (imported as
``base``).  Only the RF centre frequency is made dynamic; the TX waveform,
the PDU resampler and the 998.4 MS/s CIR estimator are unchanged.  Native
rate and pulse shape default to the CG600 chain (``--native-rate 737.28e6``,
``--pulse-shape legacy``); both are inherited from the base parser.

Echo backends (``--echo-backend``, inherited from the base parser)
-----------------------------------------------------------------
cpp-pdu ``SweepCppPduEcho``  -- **default**.  The C++ UwbRealtimeEchoTimer
        message-only
        grid.  Retune is queued with ``blk.set_freq`` (the radio worker tunes
        at a burst boundary); calibration is pushed with
        ``blk.set_cal_delay_native``.  Python never calls a UHD tune on this
        path.  The grid is armed per constant-frequency dwell so
        ``freq_hz``/``freq_offset_hz`` stay keyed by ``pulse_id`` for the same
        JSONL join as the Python path.
python  ``SweepTimedUhdEcho`` -- legacy per-pulse Python PMT publisher
        (fallback; retuning/servo can starve the timed TX and emit 'U').

Modes (``--freq-mode``)
-----------------------
fixed   Identical to the base app: drive the nominal ``--freq`` only.
scan    Start at ``--freq-start`` (default ``--freq``), step by
        ``--freq-step`` up to ``--freq-stop``, ``--freq-dwell`` pulses per
        point.  Default is one sweep then stop (``--freq-scan cycle`` repeats).
manual  Read commands from stdin at runtime (``+5MHz``, ``6489.6MHz``,
        ``status``, ``q``); the latest command takes effect at the next burst
        boundary.

TX and RX are shifted by the SAME delta.  The monostatic echo therefore stays
at zero relative CFO while the external DW3000 sits at ``CFO = delta`` -- the
quantity under study.

Per-pulse frequency is recorded in ``echo_timing.jsonl`` and
``freq_sweep.jsonl``; ``summary.json`` carries a per-frequency CIR table joined
to ``cir.jsonl`` by ``pulse_id`` (no C++ changes needed).
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
from gnuradio import gr                         # noqa: E402


class SweepTimedUhdEcho(base.TimedUhdEcho):
    """TimedUhdEcho that can retune between bursts and log per-pulse freq."""

    def __init__(self, *args, plan=None, align=None, freq_settle_s=0.05,
                 **kwargs):
        super().__init__(*args, **kwargs)
        self.plan = plan
        self.align = align
        self.nominal = float(self.freq)
        self.freq_settle_s = float(freq_settle_s)
        self.tx_freq_actual = float(self.freq)
        self.rx_freq_actual = float(self.freq) + self.rx_freq_offset
        self.freq_records = []
        self.freq_by_pulse = {}
        self.retune_count = 0
        self.retune_fail = 0

    def retune(self, freq_hz, next_pulse_id):
        """Shift TX+RX to freq_hz and re-arm the schedule at next_pulse_id."""
        freq_hz = float(freq_hz)
        if abs(freq_hz - self.freq) < 1.0:
            return False
        uhd = self._uhd
        self._usrp.set_tx_freq(uhd.types.TuneRequest(freq_hz), self.tx_ch)
        self._usrp.set_rx_freq(
            uhd.types.TuneRequest(freq_hz + self.rx_freq_offset), self.rx_ch)
        self.tx_freq_actual = float(self._usrp.get_tx_freq(self.tx_ch))
        self.rx_freq_actual = float(self._usrp.get_rx_freq(self.rx_ch))
        self.freq = 0.5 * (self.tx_freq_actual + self.rx_freq_actual
                           - self.rx_freq_offset)
        # Re-anchor the absolute schedule so the next burst arms at
        # now + settle, independent of the global pulse id.
        now = self._usrp.get_time_now().get_real_secs()
        self._t0 = now + self.freq_settle_s - next_pulse_id * self.pri_s
        self.retune_count += 1
        return True

    def _one_burst(self, pulse_id):
        target = (self.plan.freq_for(pulse_id)
                  if self.plan is not None else self.freq)
        if target is not None and abs(float(target) - self.freq) >= 1.0:
            try:
                self.retune(target, pulse_id)
            except Exception as exc:
                self.retune_fail += 1
                print("[freq] retune to %.3f MHz failed, keep %.3f MHz: %s"
                      % (float(target) / 1e6, self.freq / 1e6, exc), flush=True)
        # Publish this pulse's frequency before the CIR can be produced so the
        # UDP sink can attach it to the matching UCR2 frame.
        self.freq_by_pulse[int(pulse_id)] = (
            self.freq, self.freq - self.nominal)
        ok = super()._one_burst(pulse_id)
        offset = self.freq - self.nominal
        dwell_index = (self.plan.dwell_index(pulse_id)
                       if self.plan is not None else 0)
        align_first = self.align.last_first_peak if self.align is not None else None
        align_err = self.align.last_error if self.align is not None else None
        align_locked = self.align.locked if self.align is not None else None
        align_adapt = self.align.adapt_frames if self.align is not None else None
        align_skipped = self.align.skipped if self.align is not None else None
        # base._one_burst always appends its timing record last.
        if self._timing:
            rec = self._timing[-1]
            rec["freq_hz"] = self.freq
            rec["freq_offset_hz"] = offset
            rec["tx_freq_actual"] = self.tx_freq_actual
            rec["rx_freq_actual"] = self.rx_freq_actual
            rec["dwell_index"] = dwell_index
            rec["cal_delay_native"] = self.cal_delay_native
            rec["align_first_peak_tap"] = align_first
            rec["align_error"] = align_err
            rec["align_locked"] = align_locked
            rec["align_adapt_frames"] = align_adapt
            rec["align_skipped"] = align_skipped
        rec = {
            "pulse_id": int(pulse_id),
            "freq_hz": self.freq,
            "freq_offset_hz": offset,
            "tx_freq_actual": self.tx_freq_actual,
            "rx_freq_actual": self.rx_freq_actual,
            "dwell_index": dwell_index,
            "cal_delay_native": self.cal_delay_native,
            "align_first_peak_tap": align_first,
            "align_error": align_err,
            "align_locked": align_locked,
            "align_adapt_frames": align_adapt,
            "align_skipped": align_skipped,
        }
        self.freq_records.append(rec)
        return ok

    def _write_sc16_packet(self, pulse_id, rx):
        # Same as base, plus the per-pulse frequency fields.
        if self._sc16_iq is None or self._sc16_jsonl is None:
            return
        sc16 = base.fc32_to_sc16(rx)
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
            "iq_scale": base.IQ_SCALE,
            "window_start_sample": int(self._sc16_offset),
            "predicted_start_sample": int(predicted),
            "pre_guard_samples": int(self.pre),
            "capture_samples": int(self.rx_len - self.pre - self.tail),
            "post_guard_samples": int(self.tail),
            "schedule_index": int(pulse_id),
            "capture_mode": "x410_echo",
            "lock_state": "timed",
            "freq_hz": self.freq,
            "freq_offset_hz": self.freq - self.nominal,
            "tx_freq_actual": self.tx_freq_actual,
            "rx_freq_actual": self.rx_freq_actual,
        }
        self._sc16_jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._sc16_jsonl.flush()
        self._sc16_offset += n
        self._sc16_written += 1

    def run_schedule(self):
        if self._native is None:
            raise RuntimeError("TX waveform not set")
        self.open_sc16_dump()
        self.start_publisher()
        t_host0 = time.perf_counter()
        try:
            for pulse_id in range(self.max_pulses):
                if self.plan is not None and self.plan.stop_requested:
                    print("[freq] stop requested at pulse %d" % pulse_id,
                          flush=True)
                    break
                self._one_burst(pulse_id)
                self._done = pulse_id + 1
                if (pulse_id == 0 or (pulse_id + 1) % 100 == 0
                        or pulse_id + 1 == self.max_pulses):
                    dt = time.perf_counter() - t_host0
                    print("sched %d/%d ok=%d fail=%d late=%d sc16=%d "
                          "host_s=%.3f freq=%.3fMHz off=%+.3fMHz retune=%d"
                          % (pulse_id + 1, self.max_pulses, self._ok,
                             self._fail, self._late, self._sc16_written, dt,
                             self.freq / 1e6,
                             (self.freq - self.nominal) / 1e6,
                             self.retune_count), flush=True)
        finally:
            self.close_sc16_dump()
        return self._ok, self._fail, self._late


class SweepCppPduEcho(base.CppPduEcho):
    """CppPduEcho + the sweep app's retune / per-pulse frequency log.

    The C++ ``UwbRealtimeEchoTimer`` worker owns all UHD I/O, so a retune is
    a pending request (``blk.set_freq``) applied by the worker at the next
    burst boundary -- Python never calls a UHD tune on this path.  One grid
    is armed per constant-frequency dwell (a single PDU for
    ``--freq-mode fixed``, one PDU per dwell for ``scan``/``manual``); each
    PDU keeps the base app's ``pulse_id_increment``/``burst_count``/
    ``publish_native``/full-radar-geometry contract.  ``freq_hz`` and
    ``freq_offset_hz`` are recorded per ``pulse_id`` app-side with the same
    keys and the same ``analyze_cir_by_freq`` join as the Python path.

    Servo / settling policy is unchanged: ``PeakAlignController`` still owns
    skip/cal/lock, and ``PeakAlignSink`` pushes the updated calibration with
    ``blk.set_cal_delay_native`` (the worker reads it when it publishes the
    next burst), mirroring how the Python chain carried ``cal_delay_native``
    in the next pulse's metadata.

    Known limitation: the C++ grid consumes a ``pulse_id`` for every skipped
    (expired) slot, and ``set_freq`` is asynchronous, so a retune cannot be
    confirmed synchronously.  The re-anchor margin below (``max(arm_delay_s,
    freq_settle_s)``) keeps the first slot of every dwell comfortably in the
    future so skips do not occur in normal operation.
    """

    def __init__(self, a, native, rate, plan=None, align=None,
                 freq_settle_s=0.05):
        super().__init__(a, native, rate)
        self.plan = plan
        self.align = align
        self.nominal = float(self.freq)
        self.freq_settle_s = float(freq_settle_s)
        self.tx_freq_actual = float(self.freq)
        self.rx_freq_actual = float(self.freq) + self.rx_freq_offset
        self.freq_records = []
        self.freq_by_pulse = {}
        self.retune_count = 0
        self.retune_fail = 0
        self._done = 0

    def retune(self, freq_hz, next_pulse_id):
        """Queue a TX+RX retune on the C++ worker (burst-boundary tune)."""
        freq_hz = float(freq_hz)
        if abs(freq_hz - self.freq) < 1.0:
            return False
        self.blk.set_freq(freq_hz)
        self.freq = freq_hz
        # Async: the worker applies the pending tune at the next burst
        # boundary, so a synchronous get_tx_freq()/get_rx_freq() is not
        # available here.  Record the requested centre (what the UDP/JSONL
        # join needs); blk.freq() holds the last actually-applied value.
        self.tx_freq_actual = freq_hz
        self.rx_freq_actual = freq_hz + self.rx_freq_offset
        self.retune_count += 1
        return True

    def _segment_len(self, pulse_id):
        """Pulses that may share one grid at the current target frequency."""
        if self.plan is None or self.plan.mode == "fixed":
            return max(0, self.max_pulses - pulse_id)
        if self.plan.mode == "manual":
            return 1  # a command can change the target on any pulse
        f0 = float(self.plan.freq_for(pulse_id))
        n = 1
        while n < self.plan.dwell and pulse_id + n < self.max_pulses:
            if abs(float(self.plan.freq_for(pulse_id + n)) - f0) >= 1.0:
                break
            n += 1
        return max(1, n)

    def _record_pulse(self, pulse_id):
        """Append the per-pulse frequency record (Python-path keys)."""
        offset = self.freq - self.nominal
        dwell_index = (self.plan.dwell_index(pulse_id)
                       if self.plan is not None else 0)
        align_first = self.align.last_first_peak if self.align is not None else None
        align_err = self.align.last_error if self.align is not None else None
        align_locked = self.align.locked if self.align is not None else None
        align_adapt = self.align.adapt_frames if self.align is not None else None
        align_skipped = self.align.skipped if self.align is not None else None
        rec = {
            "pulse_id": int(pulse_id),
            "freq_hz": self.freq,
            "freq_offset_hz": offset,
            "tx_freq_actual": self.tx_freq_actual,
            "rx_freq_actual": self.rx_freq_actual,
            "dwell_index": dwell_index,
            "cal_delay_native": self.cal_delay_native,
            "align_first_peak_tap": align_first,
            "align_error": align_err,
            "align_locked": align_locked,
            "align_adapt_frames": align_adapt,
            "align_skipped": align_skipped,
        }
        self.freq_records.append(rec)
        self.freq_by_pulse[int(pulse_id)] = (self.freq, offset)

    def _schedule_meta(self, first_pulse_id, burst_count):
        """base._schedule_meta() with this dwell's schedule keys override."""
        src = super()._schedule_meta()
        meta = pmt.make_dict()
        items = pmt.dict_items(src)
        for i in range(pmt.length(items)):
            kv = pmt.nth(i, items)
            k = pmt.car(kv)
            ks = pmt.symbol_to_string(k) if pmt.is_symbol(k) else None
            if ks in ("pulse_id", "schedule_index", "burst_count",
                      "pulse_id_increment"):
                continue
            meta = pmt.dict_add(meta, k, pmt.cdr(kv))
        meta = pmt.dict_add(meta, pmt.intern("schedule_index"),
                            pmt.from_uint64(0))
        meta = pmt.dict_add(meta, pmt.intern("pulse_id"),
                            pmt.from_uint64(first_pulse_id))
        meta = pmt.dict_add(meta, pmt.intern("pulse_id_increment"),
                            pmt.from_uint64(1))
        meta = pmt.dict_add(meta, pmt.intern("burst_count"),
                            pmt.from_uint64(burst_count))
        # App-recorded frequency is also visible on the burst PDU (the C++
        # whitelist copies these keys) for downstream inspection.
        meta = pmt.dict_add(meta, pmt.intern("freq_hz"),
                            pmt.from_double(self.freq))
        meta = pmt.dict_add(meta, pmt.intern("freq_offset_hz"),
                            pmt.from_double(self.freq - self.nominal))
        return meta

    def _post_segment(self, first_pulse_id, burst_count):
        # Re-anchor at now + max(arm_delay, freq_settle): the worker applies
        # the pending tune before the first slot, and the margin is never
        # shorter than the base app's arm delay.  This is the cpp-pdu
        # counterpart of the Python chain's "t0 = now + settle" re-anchor.
        margin_s = max(self.arm_delay_s, self.freq_settle_s)
        t0 = self.blk.device_time_ticks() + int(round(margin_s * self.rate))
        meta = self._schedule_meta(first_pulse_id, burst_count)
        meta = pmt.dict_add(meta, pmt.intern("t0_ticks"), pmt.from_long(t0))
        payload = base.fc32_to_sc16(self._native)
        sched = pmt.cons(
            meta, pmt.init_s16vector(int(payload.size), payload.tolist()))
        self.blk._post(pmt.intern("schedule"), sched)
        return t0

    def _wait_published(self, target, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.blk.bursts_published() >= target:
                return True
            time.sleep(0.005)
        return False

    def run_schedule(self):
        if self._native is None:
            raise RuntimeError("TX waveform not set")
        t_host0 = time.perf_counter()
        pulse_id = 0
        while pulse_id < self.max_pulses:
            if self.plan is not None and self.plan.stop_requested:
                print("[freq] stop requested at pulse %d" % pulse_id,
                      flush=True)
                break
            target = (self.plan.freq_for(pulse_id)
                      if self.plan is not None else self.freq)
            if target is not None and abs(float(target) - self.freq) >= 1.0:
                try:
                    self.retune(target, pulse_id)
                except Exception as exc:
                    self.retune_fail += 1
                    print("[freq] retune to %.3f MHz failed, keep %.3f MHz: %s"
                          % (float(target) / 1e6, self.freq / 1e6, exc),
                          flush=True)
            count = min(self._segment_len(pulse_id),
                        self.max_pulses - pulse_id)
            for k in range(count):
                self._record_pulse(pulse_id + k)
            before = int(self.blk.bursts_published())
            t0 = self._post_segment(pulse_id, count)
            seg_ok = self._wait_published(before + count,
                                          count * self.pri_s + 15.0)
            pulse_id += count
            self._done = pulse_id
            dt = time.perf_counter() - t_host0
            print("sched %d/%d ok=%d fail=%d late=%d sc16=%d host_s=%.3f "
                  "freq=%.3fMHz off=%+.3fMHz retune=%d t0_ticks=%d "
                  "bursts=%d" % (
                      pulse_id, self.max_pulses, self._ok, self._fail,
                      self._late, self._sc16_written, dt, self.freq / 1e6,
                      (self.freq - self.nominal) / 1e6, self.retune_count,
                      t0, count), flush=True)
            if not seg_ok:
                print("WARN cpp_pdu segment timeout at pulse %d "
                      "(published=%d last_error=%s)"
                      % (pulse_id, self.blk.bursts_published(),
                         self.blk.last_error()), flush=True)
                break
        if self.blk.bursts_published() < self.max_pulses:
            print("WARN cpp_pdu sweep published=%d/%d last_error=%s"
                  % (self.blk.bursts_published(), self.max_pulses,
                     self.blk.last_error()), flush=True)
        return (int(self.blk.bursts_ok()), int(self.blk.bursts_failed()),
                int(self.blk.late_slot_skips()))


class PeakAlignSink(gr.basic_block):
    """Closed loop: read CIR taps, steer the calibration delay to the target tap.

    Subscribes to ``est.cir`` (in addition to the writer / UDP).  For the
    Python backend it updates ``echo.cal_delay_native`` so the *next* pulse's
    metadata carries the new calibration; for cpp-pdu it pushes the same value
    with ``backend.set_cal_delay_native`` (the C++ worker applies it when it
    publishes the next burst).  Message-only, no streaming ports.
    """

    def __init__(self, controller, echo, backend=None):
        gr.basic_block.__init__(self, name="peak_align_sink",
                                in_sig=None, out_sig=None)
        self.controller = controller
        self.echo = echo
        # cpp-pdu: the C++ radio worker owns UHD, so calibration must be
        # pushed with set_cal_delay_native (applied at the next published
        # burst).  None for the Python backend, whose next pulse metadata
        # carries echo.cal_delay_native.
        self.backend = backend
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
        self.echo.cal_delay_native = self.controller.cal_delay_native
        if self.backend is not None:
            self.backend.set_cal_delay_native(
                self.controller.cal_delay_native)


def parse_args():
    # --echo-backend/--res-workers/--publish-native come from the base parser
    # (parents=...), so they are not re-declared here (that would conflict).
    p = argparse.ArgumentParser(
        parents=[base.build_parser(add_help=False)],
        description="X410 CG600/CG400 HRP echo CIR with runtime frequency "
                    "tuning (defaults inherited from the base app: "
                    "--native-rate 737.28e6, --pulse-shape legacy)")
    p.add_argument("--freq-mode", choices=["fixed", "scan", "manual"],
                   default="fixed",
                   help="fixed = base app; scan = step sweep; manual = stdin")
    p.add_argument("--freq-start", type=float, default=None,
                   help="scan start in Hz (default: --freq)")
    p.add_argument("--freq-stop", type=float, default=None,
                   help="scan stop in Hz (single sweep, inclusive)")
    p.add_argument("--freq-step", type=float, default=0.0,
                   help="scan step magnitude in Hz (>0); direction follows "
                        "--freq-start/--freq-stop, so stop < start sweeps down")
    p.add_argument("--freq-dwell", type=int, default=20,
                   help="pulses per scan frequency (>=1)")
    p.add_argument("--freq-scan", choices=["once", "cycle"], default="once",
                   help="scan once then stop, or cycle until pulses")
    p.add_argument("--freq-settle-s", type=float, default=0.05,
                   help="delay after each retune before the next timed burst")
    p.add_argument("--freq-unit", choices=list(fp.FREQ_UNIT_CHOICES),
                   default=fp.DEFAULT_FREQ_UNIT,
                   help="default unit for bare manual-mode numbers "
                        "(default %s); explicit units/scientific notation win"
                        % fp.DEFAULT_FREQ_UNIT)
    p.add_argument("--peak-target-tap", type=int, default=0,
                   help="lock the first CIR peak to this tap via a calibration "
                        "servo (0 = disabled, keep the fixed --cal-delay-native)")
    p.add_argument("--peak-first-rel", type=float, default=0.5,
                   help="first peak = first tap >= this fraction of max(|CIR|)")
    p.add_argument("--peak-search-start", type=int, default=0,
                   help="ignore CIR taps below this index when finding the first peak")
    p.add_argument("--peak-search-stop", type=int, default=None,
                   help="ignore CIR taps above this index (default: last tap); "
                        "gate the search so out-of-window interference is ignored")
    p.add_argument("--peak-cal-skip", type=int, default=5,
                   help="ignore the first N bursts before calibrating; covers "
                        "the several-tap settle transient after a retune "
                        "(set 0 to calibrate from the very first burst)")
    p.add_argument("--peak-cal-pulses", type=int, default=5,
                   help="calibrate only on the first N bursts after the skip, "
                        "then freeze the axis (0 = never lock, old servo)")
    p.add_argument("--peak-lock-frames", type=int, default=2,
                   help="lock early once this many consecutive bursts land "
                        "within deadband (0 = only the --peak-cal-pulses cap)")
    p.add_argument("--dry-run", action="store_true",
                   help="print the frequency plan and exit without touching UHD")
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


def main():
    base.bootstrap_uhd_env()
    a = parse_args()
    base.resolve_echo_backend(a)
    base.resolve_dpdk_args(a)
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

    # Profile must be resolved before the [align] banner and the peak servo,
    # both of which read a.cal_delay_native.
    profile = base.NativeRateProfile(a.native_rate)
    if a.cal_delay_native is None:
        a.cal_delay_native = base.default_cal_delay_native(profile.hz)
    print("native_rate=%.1f MS/s (%s) tx=%d/%d pdu=%s work_per_native=%.6f "
          "cal_delay_native=%.1f" % (
              profile.hz / 1e6, profile.name, profile.tx_interp,
              profile.tx_decim, profile.pdu, profile.work_per_native,
              a.cal_delay_native), flush=True)

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
    elif a.freq_mode == "manual" and a.pulses <= 0:
        a.pulses = 1 << 31
        print("[freq] manual: run until 'q'", flush=True)
    if a.freq_mode == "manual":
        print("[freq] manual: bare numbers are %s (e.g. +50 = +50 %s); "
              "explicit units (5MHz) and scientific notation (10e6) win; "
              "'q'+Enter stops" % (a.freq_unit, a.freq_unit), flush=True)

    print("[freq] mode=%s nominal=%.6fMHz points=%d dwell=%d total_pulses=%d"
          % (a.freq_mode, a.freq / 1e6, plan.n_points, plan.dwell, a.pulses),
          flush=True)
    if a.freq_mode == "scan":
        print("[freq] sweep %.3f -> %.3f MHz step %+.3f MHz"
              % (plan.freqs[0] / 1e6, plan.freqs[-1] / 1e6,
                 plan.step_hz / 1e6), flush=True)
        for i, f in enumerate(plan.freqs):
            print("[freq]   #%03d %.6f MHz (offset %+.3f MHz)"
                  % (i, f / 1e6, (f - a.freq) / 1e6), flush=True)

    if a.peak_target_tap > 0:
        stop_s = "-" if a.peak_search_stop is None else str(a.peak_search_stop)
        print("[align] first peak -> tap %d (base cal_delay_native=%.3f, "
              "first_rel=%.2f, search=%d..%s, cal_skip=%d, cal_pulses=%d, "
              "lock_frames=%d)"
              % (a.peak_target_tap, a.cal_delay_native, a.peak_first_rel,
                 a.peak_search_start, stop_s, a.peak_cal_skip,
                 a.peak_cal_pulses, a.peak_lock_frames), flush=True)

    if a.dry_run:
        print("[freq] dry-run complete", flush=True)
        raise SystemExit(0)

    base.dpdk_preflight(a)

    repo = base.find_repo_root()
    os.makedirs(a.output, exist_ok=True)
    taps = a.taps or os.path.join(
        repo, "testdata", profile.taps_dir, "taps_quality_minorder.txt")
    tmpl_path = a.template or os.path.join(a.output, "sync_template_live.cf32")
    timing_path = os.path.join(a.output, "echo_timing.jsonl")

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
    native = profile.tx_native(samples)
    print("hrp_tx_998p4_samples=%d native_%s=%d resample_ms=%.2f "
          "insert_sts=%s pulse_shape=%s pulse_taps=%d pulse_center=%d "
          "code_index=%d preamble_length=%d sfd_mode=%s"
          % (samples.size, profile.label, native.size,
             (time.perf_counter() - t_rs) * 1e3,
             insert_sts, src.pulse_shape(), src.pulse_taps(),
             src.pulse_center_taps(), a.code_index, a.sync_reps,
             base.SFD_MODE),
          flush=True)
    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f" % (
        a.pulses, a.pri_s, (1.0 / a.pri_s), a.pulses * a.pri_s), flush=True)

    if a.echo_backend == "cpp-pdu" and (a.dump_rx or a.dump_sc16):
        print("WARN --dump-rx/--dump-sc16 are unsupported with "
              "--echo-backend cpp-pdu; disabling dumps", flush=True)
        a.dump_rx = False
        a.dump_sc16 = False
    dump_dir = os.path.join(a.output, "rx_iq") if a.dump_rx else ""
    sc16_dir = a.output if a.dump_sc16 else ""
    # PeakAlignController owns the skip/cal/lock policy for BOTH backends
    # (identical to the old Python chain); only the cal delivery path differs
    # (next-pulse metadata vs blk.set_cal_delay_native).
    align = fp.PeakAlignController(
        a.cal_delay_native, a.peak_target_tap,
        work_per_native=profile.work_per_native,
        first_peak_rel=a.peak_first_rel,
        search_start=a.peak_search_start,
        search_stop=a.peak_search_stop,
        cal_skip=a.peak_cal_skip,
        cal_pulses=a.peak_cal_pulses,
        lock_frames=a.peak_lock_frames)
    if a.echo_backend == "cpp-pdu":
        echo = SweepCppPduEcho(
            a, native, profile, plan=plan, align=align,
            freq_settle_s=a.freq_settle_s)
        echo_out_port = "burst"
        echo_block = echo.blk
    else:
        echo = SweepTimedUhdEcho(
            a.args, profile, a.freq, a.tx_channel, a.rx_channel,
            a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx,
            a.pre_guard_us, 15.0, a.tail_guard_us, a.sync_reps,
            a.cal_delay_native, a.arm_delay_s, a.pri_s, a.pulses, dump_dir,
            a.min_lead_s, timing_path, sc16_dir, a.rx_pad_us, a.code_index,
            base.SFD_MODE, publish_native=a.publish_native, plan=plan,
            align=align, freq_settle_s=a.freq_settle_s,
            rx_freq_offset=a.rx_freq_offset)
        echo_out_port = "rx"
        echo_block = echo
    print("echo_backend=%s" % a.echo_backend, flush=True)
    echo.set_tx_native(native)
    print("uhd_probe", echo.status, flush=True)
    if a.dump_sc16:
        tx_sc16_path = os.path.join(a.output, "tx_%s.sc16" % profile.label)
        base.fc32_to_sc16(echo._native).tofile(tx_sc16_path)
        print("wrote_tx_sc16", tx_sc16_path, "samples=%d" % echo._native.size,
              flush=True)

    if a.res_sc16_scale == "unit":
        sc16_scale = base.uwb.Sc16ScalePolicy.UnitRange
    elif a.res_sc16_scale == "raw":
        sc16_scale = base.uwb.Sc16ScalePolicy.RawInteger
    else:  # auto: cpp-pdu publishes raw UHD SC16 -> UnitRange matches FC32
        sc16_scale = (base.uwb.Sc16ScalePolicy.UnitRange
                      if a.echo_backend == "cpp-pdu"
                      else base.uwb.Sc16ScalePolicy.RawInteger)
    res = profile.make_pdu_resampler(taps, a.res_workers, sc16_scale)
    print("resampler pdu=%s sc16_scale=%s" % (profile.pdu, a.res_sc16_scale),
          flush=True)
    est_q = max(8, int(a.est_queue))
    use_pred = not a.require_sfd
    est = base.uwb.radar_cir_estimator(
        tmpl_path, a.sync_reps, base.SFD_MODE, a.code_index, 16, 100, 10, 0,
        a.sfd_search_margin, a.sync_refine_margin, a.sfd_threshold,
        a.sync_refine_threshold, True, est_q, use_pred)
    print("estimator code_index=%d preamble_length=%d sfd_search_margin=%d "
          "queue=%d use_predicted_timing=%s (overflow=drop)" % (
              a.code_index, a.sync_reps, a.sfd_search_margin, est_q, use_pred),
          flush=True)
    wr = base.uwb.cir_writer(a.output, "cir", True, 64)
    udp = None
    udp_on = (not a.no_udp) and bool(a.udp_host)
    if udp_on:
        udp = base.CirUdpSink(
            a.udp_host, int(a.udp_port), base.CIR_UDP_TAPS,
            freq_lookup=lambda pid: echo.freq_by_pulse.get(int(pid)))
        print("udp_cir %s:%s framed=UCR2(+freq_hz,freq_offset_hz) "
              "always_send_taps=%d nonblock" % (
                  a.udp_host, a.udp_port, base.CIR_UDP_TAPS), flush=True)

    align_backend = echo.blk if a.echo_backend == "cpp-pdu" else None
    align_sink = (PeakAlignSink(align, echo, align_backend)
                  if align.enabled else None)
    tb = base.gr.top_block("x410_cg400_hrp_echo_cir_sweep")
    tb.msg_connect((echo_block, echo_out_port), (res, "packet"))
    tb.msg_connect((res, "packet"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if udp is not None:
        tb.msg_connect((est, "cir"), (udp, "cir"))
    if align_sink is not None:
        tb.msg_connect((est, "cir"), (align_sink, "cir"))

    tb.start()
    echo.start_publisher()
    live_stop = threading.Event()
    live_th = base.start_live_stats(echo, est, wr, udp, live_stop, a.pri_s)
    t_run = time.perf_counter()
    echo.run_schedule()
    sched_s = time.perf_counter() - t_run
    print("schedule_wall_s=%.3f" % sched_s, flush=True)
    live_stop.set()
    live_th.join(timeout=1.5)

    expected = int(echo._done)
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
        base.print_timing_detail(echo, est)

    jsonl = os.path.join(a.output, "cir.jsonl")
    cir_stats = base.analyze_cir(jsonl, expected)
    timing_stats = base.analyze_timing(timing_path)
    freq_stats = fp.analyze_cir_by_freq(jsonl, echo.freq_records)

    freq_sweep_path = os.path.join(a.output, "freq_sweep.jsonl")
    with open(freq_sweep_path, "w", encoding="utf-8") as f:
        for rec in echo.freq_records:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    summary = {
        "hrp_samples": int(samples.size),
        "native_samples": int(native.size),
        "pulses": a.pulses,
        "pulses_done": expected,
        "pri_s": a.pri_s,
        "rate_hz": 1.0 / a.pri_s,
        "schedule_wall_s": sched_s,
        "echo_ok": echo._ok,
        "echo_fail": echo._fail,
        "echo_late": echo._late,
        "echo_pub": echo._pub_ok,
        "tx_send_error": echo._tx_send_error,
        "publish_native": echo.publish_native,
        "echo_backend": a.echo_backend,
        "res_workers": int(a.res_workers),
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
        "echo_queue_hwm": (int(echo.blk.queue_high_watermark())
                           if a.echo_backend == "cpp-pdu" else 0),
        "echo_worker_us_mean": (int(echo.blk.mean_worker_us())
                                if a.echo_backend == "cpp-pdu" else 0),
        "echo_worker_us_max": (int(echo.blk.max_worker_us())
                               if a.echo_backend == "cpp-pdu" else 0),
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
        "freq_retune_count": echo.retune_count,
        "freq_retune_fail": echo.retune_fail,
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
        "native_rate_hz": profile.hz,
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
        "iq_scale": base.IQ_SCALE,
        "sample_format": "sc16",
        "rx_window": echo.rx_len,
        "rx_window_us": echo.rx_len / profile.hz * 1e6,
        "rx_pad_us": a.rx_pad_us,
    }
    with open(os.path.join(a.output, "summary.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    if a.dump_sc16:
        meta_path = os.path.join(a.output, "metadata.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump({
                "description": (
                    "X410 CG600/CG400 monostatic HRP echo, native SC16 RX "
                    "windows, runtime frequency sweep"
                ),
                "sample_format": "sc16",
                "dtype": "int16",
                "byte_order": "little-endian",
                "layout": "interleaved_iq",
                "bytes_per_complex": 4,
                "iq_scale": base.IQ_SCALE,
                "sample_index_base": 0,
                "freq_hz": a.freq,
                "freq_mode": a.freq_mode,
                "freq_plan_hz": plan.freqs,
                "freq_step_hz": a.freq_step,
                "freq_dwell": a.freq_dwell,
                "rate_native_hz": profile.hz,
                "rate_work_hz": base.WORK_HZ,
                "code_index": a.code_index,
                "sync_repetitions": a.sync_reps,
                "sfd_mode": base.SFD_MODE,
                "insert_sts": insert_sts,
                "packets": expected,
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
                    "capture.jsonl": "one JSON object per packet (has freq)",
                    "freq_sweep.jsonl": "one JSON object per packet (freq)",
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
          "udp_ok=%d udp_fail=%d retune=%d retune_fail=%d "
          "(radio ok is not CIR/UDP ok)" % (
              summary["echo_ok"], cir_stats.get("ok", 0),
              cir_stats.get("fail", 0), summary["est_drop"],
              summary["udp_sent"], summary["udp_sent_ok"],
              summary["udp_sent_fail"], echo.retune_count,
              echo.retune_fail),
          flush=True)
    if align.enabled:
        print("[align] locked=%s skipped=%d adapt_frames=%d applied=%d "
              "cal_delay_native=%.3f last_first_peak=%s last_error=%+.1f "
              "search=%d..%s" % (
                  align.locked, align.skipped, align.adapt_frames,
                  align.applied, align.cal_delay_native,
                  align.last_first_peak, align.last_error, align.search_start,
                  "-" if align.search_stop is None else align.search_stop),
              flush=True)
    if freq_stats:
        print("CIR_BY_FREQ freq_MHz n ok fail metric_mean metric_max",
              flush=True)
        for g in freq_stats:
            print("  %12.6f %5d %5d %5d %s %s" % (
                (g["freq_hz"] or 0.0) / 1e6, g["n"], g["ok"], g["fail"],
                ("%.5f" % g["metric_mean"]) if g["metric_mean"] is not None
                else "-",
                ("%.5f" % g["metric_max"]) if g["metric_max"] else "-",
            ), flush=True)
    if align.enabled:
        print("[align] target=%d frames=%d applied=%d last_first_peak=%s "
              "last_error=%s cal_delay_native=%.3f"
              % (align.target_tap, align.frames, align.applied,
                 align.last_first_peak, align.last_error,
                 align.cal_delay_native), flush=True)
    if os.path.isfile(jsonl):
        print("cir.jsonl_lines=%d" % cir_stats.get("lines", 0), flush=True)

    if a.dump_sc16:
        ok = (summary["echo_ok"] == expected and summary["echo_late"] == 0
              and summary["sc16_packets"] == expected)
    else:
        ok = (summary["wr_ok"] == expected and summary["echo_ok"] == expected
              and summary["echo_late"] == 0
              and cir_stats.get("ok") == expected
              and cir_stats.get("missing_count", 1) == 0
              and echo.retune_fail == 0)
    raise SystemExit(0 if ok else 3)


if __name__ == "__main__":
    main()
