#!/usr/bin/env python3
"""X410 dual-TX HRP jammer for UWB sensing CIR / CFO studies.

Two TX channels of ONE X410 transmit simultaneously from a single two-row
host buffer, so the jammer is sample-aligned with the sensing burst and shares
the device clock:

  sense  TX ch0  code 9  @ f0            (unchanged sensing chain)
  jammer TX ch1  code 10 @ f0 + df       (this app)
  RX        ch3  @ f0   -> SC16 -> 65/48 PDU -> radar_cir_estimator

Because TX and RX share the reference, the interferer sits at baseband ``df``
in the sensing receiver, and ``df`` is set by the jammer channel's NCO (exact,
drift-free).  This is the controlled counterpart to sweeping the external
commercial device (see ``x410_cg400_hrp_echo_cir_sweep.py``): here the CFO is
known and the jammer preamble repeats on exactly the sensing 1016-sample
lattice, so the CIR repetition-average null at ``f_work/(2*SPS) = 491.34 kHz``
can be measured without the commercial device's clock/CFO uncertainty.

Backend selection (``--echo-backend``):

* ``align`` + ``cpp-pdu`` (M3): sense/jammer are normalised and converted to
  SC16 once at startup and submitted as a single multi-TX schedule PDU
  ``cons(meta, pmt_vector[ch0_s16, ch1_s16])`` to the C++
  ``UwbRealtimeEchoTimer`` grid.  No per-pulse Python timed loop, no
  ``tolist()`` composite rebuild.  RX reuses the standard
  ``burst -> 65/48 PDU -> radar_cir_estimator -> cir_writer/UDP`` chain.
* ``python`` (default, kept until C++ hardware acceptance): the legacy
  ``JamTimedUhdEcho`` per-pulse timed loop with the ``(2, L)`` composite
  payload; also the only backend with ``--dump-sc16`` and with
  ``--jam-mode continuous`` (the independent jammer streamer is out of
  scope for the C++ path, §11).

Planning/geometry math lives in ``echo_cir_jam_plan`` (pure Python, unit
tested); this file only wires it to UHD and the existing CIR chain.

Example::

  python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py \\
    --args addr=192.168.10.2 --jam-enable --jam-channel 1 \\
    --preamble-length 128 --gain-tx 50 --gain-rx 60 --no-udp \\
    --jam-code-index 10 --jam-preamble-length 128 --jam-scale 0.3 \\
    --jam-freq-offsets 0,245.67e3,491.34e3 --jam-dwell 50 \\
    --jam-delay-random-us 1.018 \\
    --rate-hz 200 --output /tmp/x410_jam
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
import echo_cir_jam_plan as jp                  # noqa: E402

import pmt                                      # noqa: E402
from gnuradio import gr                         # noqa: E402


# The jammer defaults to the same PSDU bytes as the sensing packet, so it is
# a real IEEE 802.15.4a HRP packet (code 10) rather than a bare tone.


def parse_freq_offsets(text):
    """Parse ``--jam-freq-offsets`` ("0,245.67e3,491.34e3") into a list of Hz."""
    out = []
    for tok in (text or "").split(","):
        tok = tok.strip()
        if tok:
            out.append(float(tok))
    return out


def _resolve_shape(shape, taps, repo):
    """Resolve a (--pulse-shape, --pulse-taps) pair to (shape, taps_path)."""
    if taps:
        return "external", taps
    if shape == "linear":
        return "external", os.path.join(
            repo, "testdata", "uwb_hrp_tx", "pulse_trunc_linear_rc183_240.f32")
    if shape == "minphase":
        return "external", os.path.join(
            repo, "testdata", "uwb_hrp_tx", "pulse_minphase_rc160_240.f32")
    return shape, ""


class JamTimedUhdEcho(base.TimedUhdEcho):
    """``TimedUhdEcho`` with a second TX channel carrying the HRP jammer.

    The base class opens the TX streamer on ``tx_channels`` and sends
    ``self._tx_payload``; this subclass installs a ``(2, L)`` payload whose
    row 0 is the (normalised) sensing burst and row 1 the scaled jammer at
    ``jam_delay_native``, then retunes the jammer channel between dwells.
    """

    def __init__(self, args, profile, freq, tx_ch, rx_ch, tx_ant, rx_ant,
                 gain_tx, gain_rx, pre_us, range_m, tail_us, sync_reps,
                 cal_delay_native, arm_delay_s, pri_s, max_pulses,
                 rx_dump_dir="", min_lead_s=0.002, timing_path="",
                 sc16_dump_dir="", rx_pad_us=8.0,
                 code_index=base.DEFAULT_CODE_INDEX, sfd_mode=base.SFD_MODE,
                 publish_native=0, rx_freq_offset=0.0,
                 jam_enable=True, jam_ch=jp.DEFAULT_JAM_CHANNEL,
                 jam_antenna="TX/RX0", jam_gain_tx=0.0, jam_freq_offset=0.0,
                 jam_scale=0.3, jam_delay_us=0.0, jam_offsets=None,
                 jam_dwell=0, freq_settle_s=0.05,
                 jam_delay_random_us=None, jam_delay_seed=None):
        tx_channels = [int(tx_ch), int(jam_ch)] if jam_enable else [int(tx_ch)]
        super().__init__(
            args, profile, freq, tx_ch, rx_ch, tx_ant, rx_ant,
            gain_tx, gain_rx, pre_us, range_m, tail_us, sync_reps,
            cal_delay_native, arm_delay_s, pri_s, max_pulses,
            rx_dump_dir, min_lead_s, timing_path, sc16_dump_dir, rx_pad_us,
            code_index, sfd_mode, publish_native=publish_native,
            rx_freq_offset=rx_freq_offset, tx_channels=tx_channels)

        self.jam_enable = bool(jam_enable)
        self.jam_ch = int(jam_ch)
        self.jam_antenna = jam_antenna
        self.jam_gain_tx = float(jam_gain_tx)
        self.jam_scale = float(jam_scale)
        self.jam_delay_us = float(jam_delay_us)
        self.jam_delay_random_us = (
            None if jam_delay_random_us in (None, "")
            else float(jam_delay_random_us))
        self.tx_lead_native = 0
        if self.jam_delay_random_us is not None:
            if jam_delay_seed is None:
                self.jam_delay_seed = int(
                    np.random.default_rng().integers(0, 2**31 - 1))
            else:
                self.jam_delay_seed = int(jam_delay_seed)
            self._delay_rng = np.random.default_rng(self.jam_delay_seed)
        else:
            self.jam_delay_seed = None
            self._delay_rng = None
        self.jam_delay_native_lo = 0
        self.jam_delay_native_hi = 0
        self._jam_scaled = None
        self.jam_offsets = list(jam_offsets) if jam_offsets else []
        self.jam_dwell = max(0, int(jam_dwell))
        self.freq_settle_s = float(freq_settle_s)
        self.jam_freq_offset = float(jam_freq_offset)
        self.jam_freq_hz = float(freq) + self.jam_freq_offset
        self.jam_freq_actual = self.jam_freq_hz
        self.jam_delay_native = 0
        self.jam_native = None
        self.jam_payload_native = 0
        self.jam_retune_count = 0
        self.jam_retune_fail = 0
        self.jam_by_pulse = {}
        self.jam_records = []

        if self.jam_enable:
            # The sense channel was tuned by super().__init__(); the RX LO
            # stays at --freq so the jammer lands at baseband +offset.
            self._usrp.set_tx_freq(
                self._uhd.types.TuneRequest(self.jam_freq_hz), self.jam_ch)
            self._usrp.set_tx_gain(self.jam_gain_tx, self.jam_ch)
            self._usrp.set_tx_antenna(jam_antenna, self.jam_ch)
            self.jam_freq_actual = float(self._usrp.get_tx_freq(self.jam_ch))

        self.status["jam_enabled"] = self.jam_enable
        self.status["jam_channel"] = self.jam_ch
        self.status["jam_antenna"] = (
            self._usrp.get_tx_antenna(self.jam_ch) if self.jam_enable
            else jam_antenna)
        self.status["jam_gain_tx"] = self.jam_gain_tx
        self.status["jam_freq_hz"] = self.jam_freq_hz
        self.status["jam_freq_offset_hz"] = self.jam_freq_offset
        self.status["jam_scale"] = self.jam_scale
        self.status["jam_delay_random_us"] = self.jam_delay_random_us
        self.status["jam_delay_seed"] = (
            self.jam_delay_seed if self.jam_delay_random_us is not None
            else None)

    # -- jammer payload -----------------------------------------------------

    def prepare_jam(self, sense_native, jam_native):
        """Install the two-row TX payload.

        ``sense_native`` is the raw sensing native waveform (the base class
        normalises it to peak 0.8); ``jam_native`` is the raw jammer native
        waveform, scaled so its peak is ``jam_scale * peak(sense)``.
        """
        jam_native = (None if jam_native is None
                      else np.asarray(jam_native, dtype=np.complex64))
        if jam_native is not None and jam_native.size == 0:
            jam_native = None
        self.jam_native = jam_native
        jam_len = 0 if jam_native is None else int(jam_native.size)
        self.jam_payload_native = jam_len
        sense_len = int(np.asarray(sense_native).size)
        if self.jam_delay_random_us is not None:
            half = jp.delay_half_span_native(
                self.jam_delay_random_us, self.rate)
            self.tx_lead_native = half
            self.jam_delay_native_lo = -half
            self.jam_delay_native_hi = half
            self.jam_delay_native = 0
            self.jam_delay_us = 0.0
            length = jp.bipolar_tx_len(sense_len, jam_len, half)
        else:
            self.tx_lead_native = 0
            self.jam_delay_native = jp.placement_native(
                self.jam_delay_us, self.rate)
            self.jam_delay_native_lo = self.jam_delay_native
            self.jam_delay_native_hi = self.jam_delay_native
            length = jp.combined_tx_len(
                sense_len, jam_len, self.jam_delay_native)

        # Geometry first (the RX window must cover the composite), then the
        # base normalisation, then the composite payload on top of it.
        self.tx_geometry_native = int(length)
        self.set_tx_native(sense_native)
        sense_peak = float(np.max(np.abs(self._native))) or 1.0
        if jam_native is None:
            jam_scaled = None
        else:
            jam_peak = float(np.max(np.abs(jam_native))) or 1.0
            jam_scaled = (jam_native / jam_peak
                          * (self.jam_scale * sense_peak)).astype(np.complex64)
        self._jam_scaled = jam_scaled
        if self.tx_lead_native:
            # Sense is parked at index D so a negative jam delay still fits.
            # pre_guard grows by D so CIR origin stays on the sensing start.
            self._tx_payload = jp.compose_tx_native_at(
                self._native, jam_scaled, self.tx_lead_native,
                self.tx_lead_native, length)
            self.pre += int(self.tx_lead_native)
            self.rx_len = (int(self.rx_len) + int(self.tx_lead_native) + 3) // 4 * 4
            if int(self.publish_native) > 0:
                self.publish_native = int(self.publish_native) + int(
                    self.tx_lead_native)
            self.status["rx_window"] = self.rx_len
            self.status["publish_native"] = self.publish_native
        else:
            self._tx_payload = jp.compose_tx_native(
                self._native, jam_scaled, self.jam_delay_native)

        self.status["tx_geometry_native"] = int(length)
        self.status["tx_lead_native"] = int(self.tx_lead_native)
        self.status["jam_delay_us"] = self.jam_delay_us
        self.status["jam_delay_native"] = self.jam_delay_native
        self.status["jam_delay_native_lo"] = self.jam_delay_native_lo
        self.status["jam_delay_native_hi"] = self.jam_delay_native_hi
        self.status["jam_native"] = jam_len
        self.status["jam_tx_samples"] = int(self._tx_payload.shape[1])
        self.status["jam_peak_scale"] = self.jam_scale
        return self._tx_payload

    def _randomize_jam_delay(self):
        """Draw a new relative jammer delay and rewrite TX row 1."""
        if self._delay_rng is None or self._tx_payload is None:
            return
        delay = jp.draw_delay_native(
            self._delay_rng, self.jam_delay_native_lo,
            self.jam_delay_native_hi)
        jp.place_jam_row(
            self._tx_payload, self._jam_scaled,
            int(self.tx_lead_native) + int(delay))
        self.jam_delay_native = int(delay)
        self.jam_delay_us = delay / self.rate * 1e6

    # -- jammer dwell retune ------------------------------------------------

    def _jam_target_offset(self, pulse_id):
        if not self.jam_offsets or self.jam_dwell <= 0:
            return None
        return float(self.jam_offsets[(pulse_id // self.jam_dwell)
                                      % len(self.jam_offsets)])

    def _retune_jam(self, offset_hz, pulse_id):
        """Shift only the jammer TX channel; re-anchor the timed grid."""
        try:
            self._usrp.set_tx_freq(
                self._uhd.types.TuneRequest(self.freq + float(offset_hz)),
                self.jam_ch)
        except Exception as exc:  # pragma: no cover - device path
            self.jam_retune_fail += 1
            print("[jam] retune to %+.3f kHz failed, keep %+.3f kHz: %s"
                  % (offset_hz / 1e3, self.jam_freq_offset / 1e3, exc),
                  flush=True)
            return False
        self.jam_freq_actual = float(self._usrp.get_tx_freq(self.jam_ch))
        self.jam_freq_offset = float(offset_hz)
        self.jam_freq_hz = self.freq + self.jam_freq_offset
        now = self._usrp.get_time_now().get_real_secs()
        self._t0 = now + self.freq_settle_s - pulse_id * self.pri_s
        self.jam_retune_count += 1
        return True

    def _record_jam(self, pulse_id):
        rec = {
            "pulse_id": int(pulse_id),
            "jam_freq_hz": self.jam_freq_hz,
            "jam_freq_offset_hz": self.jam_freq_offset,
            "jam_freq_actual_hz": self.jam_freq_actual,
            "jam_freq_actual_offset_hz": self.jam_freq_actual - self.freq,
            "jam_delay_us": self.jam_delay_us,
            "jam_delay_native": self.jam_delay_native,
            "jam_scale": self.jam_scale,
            "jam_retune": self.jam_retune_count,
        }
        self.jam_records.append(rec)
        self.jam_by_pulse[int(pulse_id)] = (self.jam_freq_hz,
                                            self.jam_freq_offset)
        return rec

    def run_schedule(self):
        if self._native is None:
            raise RuntimeError("TX waveform not set")
        self.open_sc16_dump()
        self.start_publisher()
        t_host0 = time.perf_counter()
        try:
            scan = bool(self.jam_offsets and self.jam_dwell > 0)
            pulse_id = 0
            scan_step = 0
            ok_in_step = 0
            retune_settle_s = (max(0, len(self.jam_offsets) - 1) *
                               self.freq_settle_s) if scan else 0.0
            deadline = (time.monotonic() + self.max_pulses * self.pri_s +
                        retune_settle_s + 15.0)
            while (scan_step < len(self.jam_offsets) if scan
                   else pulse_id < self.max_pulses):
                if scan and time.monotonic() >= deadline:
                    raise RuntimeError(
                        "jam scan incomplete: step=%d/%d dwell=%d/%d "
                        "attempts=%d ok=%d fail=%d late=%d" %
                        (scan_step, len(self.jam_offsets), ok_in_step,
                         self.jam_dwell, pulse_id, self._ok, self._fail,
                         self._late))

                if scan:
                    target = float(self.jam_offsets[scan_step])
                    if abs(target - self.jam_freq_offset) >= 1.0:
                        # A failed retune must not transmit another sample at
                        # the already-complete old frequency.
                        if not self._retune_jam(target, pulse_id):
                            time.sleep(min(max(self.pri_s, 0.001), 0.05))
                            continue

                self._randomize_jam_delay()
                captured = bool(self._one_burst(pulse_id))
                if self.jam_enable:
                    self._record_jam(pulse_id)
                    self._prepare_timing_record(pulse_id)
                pulse_id += 1
                self._done = pulse_id

                if scan and captured:
                    ok_in_step += 1
                    if ok_in_step == self.jam_dwell:
                        scan_step += 1
                        ok_in_step = 0

                if (pulse_id == 1 or pulse_id % 100 == 0 or
                        (scan and scan_step == len(self.jam_offsets)) or
                        (not scan and pulse_id == self.max_pulses)):
                    dt = time.perf_counter() - t_host0
                    print("sched attempts=%d target_ok=%d ok=%d fail=%d "
                          "late=%d sc16=%d host_s=%.3f "
                          "jam_step=%d/%d dwell_ok=%d/%d "
                          "jam_off=%+.6fMHz jam_dly=%.3fus retune=%d"
                          % (pulse_id, self.max_pulses, self._ok,
                             self._fail, self._late, self._sc16_written, dt,
                             scan_step, len(self.jam_offsets), ok_in_step,
                             self.jam_dwell, self.jam_freq_offset / 1e6,
                             self.jam_delay_us, self.jam_retune_count),
                          flush=True)
        finally:
            self.close_sc16_dump()
        return self._ok, self._fail, self._late

    def _prepare_timing_record(self, pulse_id):
        """Annotate the burst's timing record with the jammer frequency."""
        if not self._timing:
            return
        rec = self._timing[-1]
        rec["jam_freq_hz"] = self.jam_freq_hz
        rec["jam_freq_offset_hz"] = self.jam_freq_offset
        rec["jam_delay_us"] = self.jam_delay_us
        rec["jam_delay_native"] = self.jam_delay_native
        rec["jam_retune"] = self.jam_retune_count


class _JamFreqPlan:
    """Static jammer scan plan for summaries and legacy label fallback.

    The C++ worker counts successful RX captures, so failures and late-slot
    gaps make ``pulse_id // dwell`` unsuitable for actual burst labels.
    Runtime consumers therefore use the propagated ``jam_freq_*`` metadata;
    this arithmetic lookup is retained only for sources lacking that metadata.
    """

    def __init__(self, freq, off0, offsets, dwell):
        self.freq = float(freq)
        self.off0 = float(off0)
        self.offsets = [float(o) for o in (offsets or [])]
        self.dwell = max(0, int(dwell))

    def offset_for(self, pulse_id):
        if self.offsets and self.dwell > 0:
            return float(self.offsets[(int(pulse_id) // self.dwell)
                                      % len(self.offsets)])
        return self.off0

    def lookup(self, pulse_id):
        off = self.offset_for(pulse_id)
        return (self.freq + off, off)

    def dwell_plan(self):
        """Compact per-dwell scan plan for the summary (bounded)."""
        if not self.offsets or self.dwell <= 0:
            return [{"dwell_index": 0, "pulse_start": 0,
                     "pulse_count": None,
                     "jam_freq_hz": self.freq + self.off0,
                     "jam_freq_offset_hz": self.off0}]
        return [{"dwell_index": i, "pulse_start": i * self.dwell,
                 "pulse_count": self.dwell,
                 "jam_freq_hz": self.freq + float(o),
                 "jam_freq_offset_hz": float(o)}
                for i, o in enumerate(self.offsets)]


def build_cpp_schedule_pdu(cpp_meta, radar_meta, ch0_s16, ch1_s16,
                           base_freq_hz=0.0):
    """Assemble one multi-TX schedule PDU: cons(meta, pmt_vector[ch0, ch1]).

    ``cpp_meta`` is the plain dict from ``jp.build_cpp_schedule_meta``
    (§5.3 keys); ``radar_meta`` carries the standard schedule keys with
    the same semantics as ``base.CppPduEcho._schedule_meta``
    (``tx_samples``/``rx_samples``/``publish_native``/geometry, ``t0_ticks``
    added by the caller at run time).  Waveforms are int16 SC16 arrays
    (``2 * samples`` elements each).  Called once at startup, never
    per-pulse, so the one-time ``tolist()`` here is not on the hot path.

    ``base_freq_hz`` is the ABSOLUTE centre frequency the scan offsets are
    relative to.  It MUST be provided whenever the jam plan has a nonzero
    offset: the C++ worker adds it to each dwell offset before tuning, and
    with an unknown (0) base a scan would tune the jammer out of band
    (measured 2026-09-19: requested offset-only 491339 Hz -> device coerced
    to 1 MHz).  Harmonic/relative offsets alone are never an absolute tune.
    """
    meta = pmt.make_dict()
    if base_freq_hz > 0.0:
        meta = pmt.dict_add(meta, pmt.intern("freq_hz"),
                            pmt.from_double(float(base_freq_hz)))
    for key, val in (("tx_samples", radar_meta["tx_samples"]),
                     ("rx_samples", radar_meta["rx_samples"]),
                     ("schedule_index", radar_meta.get("schedule_index", 0)),
                     ("pulse_id", radar_meta.get("pulse_id", 0)),
                     ("pulse_id_increment",
                      radar_meta.get("pulse_id_increment", 1)),
                     ("burst_count", radar_meta.get("burst_count", 0)),
                     ("publish_native",
                      radar_meta.get("publish_native", 0))):
        meta = pmt.dict_add(meta, pmt.intern(key), pmt.from_uint64(int(val)))
    meta = pmt.dict_add(meta, pmt.intern("sample_rate"),
                        pmt.from_double(float(radar_meta["sample_rate"])))
    for key, val in (("window_start_sample", 0),
                     ("pre_guard_samples", radar_meta["pre_guard_samples"]),
                     ("capture_samples", radar_meta["capture_samples"]),
                     ("post_guard_samples",
                      radar_meta["post_guard_samples"]),
                     ("sample_count", radar_meta["sample_count"]),
                     ("rx_capture_samples",
                      radar_meta["rx_capture_samples"]),
                     ("sync_repetitions", radar_meta["sync_repetitions"]),
                     ("code_index", radar_meta["code_index"]),
                     ("sync_samples", radar_meta["sync_samples"]),
                     ("sfd_samples", radar_meta["sfd_samples"]),
                     ("tx_packet_samples", radar_meta["tx_packet_samples"]),
                     ("num_delay_samps", radar_meta["num_delay_samps"])):
        meta = pmt.dict_add(meta, pmt.intern(key), pmt.from_long(int(val)))
    meta = pmt.dict_add(meta, pmt.intern("sfd_mode"),
                        pmt.intern(str(radar_meta["sfd_mode"])))
    meta = pmt.dict_add(meta, pmt.intern("source"),
                        pmt.intern(str(radar_meta.get("source",
                                                     "x410_echo"))))
    meta = pmt.dict_add(
        meta, pmt.intern("calibration_delay_native_samples"),
        pmt.from_double(float(radar_meta["cal_delay_native"])))
    # §5.3 multi-TX keys: float us/s already converted to native integer
    # ticks by jp.build_cpp_schedule_meta in Python.
    meta = pmt.dict_add(meta, pmt.intern("tx_channel_count"),
                        pmt.from_long(int(cpp_meta["tx_channel_count"])))
    meta = pmt.dict_add(meta, pmt.intern("tx_waveform_samples"),
                        pmt.init_u64vector(
                            len(cpp_meta["tx_waveform_samples"]),
                            [int(v) for v in
                             cpp_meta["tx_waveform_samples"]]))
    meta = pmt.dict_add(meta, pmt.intern("tx_base_offsets_native"),
                        pmt.init_u64vector(
                            len(cpp_meta["tx_base_offsets_native"]),
                            [int(v) for v in
                             cpp_meta["tx_base_offsets_native"]]))
    meta = pmt.dict_add(meta, pmt.intern("jam_logical_channel"),
                        pmt.from_long(int(cpp_meta["jam_logical_channel"])))
    meta = pmt.dict_add(meta, pmt.intern("jam_delay_mode"),
                        pmt.string_to_symbol(str(cpp_meta["jam_delay_mode"])))
    meta = pmt.dict_add(meta, pmt.intern("jam_delay_lo_native"),
                        pmt.from_long(int(cpp_meta["jam_delay_lo_native"])))
    meta = pmt.dict_add(meta, pmt.intern("jam_delay_hi_native"),
                        pmt.from_long(int(cpp_meta["jam_delay_hi_native"])))
    meta = pmt.dict_add(meta, pmt.intern("jam_delay_seed"),
                        pmt.from_uint64(int(cpp_meta["jam_delay_seed"])))
    offs = [float(o) for o in cpp_meta["jam_freq_offsets_hz"]]
    meta = pmt.dict_add(meta, pmt.intern("jam_freq_offsets_hz"),
                        pmt.init_f64vector(len(offs), offs))
    meta = pmt.dict_add(meta, pmt.intern("jam_dwell"),
                        pmt.from_uint64(int(cpp_meta["jam_dwell"])))
    meta = pmt.dict_add(meta, pmt.intern("jam_freq_settle_ticks"),
                        pmt.from_uint64(
                            int(cpp_meta["jam_freq_settle_ticks"])))
    v0 = pmt.init_s16vector(int(np.asarray(ch0_s16).size),
                            np.asarray(ch0_s16).tolist())
    v1 = pmt.init_s16vector(int(np.asarray(ch1_s16).size),
                            np.asarray(ch1_s16).tolist())
    vec = pmt.make_vector(2, pmt.PMT_NIL)
    pmt.vector_set(vec, 0, v0)
    pmt.vector_set(vec, 1, v1)
    return pmt.cons(meta, vec)


def _make_dual_tx_block(a, rate, pri_num, pre_guard_ticks):
    """Build the dual-TX C++ EchoTimer block (M1/M2 factory, probed).

    The multi-channel factory, TX channel/gain/antenna/frequency arrays
    and dry-run surface land with the C++ M1/M2 work (§6.3); until then
    this raises an actionable error instead of silently sending
    sense-only.  Kept tiny on purpose: one probe, one clear failure.
    """
    uwb_mod = base.uwb
    ctor = getattr(uwb_mod, "realtime_echo_timer_uhd_multitx", None)
    if ctor is None:
        raise SystemExit(
            "cpp-pdu dual-TX needs the C++ multi-TX EchoTimer factory "
            "(M1/M2, in progress): this build only provides single-TX "
            "realtime_echo_timer_uhd. "
            "Use --echo-backend python (the default for this app).")
    try:
        return ctor(
            device_args=a.args, sample_rate_hz=float(rate),
            tx_channels=[int(a.tx_channel), int(a.jam_channel)],
            rx_channel=int(a.rx_channel),
            tx_antennas=[a.tx_antenna, a.jam_antenna],
            rx_antenna=a.rx_antenna,
            tx_gains_db=[float(a.gain_tx), float(a.jam_gain_tx)],
            rx_gain_db=float(a.gain_rx),
            tx_freqs_hz=[float(a.freq), float(a.freq) + float(
                getattr(a, "jam_off0_hz", 0.0))],
            clock_source="internal", time_source="internal",
            # Scheduler PRI is an exact rational in DEVICE TICKS
            # (ticks_per_pri = pri_num / pri_den), so pri_den must be 1:
            # the tick count already carries the sample rate.  Same rule as
            # base.CppPduEcho (pri_num = round(pri_s * rate), pri_den = 1).
            pri_num=int(pri_num), pri_den=1,
            pre_guard_ticks=int(pre_guard_ticks),
            # Fragment size is a real throughput knob on the X410: with
            # 65536 a burst splits into 3 sends per channel and the worker
            # drifts (measured 2026-09-19: 300 Hz late=90..120); with
            # 262144 the burst is ONE send per channel and 300 Hz runs
            # late=0.  Exposed as --max-fragment-size.
            max_fragment_size=int(getattr(a, "max_fragment_size", 262144)),
            max_catchup_slots=1 << 20,
            queue_capacity=4, collect_wait_ms=1000,
            max_tx_samples=1 << 21, max_rx_samples=1 << 21,
            rx_freq_offset_hz=float(a.rx_freq_offset))
    except TypeError as exc:
        raise SystemExit(
            "dual-TX factory signature mismatch (C++ M2 API drift): %s; "
            "wire the landed factory name/args in _make_dual_tx_block or "
            "use --echo-backend python." % exc)


class JamCppPduEcho:
    """cpp-pdu dual-TX adapter: one multi-TX schedule PDU, no Python loop.

    Startup (once): the sense/jammer waveforms (already resampled to
    native upstream) are normalised exactly like
    ``JamTimedUhdEcho.prepare_jam`` (sense peak 0.8, jammer peak
    ``jam_scale`` of sense) and converted to SC16 once each.  The §5.3
    schedule metadata comes from ``jp.build_cpp_schedule_meta`` (float
    us/s converted to native integer ticks in Python).
    ``run_schedule`` posts a single
    ``cons(meta, pmt_vector[ch0_s16, ch1_s16])`` PDU and waits for the C++
    grid, exactly like ``base.CppPduEcho``.  No per-pulse ``_one_burst``,
    no ``tolist()`` composite rebuild, no sync raw-IQ dump.

    ``__init__`` is hardware-free (pure numpy + geometry); the C++ block
    is built by ``_build_block()`` so ``--dry-run`` never touches UHD.
    """

    def __init__(self, a, profile, sense_native, jam_native, jam_offsets,
                 jam_delay_random_us):
        self._a = a
        self._profile = profile
        self.rate = float(profile.hz)
        self.tx_interp = int(profile.tx_interp)
        self.tx_decim = int(profile.tx_decim)
        self.native_label = profile.label
        self.freq = float(a.freq)
        self.rx_freq_offset = float(a.rx_freq_offset)
        self.code_index = int(a.code_index)
        self.sfd_mode = base.SFD_MODE
        self.tx_ch = int(a.tx_channel)
        self.rx_ch = int(a.rx_channel)
        self.jam_ch = int(a.jam_channel)
        self.tx_channels = [self.tx_ch, self.jam_ch]
        self.pri_s = float(a.pri_s)
        self.sync_reps = int(a.sync_reps)
        self.cal_delay_native = float(a.cal_delay_native)
        self.pre_us = float(a.pre_guard_us)
        self.range_m = 15.0
        self.tail_us = float(a.tail_guard_us)
        self.rx_pad_us = float(a.rx_pad_us)
        self.arm_delay_s = float(a.arm_delay_s)
        self.max_pulses = int(a.pulses)

        self.jam_enable = True
        self.jam_antenna = a.jam_antenna
        self.jam_gain_tx = float(a.jam_gain_tx)
        self.jam_scale = float(a.jam_scale)
        self.jam_delay_random_us = jam_delay_random_us
        self.jam_offsets = list(jam_offsets) if jam_offsets else []
        self.jam_dwell = max(0, int(a.jam_dwell))
        self.freq_settle_s = float(a.jam_freq_settle_s)
        off0 = (float(self.jam_offsets[0]) if self.jam_offsets
                else float(a.jam_freq_offset))
        self.jam_freq_offset = off0
        self.jam_freq_hz = self.freq + off0
        # Commanded jammer frequency; the NCO readback arrives via the C++
        # retune path (M5), so actual == commanded until retunes are
        # reported by the block.
        self.jam_freq_actual = self.jam_freq_hz
        if jam_delay_random_us is not None:
            self.jam_delay_us = 0.0
            self.jam_delay_native = 0
        else:
            self.jam_delay_us = float(a.jam_delay_us)
            self.jam_delay_native = jp.placement_native(
                self.jam_delay_us, self.rate)
        a.jam_off0_hz = off0

        # -- one-time waveform prep (mirrors prepare_jam scaling) --------
        sense_arr = np.asarray(sense_native, dtype=np.complex64)
        if sense_arr.size == 0:
            raise SystemExit("cpp-pdu dual-TX needs a sensing waveform")
        if jam_native is None or np.asarray(jam_native).size == 0:
            raise SystemExit(
                "cpp-pdu dual-TX needs a jammer waveform "
                "(jam enabled but empty)")
        sense_peak_raw = float(np.max(np.abs(sense_arr))) or 1.0
        self._sense_norm = (sense_arr / sense_peak_raw * 0.8).astype(
            np.complex64)
        sense_peak = float(np.max(np.abs(self._sense_norm))) or 1.0
        jam_arr = np.asarray(jam_native, dtype=np.complex64)
        jam_peak = float(np.max(np.abs(jam_arr))) or 1.0
        self._jam_scaled = (jam_arr / jam_peak
                            * (self.jam_scale * sense_peak)).astype(
                                np.complex64)
        self._sense_sc16 = base.fc32_to_sc16(self._sense_norm)
        self._jam_sc16 = base.fc32_to_sc16(self._jam_scaled)

        # -- §5.3 schedule metadata (float→ticks here, not on hot path) ---
        self.cpp_meta = jp.build_cpp_schedule_meta(
            sense_len=int(self._sense_norm.size),
            jam_len=int(self._jam_scaled.size),
            native_hz=self.rate,
            delay_us=self.jam_delay_us,
            delay_random_us=self.jam_delay_random_us,
            delay_seed=int(a.jam_delay_seed or 0),
            jam_offsets_hz=list(self.jam_offsets),
            jam_dwell=self.jam_dwell,
            freq_settle_s=self.freq_settle_s,
            jam_logical_channel=1)
        self.tx_samples_L = int(self.cpp_meta["tx_samples"])
        self.tx_waveform_samples = [int(v) for v in
                                    self.cpp_meta["tx_waveform_samples"]]
        self.tx_base_offsets_native = [int(v) for v in
                                       self.cpp_meta["tx_base_offsets_native"]]
        self.jam_delay_native_lo = int(
            self.cpp_meta["jam_delay_lo_native"])
        self.jam_delay_native_hi = int(
            self.cpp_meta["jam_delay_hi_native"])
        self.jam_delay_seed = int(self.cpp_meta["jam_delay_seed"])
        self.jam_delay_mode = str(self.cpp_meta["jam_delay_mode"])
        self.jam_freq_settle_ticks = int(
            self.cpp_meta["jam_freq_settle_ticks"])
        self.window_layout = jp.contiguous_window_layout(
            sense_len=int(self._sense_norm.size),
            jam_len=int(self._jam_scaled.size),
            native_hz=self.rate,
            delay_us=self.jam_delay_us,
            delay_random_us=self.jam_delay_random_us)
        self.max_fragment_size = int(
            getattr(a, "max_fragment_size", 262144))
        try:
            jp.require_fragment_covers_L(
                self.max_fragment_size, self.tx_samples_L)
        except ValueError as exc:
            raise SystemExit(str(exc))
        # Sense parked at D for uniform delay: fold D into the published
        # pre-guard/RX window exactly like prepare_jam so the CIR origin
        # stays on the sensing start (timed grid itself is unshifted).
        self.tx_lead_native = int(self.tx_base_offsets_native[0])
        (pre0, self.sync_n, self.sfd_n, self.rng_n, self.tail,
         self.pad_n, rx0) = base.rx_geometry(
            self.rate, self.pre_us, self.sync_reps, self.range_m,
            self.tail_us, self.tx_samples_L, self.rx_pad_us,
            self.tx_interp, self.tx_decim)
        if int(a.publish_native) < 0:
            pub = base.cir_publish_native(
                pre0, self.sync_reps, self.cal_delay_native, self.rate)
        else:
            pub = int(a.publish_native)
        if self.tx_lead_native:
            self.pre = int(pre0) + int(self.tx_lead_native)
            self.rx_len = (int(rx0) + int(self.tx_lead_native) + 3) // 4 * 4
            self.publish_native = (int(pub) + int(self.tx_lead_native)
                                   if int(pub) > 0 else int(pub))
        else:
            self.pre = int(pre0)
            self.rx_len = int(rx0)
            self.publish_native = int(pub)

        self.freq_plan = _JamFreqPlan(
            self.freq, off0, self.jam_offsets, self.jam_dwell)
        # Python-compat attributes read by main()/live stats/summary.
        self._blk = None
        self._pub_q = base._EmptyQueue()
        self._timing = []
        self._pub_timing = []
        self._sc16_written = 0
        self._sc16_offset = 0
        self._arm_ticks = int(round(self.arm_delay_s * self.rate))
        pri_num = int(round(self.pri_s * self.rate))
        if pri_num <= 0:
            raise SystemExit("pri_s too small for the device tick grid")
        self._pri_num = pri_num
        self._pre_guard_ticks = int(round(self.pre_us * 1e-6 * self.rate))

        self.status = {
            "backend": "cpp-pdu-multitx",
            "tx_rate": self.rate, "rx_rate": self.rate,
            "tx_ant": a.tx_antenna, "rx_ant": a.rx_antenna,
            "rx_window": self.rx_len, "pre": self.pre,
            "publish_native": self.publish_native,
            "pri_s": self.pri_s, "max_pulses": self.max_pulses,
            "rx_pad_us": self.rx_pad_us,
            "code_index": self.code_index, "sfd_mode": self.sfd_mode,
            "sync_reps": self.sync_reps,
            "tx_native": self.tx_samples_L,
            "tx_waveform_samples": list(self.tx_waveform_samples),
            "tx_base_offsets_native":
                list(self.tx_base_offsets_native),
            "rx_window_us": self.rx_len / self.rate * 1e6,
            "tx_us": self.tx_samples_L / self.rate * 1e6,
            "pad": self.pad_n,
            "tx_channels": list(self.tx_channels),
            "jam_enabled": True,
            "jam_channel": self.jam_ch,
            "jam_delay_mode": self.jam_delay_mode,
            "jam_delay_seed": self.jam_delay_seed,
        }

    # -- C++ block counters (probed; missing M5/M6 methods read as 0) -----
    def _blk_u64(self, name):
        blk = self._blk
        if blk is None:
            return 0
        fn = getattr(blk, name, None)
        if not callable(fn):
            return 0
        try:
            return int(fn())
        except Exception:
            return 0

    @property
    def _ok(self):
        return self._blk_u64("bursts_ok")

    @property
    def _fail(self):
        return self._blk_u64("bursts_failed")

    @property
    def _late(self):
        return self._blk_u64("late_slot_skips")

    @property
    def _pub_ok(self):
        return self._blk_u64("bursts_published")

    @property
    def _tx_send_error(self):
        return 0

    @property
    def blk(self):
        return self._blk

    @property
    def jam_retune_count(self):
        return self._blk_u64("jam_retunes")

    @property
    def jam_retune_fail(self):
        return self._blk_u64("jam_retune_failures")

    def cpp_async_counts(self):
        """TX async event counters (§5.6); zeros until M2 exposes them."""
        out = {"underflow": 0, "seq_error": 0, "time_error": 0,
               "unmatched": 0, "dropped": 0, "ack": 0}
        blk = self._blk
        fn = getattr(blk, "tx_async_counts", None) if blk is not None else None
        if not callable(fn):
            return out
        try:
            cur = fn()
        except Exception:
            return out
        if isinstance(cur, dict):
            for k in out:
                try:
                    out[k] = int(cur.get(k, 0))
                except (TypeError, ValueError):
                    pass
        else:
            for k in out:
                v = getattr(cur, k, None)
                if isinstance(v, (int, float)):
                    out[k] = int(v)
        return out

    def set_tx_native(self, wave):  # waveforms captured in __init__
        pass

    def start_publisher(self):
        pass

    def stop_publisher(self):
        pass

    def print_dry_run(self):
        print("[jam] cpp-schedule: tx_channel_count=2 tx_samples(L)=%d "
              "tx_waveform_samples=[%d,%d] tx_base_offsets_native=[%d,%d] "
              "jam_delay_mode=%s lo=%d hi=%d seed=%d offsets=%d dwell=%d "
              "settle_ticks=%d payload=pmt_vector[s16:%d,s16:%d] "
              "rx_len=%d pre=%d publish_native=%d (tx_channels=%s)"
              % (self.tx_samples_L,
                 self.tx_waveform_samples[0], self.tx_waveform_samples[1],
                 self.tx_base_offsets_native[0],
                 self.tx_base_offsets_native[1],
                 self.jam_delay_mode,
                 self.jam_delay_native_lo, self.jam_delay_native_hi,
                 self.jam_delay_seed, len(self.jam_offsets), self.jam_dwell,
                 self.jam_freq_settle_ticks,
                 int(self._sense_sc16.size), int(self._jam_sc16.size),
                 self.rx_len, self.pre, self.publish_native,
                 self.tx_channels), flush=True)
        lay = self.window_layout
        print("[jam] cpp-layout: layout=contiguous-window L=%d D=%d "
              "backing_length=%d jam_window=[%d, %d] data_fragments=%d "
              "max_fragment_size=%d"
              % (lay["L"], lay["D"], lay["backing_length"],
                 lay["jam_window_begin_min"], lay["jam_window_begin_max"],
                 lay["data_fragments"], self.max_fragment_size),
              flush=True)
        print("[jam] cpp-schedule-meta %s"
              % json.dumps(self.cpp_meta, sort_keys=True), flush=True)

    def _build_block(self):
        if self._blk is None:
            self._blk = _make_dual_tx_block(
                self._a, self.rate, self._pri_num, self._pre_guard_ticks)
        return self._blk

    def _radar_meta(self):
        cap = self.rx_len - self.pre - self.tail
        if cap < 0:
            cap = self.rx_len
        return {
            "tx_samples": self.tx_samples_L,
            "rx_samples": self.rx_len,
            "schedule_index": 0,
            "pulse_id": 0,
            "pulse_id_increment": 1,
            "burst_count": self.max_pulses,
            "publish_native": self.publish_native,
            "sample_rate": self.rate,
            "pre_guard_samples": self.pre,
            "capture_samples": cap,
            "post_guard_samples": self.tail,
            "sample_count": self.rx_len,
            "rx_capture_samples": self.rx_len,
            "sync_repetitions": self.sync_reps,
            "sfd_mode": self.sfd_mode,
            "code_index": self.code_index,
            "sync_samples": self.sync_n,
            "sfd_samples": self.sfd_n,
            "tx_packet_samples": self.tx_samples_L,
            "num_delay_samps": int(round(self.cal_delay_native)),
            "cal_delay_native": self.cal_delay_native,
            "source": "x410_echo",
        }

    def run_schedule(self):
        # Must be called AFTER tb.start(): the block's start() prepares the
        # UHD backend, after which the device clock is readable.
        self._build_block()
        t0 = self._blk.device_time_ticks() + self._arm_ticks
        sched = build_cpp_schedule_pdu(
            self.cpp_meta, self._radar_meta(),
            self._sense_sc16, self._jam_sc16,
            base_freq_hz=self.freq)
        meta = pmt.dict_add(pmt.car(sched), pmt.intern("t0_ticks"),
                            pmt.from_long(t0))
        sched = pmt.cons(meta, pmt.cdr(sched))
        self._blk._post(pmt.intern("schedule"), sched)
        print("cpp_pdu_multitx schedule t0_ticks=%d tx_samples(L)=%d "
              "waveforms=[%d,%d] offsets=[%d,%d] rx_samples=%d "
              "publish_native=%d bursts=%d delay_mode=%s"
              % (t0, self.tx_samples_L,
                 self.tx_waveform_samples[0], self.tx_waveform_samples[1],
                 self.tx_base_offsets_native[0],
                 self.tx_base_offsets_native[1],
                 self.rx_len, self.publish_native, self.max_pulses,
                 self.jam_delay_mode), flush=True)
        # Scan dwell is a successful-capture quota.  Failed/late attempts
        # are retried by the C++ worker, and each inter-frequency retune adds
        # settle time that is outside the nominal pulse grid.
        retune_settle_s = max(0, len(self.jam_offsets) - 1) * self.freq_settle_s
        deadline = (time.monotonic() + self.max_pulses * self.pri_s +
                    retune_settle_s + 15.0)
        while time.monotonic() < deadline:
            if self._blk.bursts_ok() >= self.max_pulses:
                break
            time.sleep(0.02)
        if self._blk.bursts_ok() < self.max_pulses:
            raise RuntimeError(
                "cpp_pdu_multitx scan incomplete: ok=%d/%d published=%d "
                "failed=%d late=%d last_error=%s"
                % (self._blk.bursts_ok(), self.max_pulses,
                   self._blk.bursts_published(), self._blk.bursts_failed(),
                   self._blk.late_slot_skips(), self._blk.last_error()))
        return (int(self._blk.bursts_ok()),
                int(self._blk.bursts_failed()),
                int(self._blk.late_slot_skips()))


def parse_args():
    p = argparse.ArgumentParser(
        parents=[base.build_parser(add_help=False)],
        description="X410 dual-TX HRP jammer for UWB sensing CIR: a second "
                    "TX channel carries a different-preamble-code HRP signal "
                    "at an exact NCO frequency offset (align supports "
                    "--echo-backend cpp-pdu and python; continuous is "
                    "python-only).")
    g = p.add_argument_group("jammer")
    g.add_argument("--jam-enable", action="store_true",
                   help="Transmit the second (jammer) TX channel.  Without "
                        "it the app degenerates to the base single-TX app.")
    g.add_argument("--jam-mode", choices=list(jp.JAM_MODES), default="align",
                   help="align = jammer buffer re-armed to the sensing PRI "
                        "(default); continuous = independent repeat; off = "
                        "silent")
    g.add_argument("--jam-channel", type=int, default=jp.DEFAULT_JAM_CHANNEL,
                   help="X410 TX channel for the jammer (must differ from "
                        "--tx-channel/--rx-channel; default %d = DB0/RF1, the "
                        "same daughterboard as the sensing TX)"
                        % jp.DEFAULT_JAM_CHANNEL)
    g.add_argument("--jam-antenna", default="TX/RX0")
    g.add_argument("--jam-gain-tx", type=float, default=20.0,
                   help="jammer channel TX gain (0..60 dB); start low")
    g.add_argument("--jam-freq-offset", type=float, default=0.0,
                   help="jammer carrier offset vs --freq, in Hz (may be "
                        "negative).  This IS the sensing<->jammer CFO: the RX "
                        "LO stays at --freq, so the jammer appears at baseband "
                        "+offset.  Use 491340.0 for the f_work/(2*SPS) null.")
    g.add_argument("--jam-freq-offsets", default="",
                   help="comma-separated offsets (Hz) for a dwell scan, e.g. "
                        "\"0,245.67e3,491.34e3\"; overrides --jam-freq-offset. "
                        "Mutually exclusive with --jam-freq-start.")
    g.add_argument("--jam-freq-start", type=float, default=None,
                   help="jammer offset scan start in Hz (relative to --freq). "
                        "Requires --jam-freq-stop and --jam-freq-step; "
                        "stop < start sweeps down.")
    g.add_argument("--jam-freq-stop", type=float, default=None,
                   help="jammer offset scan stop in Hz (always included)")
    g.add_argument("--jam-freq-step", type=float, default=None,
                   help="jammer offset scan step magnitude in Hz")
    g.add_argument("--jam-dwell", type=int, default=50,
                   help="CIR pulses per jammer offset when scanning")
    g.add_argument("--jam-code-index", type=int,
                   default=jp.DEFAULT_JAM_CODE_INDEX,
                   choices=[9, 10, 11, 12],
                   help="jammer HRP preamble code (default 10 = the "
                        "communication code; sensing uses 9)")
    g.add_argument("--jam-preamble-length", type=int, default=None,
                   choices=list(base.PREAMBLE_LENGTH_CHOICES),
                   help="jammer SYNC repetitions (default: same as "
                        "--preamble-length)")
    g.add_argument("--jam-psdu-hex", default=None,
                   help="jammer PSDU bytes (default: the same PSDU as the "
                        "sensing packet)")
    g.add_argument("--jam-waveform", choices=["preamble", "packet"],
                   default="preamble",
                   help="preamble = keep only the SYNC segment (pure periodic "
                        "preamble, best for the CFO null); packet = full HRP "
                        "packet (more realistic)")
    g.add_argument("--jam-pulse-shape", default=None,
                   help="jammer pulse shape (default: inherit --pulse-shape)")
    g.add_argument("--jam-pulse-sigma-ns", type=float, default=None)
    g.add_argument("--jam-pulse-bw-mhz", type=float, default=None)
    g.add_argument("--jam-pulse-taps", default="")
    g.add_argument("--jam-delay-us", type=float, default=0.0,
                   help="jammer placement inside the RX window, relative to "
                        "the sensing burst start (ignored when "
                        "--jam-delay-random-us is set)")
    g.add_argument("--jam-delay-random-us", default="",
                   help="per-pulse jammer time offset, uniform on the native "
                        "sample grid in [-T, +T] microseconds.  Empty keeps "
                        "the fixed --jam-delay-us.  T <= 2000 us.")
    g.add_argument("--jam-delay-seed", type=int, default=None,
                   help=argparse.SUPPRESS)
    g.add_argument("--jam-scale", type=float, default=0.3,
                   help="jammer peak amplitude / sensing TX peak (the sensing "
                        "is normalised to 0.8 full scale)")
    g.add_argument("--jam-repeat-pri-us", type=float, default=0.0,
                   help="continuous-mode jammer repeat interval in us")
    g.add_argument("--jam-no-sts", action="store_true")
    g.add_argument("--jam-freq-settle-s", type=float, default=0.05,
                   help="settle time after each jammer retune")
    g.add_argument("--max-fragment-size", type=int, default=262144,
                   help="per-send fragment size (samples/channel) for the "
                        "multi-TX burst plan.  262144 keeps the whole "
                        "188999-sample burst in ONE send per channel "
                        "(X410 measured: 65536 -> 3 sends and late skips at "
                        "200-300 Hz; 262144 -> late 0 at 300 Hz)")
    g.add_argument("--dry-run", action="store_true",
                   help="print the jam plan and exit without touching UHD")
    return p.parse_args()


def main():
    base.bootstrap_uhd_env()
    a = parse_args()
    if a.echo_backend is None:
        # Default stays python until the C++ dual-TX path passes hardware
        # acceptance (M6/M7 flip the default).
        a.echo_backend = "python"
    use_cpp = (a.echo_backend == "cpp-pdu")

    # --preamble-length (primary) / --sync-reps (alias) resolution.
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

    jam_enabled = bool(a.jam_enable) and a.jam_mode != "off"
    jam_offsets = parse_freq_offsets(a.jam_freq_offsets)
    scan_args = (a.jam_freq_start, a.jam_freq_stop, a.jam_freq_step)
    scan_set = [v is not None for v in scan_args]
    if jam_offsets and any(scan_set):
        raise SystemExit(
            "--jam-freq-offsets cannot be combined with "
            "--jam-freq-start/--jam-freq-stop/--jam-freq-step")
    if any(scan_set) and not all(scan_set):
        raise SystemExit(
            "jammer offset scan needs all of --jam-freq-start, "
            "--jam-freq-stop and --jam-freq-step")
    if all(scan_set):
        try:
            jam_offsets = jp.make_freq_offset_scan(
                a.jam_freq_start, a.jam_freq_stop, a.jam_freq_step)
        except ValueError as exc:
            raise SystemExit(str(exc))
    jam_delay_random = jp.parse_delay_random_us(a.jam_delay_random_us)
    if jam_delay_random is not None and a.jam_delay_seed is None:
        # Resolve the seed once in main so --dry-run prints the exact seed
        # both backends will use (JamTimedUhdEcho auto-generates the same
        # way when it receives None).
        a.jam_delay_seed = int(
            np.random.default_rng().integers(0, 2**31 - 1))
    if use_cpp and jam_enabled and a.jam_mode == "continuous":
        raise SystemExit(
            "--echo-backend cpp-pdu does not support --jam-mode continuous: "
            "the independent jammer streamer lives only in the python "
            "backend (a C++ continuous streamer is out of scope, §11). "
            "Use --echo-backend python (the default for this app).")
    if jam_offsets:
        if a.jam_dwell <= 0:
            raise SystemExit("jammer offset scan needs --jam-dwell >= 1")
        a.pulses = len(jam_offsets) * int(a.jam_dwell)
        step = (jam_offsets[1] - jam_offsets[0]) if len(jam_offsets) > 1 else 0.0
        print("[jam] offset scan: %d offsets x dwell %d -> %d pulses "
              "(%.3f -> %.3f kHz, step %+.3f kHz, wall_s=%.1f)"
              % (len(jam_offsets), a.jam_dwell, a.pulses,
                 jam_offsets[0] / 1e3, jam_offsets[-1] / 1e3, step / 1e3,
                 a.pulses * a.pri_s), flush=True)
    jp.validate_jam_args({
        "jam_enabled": jam_enabled,
        "jam_mode": a.jam_mode,
        "jam_channel": int(a.jam_channel),
        "tx_channel": int(a.tx_channel),
        "rx_channel": int(a.rx_channel),
        "jam_freq_offset_hz": float(
            jam_offsets[0] if jam_offsets else a.jam_freq_offset),
        "jam_code_index": int(a.jam_code_index),
        "jam_scale": float(a.jam_scale),
        "jam_repeat_pri_us": float(a.jam_repeat_pri_us),
        "jam_delay_random_us": jam_delay_random,
    })

    repo = base.find_repo_root()
    os.makedirs(a.output, exist_ok=True)
    profile = base.NativeRateProfile(a.native_rate)
    if a.cal_delay_native is None:
        a.cal_delay_native = base.default_cal_delay_native(profile.hz)
    print("native_rate=%.1f MS/s (%s) tx=%d/%d pdu=%s work_per_native=%.6f "
          "cal_delay_native=%.1f"
          % (profile.hz / 1e6, profile.name, profile.tx_interp,
             profile.tx_decim, profile.pdu, profile.work_per_native,
             a.cal_delay_native), flush=True)
    taps = a.taps or os.path.join(
        repo, "testdata", profile.taps_dir, "taps_quality_minorder.txt")
    tmpl_path = a.template or os.path.join(a.output, "sync_template_live.cf32")
    timing_path = os.path.join(a.output, "echo_timing.jsonl")

    # -- sensing waveform (identical to the base app) -----------------------
    psdu = base.hex_to_bytes(a.psdu_hex)
    pulse_shape, pulse_taps = _resolve_shape(a.pulse_shape, a.pulse_taps, repo)
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
          "insert_sts=%s pulse_shape=%s pulse_taps=%d code_index=%d "
          "preamble_length=%d sfd_mode=%s"
          % (samples.size, profile.label, native.size,
             (time.perf_counter() - t_rs) * 1e3, insert_sts, src.pulse_shape(),
             src.pulse_taps(), a.code_index,
             a.sync_reps, base.SFD_MODE), flush=True)

    # -- jammer waveform ----------------------------------------------------
    jam_sync_reps = (a.jam_preamble_length if a.jam_preamble_length is not None
                     else a.sync_reps)
    jam_native = None
    jam_work_samples = 0
    if jam_enabled:
        jam_psdu = base.hex_to_bytes(a.jam_psdu_hex or a.psdu_hex)
        if a.jam_pulse_shape is None and not a.jam_pulse_taps:
            jam_shape, jam_taps = pulse_shape, pulse_taps
        else:
            jam_shape, jam_taps = _resolve_shape(
                a.jam_pulse_shape or a.pulse_shape, a.jam_pulse_taps, repo)
        jam_sigma = (a.jam_pulse_sigma_ns if a.jam_pulse_sigma_ns is not None
                     else a.pulse_sigma_ns)
        jam_bw = (a.jam_pulse_bw_mhz if a.jam_pulse_bw_mhz is not None
                  else a.pulse_bw_mhz)
        jam_sts = False if a.jam_no_sts else insert_sts
        jam_src = base.uwb.hrp_packet_source(
            jam_psdu, jam_sync_reps, base.SFD_MODE, a.jam_code_index, 0.8,
            a.pri_s, False, jam_sts, False, jam_shape, jam_sigma, jam_bw,
            jam_taps)
        jam_work = np.array(jam_src.samples(), dtype=np.complex64)
        full_work = int(jam_work.size)
        if a.jam_waveform == "preamble":
            jam_work = jam_work[:jam_sync_reps * base.SPS]
        t_j = time.perf_counter()
        jam_native = profile.tx_native(jam_work)
        jam_work_samples = int(jam_work.size)
        print("jam_enabled code=%d waveform=%s preamble=%d jam_work=%d "
              "(full=%d) jam_native=%d resample_ms=%.2f"
              % (a.jam_code_index, a.jam_waveform, jam_sync_reps,
                 jam_work_samples, full_work, jam_native.size,
                 (time.perf_counter() - t_j) * 1e3), flush=True)
        off0 = jam_offsets[0] if jam_offsets else a.jam_freq_offset
        if jam_delay_random is not None:
            delay_txt = "uniform ±%.3f us" % jam_delay_random
        else:
            delay_txt = "fixed %.3f us" % a.jam_delay_us
        print("[jam] mode=%s channel=%d ant=%s gain=%.1f freq=%.6f MHz "
              "offset=%+.6f kHz scale=%.3f delay=%s"
              % (a.jam_mode, a.jam_channel, a.jam_antenna, a.jam_gain_tx,
                 (a.freq + off0) / 1e6, off0 / 1e3, a.jam_scale,
                 delay_txt), flush=True)
        if jam_offsets and len(jam_offsets) <= 16:
            print("[jam] offsets(kHz)=%s"
                  % ",".join("%+.3f" % (o / 1e3) for o in jam_offsets),
                  flush=True)
    else:
        print("jam_enabled=False: single-TX sensing only", flush=True)

    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f"
          % (a.pulses, a.pri_s, 1.0 / a.pri_s, a.pulses * a.pri_s), flush=True)

    if a.dry_run:
        jam_len = 0 if jam_native is None else int(jam_native.size)
        if jam_delay_random is not None:
            half = jp.delay_half_span_native(jam_delay_random, profile.hz)
            lo_n, hi_n = -half, half
            length = jp.bipolar_tx_len(int(native.size), jam_len, half)
        else:
            lo_n = hi_n = jp.placement_native(a.jam_delay_us, profile.hz)
            length = jp.combined_tx_len(int(native.size), jam_len, hi_n)
        print("[jam] dry-run: sense_native=%d jam_native=%d "
              "delay_native=[%d, %d] composite_len=%d (tx_channels=%s)"
              % (native.size, jam_len, lo_n, hi_n, length,
                 [a.tx_channel, a.jam_channel] if jam_enabled
                 else [a.tx_channel]), flush=True)
        if use_cpp:
            if jam_enabled:
                # Pure prep only (no UHD): prints the one-shot multi-TX
                # schedule PDU geometry the C++ grid will receive.
                JamCppPduEcho(
                    a, profile, native, jam_native, jam_offsets,
                    jam_delay_random).print_dry_run()
            else:
                print("[jam] dry-run cpp-pdu single-TX: tx_samples=%d "
                      "rx via standard geometry (jam disabled)"
                      % (int(native.size),), flush=True)
        print("[jam] dry-run complete", flush=True)
        raise SystemExit(0)

    # Fold --use-dpdk / --mgmt-addr into a.args BEFORE the block is built:
    # without this the device args stay kernel-UDP and `use_dpdk=1` never
    # reaches UHD (measured 2026-09-19: mgmt_addr stayed on the QSFP addr and
    # the run silently fell back to kernel UDP with mass TX underflow).
    base.resolve_dpdk_args(a)
    base.dpdk_preflight(a)

    if a.require_sfd and a.publish_native < 0:
        print("WARN --require-sfd with --publish-native -1 (auto ROI): "
              "falling back to full window (0)", flush=True)
        a.publish_native = 0
    if use_cpp and (a.dump_rx or a.dump_sc16):
        # The cpp-pdu acceptance path has no sync raw-IQ sink on the radio
        # worker (an async PDU writer is future work, §6.2, not a perf
        # gate); disable instead of silently dropping samples.
        print("WARN --dump-rx/--dump-sc16 are unsupported with "
              "--echo-backend cpp-pdu; disabling dumps", flush=True)
        a.dump_rx = False
        a.dump_sc16 = False
    dump_dir = os.path.join(a.output, "rx_iq") if a.dump_rx else ""
    sc16_dir = a.output if a.dump_sc16 else ""

    off0 = jam_offsets[0] if jam_offsets else a.jam_freq_offset
    if use_cpp:
        if jam_enabled:
            # One-shot multi-TX schedule PDU (M3 fixed align); the C++
            # grid owns all timed I/O, retune and per-pulse delay.
            echo = JamCppPduEcho(
                a, profile, native, jam_native, jam_offsets,
                jam_delay_random)
            echo._build_block()
        else:
            # Jam disabled: reuse the standard single-TX cpp-pdu app path
            # so payload/metadata/CIR stay bit-compatible with it.
            echo = base.CppPduEcho(a, native, profile)
    else:
        echo = JamTimedUhdEcho(
            a.args, profile, a.freq, a.tx_channel, a.rx_channel,
            a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx,
            a.pre_guard_us, 15.0, a.tail_guard_us, a.sync_reps,
            a.cal_delay_native, a.arm_delay_s, a.pri_s, a.pulses, dump_dir,
            a.min_lead_s, timing_path, sc16_dir, a.rx_pad_us, a.code_index,
            base.SFD_MODE, publish_native=a.publish_native,
            rx_freq_offset=a.rx_freq_offset,
            jam_enable=jam_enabled, jam_ch=a.jam_channel,
            jam_antenna=a.jam_antenna, jam_gain_tx=a.jam_gain_tx,
            jam_freq_offset=off0, jam_scale=a.jam_scale,
            jam_delay_us=a.jam_delay_us, jam_offsets=jam_offsets,
            jam_dwell=a.jam_dwell, freq_settle_s=a.jam_freq_settle_s,
            jam_delay_random_us=jam_delay_random,
            jam_delay_seed=a.jam_delay_seed)
        if jam_enabled:
            echo.prepare_jam(native, jam_native)
        else:
            echo.set_tx_native(native)
    print("echo_backend=%s" % a.echo_backend, flush=True)
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
    elif use_cpp:
        # cpp-pdu publishes native SC16 -> normalise like the UHD FC32
        # chain (same "auto" rule as the base app).
        sc16_scale = base.uwb.Sc16ScalePolicy.UnitRange
    else:  # python backend -> raw integer like the legacy chain
        sc16_scale = base.uwb.Sc16ScalePolicy.RawInteger
    res = profile.make_pdu_resampler(taps, a.res_workers, sc16_scale)
    est_q = max(8, int(a.est_queue))
    use_pred = not a.require_sfd
    cir_skip_initial = 0 if a.cir_output == "repetitions" else 10
    est = base.uwb.radar_cir_estimator(
        tmpl_path, a.sync_reps, base.SFD_MODE, a.code_index, 16, 100,
        cir_skip_initial, 0,
        a.sfd_search_margin, a.sync_refine_margin, a.sfd_threshold,
        a.sync_refine_threshold, True, est_q, use_pred,
        a.cir_output == "repetitions")
    cir_records_per_pulse = (max(1, int(a.sync_reps) - cir_skip_initial)
                             if a.cir_output == "repetitions" else 1)
    wr = base.uwb.cir_writer(a.output, "cir", True,
                             max(256, 4 * cir_records_per_pulse))
    udp = None
    if (not a.no_udp) and bool(a.udp_host):
        if use_cpp and jam_enabled:
            udp = base.CirUdpSink(
                a.udp_host, int(a.udp_port), base.CIR_UDP_TAPS,
                freq_lookup=lambda pid: echo.freq_plan.lookup(int(pid)))
        elif use_cpp:
            udp = base.CirUdpSink(
                a.udp_host, int(a.udp_port), base.CIR_UDP_TAPS,
                freq_lookup=lambda pid: (echo.freq, 0.0))
        else:
            udp = base.CirUdpSink(
                a.udp_host, int(a.udp_port), base.CIR_UDP_TAPS,
                freq_lookup=lambda pid: echo.jam_by_pulse.get(
                    int(pid), (echo.jam_freq_hz, echo.jam_freq_offset)))
        print("udp_cir %s:%s framed=UCR3 repetition+jam_freq from "
              "%s" % (a.udp_host, a.udp_port,
                      "jam freq plan (cpp-pdu)" if use_cpp and jam_enabled
                      else ("sense freq (cpp-pdu single-TX)" if use_cpp
                            else "jam_by_pulse")), flush=True)

    tb = gr.top_block("x410_cg400_hrp_echo_cir_jam")
    if use_cpp:
        # The cpp-pdu path is the C++ block itself; the wrapper only
        # carries the Python-compat attribute surface.
        tb.msg_connect((echo.blk, "burst"), (res, "packet"))
    else:
        tb.msg_connect((echo, "rx"), (res, "packet"))
    tb.msg_connect((res, "packet"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if udp is not None:
        tb.msg_connect((est, "cir"), (udp, "cir"))

    tb.start()
    echo.start_publisher()
    live_stop = threading.Event()
    live_th = base.start_live_stats(echo, est, wr, udp, live_stop, a.pri_s, res)
    t_run = time.perf_counter()
    echo.run_schedule()
    sched_s = time.perf_counter() - t_run
    print("schedule_wall_s=%.3f" % sched_s, flush=True)
    live_stop.set()
    live_th.join(timeout=1.5)

    deadline = time.time() + 8.0
    while time.time() < deadline:
        written = wr.frames_written() + wr.frames_failed()
        if (written >= echo._ok * cir_records_per_pulse and est.drained()
                and echo._pub_q.empty()):
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
    cir_stats = base.analyze_cir(jsonl, a.pulses)
    timing_stats = base.analyze_timing(timing_path)
    # -- summary locals (per backend/path; the literal below stays flat) --
    _async_zero = {"underflow": 0, "seq_error": 0, "time_error": 0,
                   "unmatched": 0, "dropped": 0, "ack": 0}
    if use_cpp and jam_enabled:
        tx_channels_list = list(echo.tx_channels)
        sum_delay_us = float(echo.jam_delay_us)
        sum_delay_native = int(echo.jam_delay_native)
        sum_delay_random = echo.jam_delay_random_us
        sum_delay_lo = int(echo.jam_delay_native_lo)
        sum_delay_hi = int(echo.jam_delay_native_hi)
        sum_tx_lead = int(echo.tx_lead_native)
        sum_delay_seed = (int(echo.jam_delay_seed)
                          if echo.jam_delay_random_us is not None else None)
        sum_tx_samples = int(echo.tx_samples_L)
        sum_freq_offset = float(echo.jam_freq_offset)
        sum_freq_actual = float(echo.jam_freq_actual)
        sum_retune = int(echo.jam_retune_count)
        sum_retune_fail = int(echo.jam_retune_fail)
        # Placeholder: the C++ grid applies per-pulse delays device-side
        # (M4); a published per-pulse delay counter does not exist yet.
        sum_delay_updates = 0
        sum_async = echo.cpp_async_counts()
        # Bounded per-dwell plan (the C++ grid owns per-pulse execution).
        jam_sweep = echo.freq_plan.dwell_plan()
    elif use_cpp:
        tx_channels_list = [int(a.tx_channel)]
        sum_delay_us = 0.0
        sum_delay_native = 0
        sum_delay_random = None
        sum_delay_lo = 0
        sum_delay_hi = 0
        sum_tx_lead = 0
        sum_delay_seed = None
        sum_tx_samples = int(native.size)
        sum_freq_offset = 0.0
        sum_freq_actual = float(a.freq)
        sum_retune = 0
        sum_retune_fail = 0
        sum_delay_updates = 0
        sum_async = dict(_async_zero)
        jam_sweep = []
    else:
        tx_channels_list = list(echo.tx_channels)
        sum_delay_us = float(echo.jam_delay_us)
        sum_delay_native = int(echo.jam_delay_native)
        sum_delay_random = echo.jam_delay_random_us
        sum_delay_lo = int(echo.jam_delay_native_lo)
        sum_delay_hi = int(echo.jam_delay_native_hi)
        sum_tx_lead = int(echo.tx_lead_native)
        sum_delay_seed = (int(echo.jam_delay_seed)
                          if echo.jam_delay_random_us is not None else None)
        sum_tx_samples = int(echo._tx_payload.shape[-1])
        sum_freq_offset = float(echo.jam_freq_offset)
        sum_freq_actual = float(echo.jam_freq_actual)
        sum_retune = int(echo.jam_retune_count)
        sum_retune_fail = int(echo.jam_retune_fail)
        sum_delay_updates = 0
        sum_async = dict(_async_zero)
        jam_sweep = sorted(echo.jam_records, key=lambda r: r["pulse_id"])
    # Dual-TX geometry plan for the summary (§5.3 keys).  Computed from the
    # same inputs as the schedule PDU so python/cpp geometries agree; a
    # failure here must never break a completed python run, hence the
    # guarded fallback (the cpp path already fails fast at construction).
    try:
        if jam_enabled and jam_native is not None:
            tx_plan = jp.build_cpp_schedule_meta(
                sense_len=int(native.size),
                jam_len=int(jam_native.size),
                native_hz=float(profile.hz),
                delay_us=float(a.jam_delay_us),
                delay_random_us=jam_delay_random,
                delay_seed=int(a.jam_delay_seed or 0),
                jam_offsets_hz=list(jam_offsets),
                jam_dwell=int(a.jam_dwell),
                freq_settle_s=float(a.jam_freq_settle_s),
                jam_logical_channel=1)
        else:
            raise ValueError("single-TX")
    except ValueError:
        tx_plan = None
    if tx_plan is None:
        tx_plan = {
            "tx_channel_count": 1,
            "tx_samples": sum_tx_samples,
            "tx_waveform_samples": [int(native.size)],
            "tx_base_offsets_native": [0],
            "jam_logical_channel": 0,
            "jam_delay_mode": ("uniform"
                               if jam_delay_random is not None else "fixed"),
            "jam_delay_lo_native": sum_delay_lo,
            "jam_delay_hi_native": sum_delay_hi,
            "jam_delay_seed": 0,
            "jam_freq_offsets_hz": [],
            "jam_dwell": 0,
            "jam_freq_settle_ticks": 0,
        }
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
        "publish_native": echo.publish_native,
        "res_rx": res.pdus_received(),
        "res_tx": res.pdus_emitted(),
        "res_drop": res.pdus_dropped(),
        "est_rx": est.pdus_received(),
        "est_enqueued": est.pdus_enqueued(),
        "est_done": est.pdus_completed(),
        "est_fail": est.pdus_failed(),
        "est_drop": est.pdus_dropped(),
        "est_invalid": est.invalid_inputs(),
        "wr_ok": wr.frames_written(),
        "wr_fail": wr.frames_failed(),
        "wr_invalid": wr.frames_invalid(),
        "use_predicted_timing": use_pred,
        "est_queue_capacity": est_q,
        "est_service_us_mean": int(est.service_mean_us()),
        "est_service_us_max": int(est.service_max_us()),
        "udp_sent": 0 if udp is None else udp.sent,
        "udp_sent_ok": 0 if udp is None else udp.sent_ok,
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
        "cir_output": a.cir_output,
        "cir_skip_initial": cir_skip_initial,
        "cir_records_per_pulse": cir_records_per_pulse,
        "sfd_mode": base.SFD_MODE,
        "gain_tx": a.gain_tx,
        "gain_rx": a.gain_rx,
        "tx_channel": a.tx_channel,
        "rx_channel": a.rx_channel,
        "tx_channels": tx_channels_list,
        "echo_backend": a.echo_backend,
        "jam_enabled": jam_enabled,
        "jam_mode": a.jam_mode,
        "jam_channel": int(a.jam_channel),
        "jam_antenna": a.jam_antenna,
        "jam_gain_tx": a.jam_gain_tx,
        "jam_code_index": int(a.jam_code_index),
        "jam_preamble_length": int(jam_sync_reps),
        "jam_waveform": a.jam_waveform,
        "jam_scale": a.jam_scale,
        "jam_delay_us": sum_delay_us,
        "jam_delay_native": sum_delay_native,
        "jam_delay_random_us": sum_delay_random,
        "jam_delay_native_lo": sum_delay_lo,
        "jam_delay_native_hi": sum_delay_hi,
        "tx_lead_native": sum_tx_lead,
        "jam_delay_seed": sum_delay_seed,
        "jam_tx_samples": sum_tx_samples,
        "jam_samples_work": int(jam_work_samples),
        "jam_native_samples": int(0 if jam_native is None else jam_native.size),
        "jam_freq_offset_hz": sum_freq_offset,
        "jam_freq_actual_hz": sum_freq_actual,
        "jam_offsets_hz": jam_offsets,
        "jam_dwell": int(a.jam_dwell),
        "jam_retune": sum_retune,
        "jam_retune_fail": sum_retune_fail,
        "jam_sweep": jam_sweep,
        "tx_channel_count": int(tx_plan["tx_channel_count"]),
        "tx_samples_per_channel": int(tx_plan["tx_samples"]),
        "tx_waveform_samples": [int(v) for v in
                                tx_plan["tx_waveform_samples"]],
        "tx_base_offsets_native": [int(v) for v in
                                   tx_plan["tx_base_offsets_native"]],
        "jam_delay_mode": str(tx_plan["jam_delay_mode"]),
        "jam_freq_settle_ticks": int(tx_plan["jam_freq_settle_ticks"]),
        "cpp_delay_updates": sum_delay_updates,
        "cpp_jam_retune": sum_retune,
        "cpp_jam_retune_fail": sum_retune_fail,
        "cpp_tx_async": {k: int(v) for k, v in sum_async.items()},
    }
    with open(os.path.join(a.output, "summary.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")

    print("SUMMARY", json.dumps(summary), flush=True)
    print("radio_ok=%d cir_ok=%d cir_fail=%d est_drop=%d jam_retune=%d"
          % (summary["echo_ok"], cir_stats.get("ok", 0),
             cir_stats.get("fail", 0), summary["est_drop"],
             summary["jam_retune"]), flush=True)
    if os.path.isfile(jsonl):
        print("cir.jsonl_lines=%d" % cir_stats.get("lines", 0), flush=True)
    ok = (summary["wr_ok"] == a.pulses * cir_records_per_pulse
          and summary["echo_ok"] == a.pulses
          and summary["echo_late"] == 0)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
