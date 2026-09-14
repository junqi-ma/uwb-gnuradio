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

Why the python backend only: the C++ ``UwbRealtimeEchoTimer`` owns a single
TX streamer and couples TX to the schedule PDU geometry; a second TX channel
there means changing the core scheduler + backend + bindings.  This app is a
fixed/low-rate research tool, so it forces ``--echo-backend python`` (which
also provides ``--dump-sc16`` for offline analysis of the composite RX).

Planning/geometry math lives in ``echo_cir_jam_plan`` (pure Python, unit
tested); this file only wires it to UHD and the existing CIR chain.

Example::

  python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py \\
    --args addr=192.168.10.2 --jam-enable --jam-channel 1 \\
    --preamble-length 128 --gain-tx 50 --gain-rx 60 --no-udp \\
    --jam-code-index 10 --jam-preamble-length 128 --jam-scale 0.3 \\
    --jam-freq-offsets 0,245.67e3,491.34e3 --jam-dwell 50 \\
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
                 jam_dwell=0, freq_settle_s=0.05):
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
        self.jam_delay_native = jp.placement_native(self.jam_delay_us, self.rate)
        jam_len = 0 if jam_native is None else int(jam_native.size)
        self.jam_payload_native = jam_len
        length = jp.combined_tx_len(int(np.asarray(sense_native).size),
                                    jam_len, self.jam_delay_native)

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
        self._tx_payload = jp.compose_tx_native(
            self._native, jam_scaled, self.jam_delay_native)

        self.status["tx_geometry_native"] = int(length)
        self.status["jam_delay_us"] = self.jam_delay_us
        self.status["jam_delay_native"] = self.jam_delay_native
        self.status["jam_native"] = jam_len
        self.status["jam_tx_samples"] = int(self._tx_payload.shape[1])
        self.status["jam_peak_scale"] = self.jam_scale
        return self._tx_payload

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
            return
        self.jam_freq_actual = float(self._usrp.get_tx_freq(self.jam_ch))
        self.jam_freq_offset = float(offset_hz)
        self.jam_freq_hz = self.freq + self.jam_freq_offset
        now = self._usrp.get_time_now().get_real_secs()
        self._t0 = now + self.freq_settle_s - pulse_id * self.pri_s
        self.jam_retune_count += 1

    def _record_jam(self, pulse_id):
        rec = {
            "pulse_id": int(pulse_id),
            "jam_freq_hz": self.jam_freq_hz,
            "jam_freq_offset_hz": self.jam_freq_offset,
            "jam_freq_actual_hz": self.jam_freq_actual,
            "jam_freq_actual_offset_hz": self.jam_freq_actual - self.freq,
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
            for pulse_id in range(self.max_pulses):
                target = self._jam_target_offset(pulse_id)
                if target is not None \
                        and abs(target - self.jam_freq_offset) >= 1.0:
                    self._retune_jam(target, pulse_id)
                self._one_burst(pulse_id)
                if self.jam_enable:
                    self._record_jam(pulse_id)
                    self._prepare_timing_record(pulse_id)
                self._done = pulse_id + 1
                if (pulse_id == 0 or (pulse_id + 1) % 100 == 0
                        or pulse_id + 1 == self.max_pulses):
                    dt = time.perf_counter() - t_host0
                    print("sched %d/%d ok=%d fail=%d late=%d sc16=%d "
                          "host_s=%.3f jam_off=%+.6fMHz retune=%d"
                          % (pulse_id + 1, self.max_pulses, self._ok,
                             self._fail, self._late, self._sc16_written, dt,
                             self.jam_freq_offset / 1e6,
                             self.jam_retune_count), flush=True)
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
        rec["jam_retune"] = self.jam_retune_count


def parse_args():
    p = argparse.ArgumentParser(
        parents=[base.build_parser(add_help=False)],
        description="X410 dual-TX HRP jammer for UWB sensing CIR: a second "
                    "TX channel carries a different-preamble-code HRP signal "
                    "at an exact NCO frequency offset (python backend only).")
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
                        "\"0,245.67e3,491.34e3\"; overrides --jam-freq-offset")
    g.add_argument("--jam-dwell", type=int, default=50,
                   help="pulses per offset when --jam-freq-offsets is used")
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
                        "the sensing burst start")
    g.add_argument("--jam-scale", type=float, default=0.3,
                   help="jammer peak amplitude / sensing TX peak (the sensing "
                        "is normalised to 0.8 full scale)")
    g.add_argument("--jam-repeat-pri-us", type=float, default=0.0,
                   help="continuous-mode jammer repeat interval in us")
    g.add_argument("--jam-no-sts", action="store_true")
    g.add_argument("--jam-freq-settle-s", type=float, default=0.05,
                   help="settle time after each jammer retune")
    g.add_argument("--dry-run", action="store_true",
                   help="print the jam plan and exit without touching UHD")
    return p.parse_args()


def main():
    base.bootstrap_uhd_env()
    a = parse_args()
    if a.echo_backend == "cpp-pdu":
        raise SystemExit(
            "--echo-backend cpp-pdu has a single TX streamer and cannot carry "
            "the jammer.\nUse the python backend (default for this app): "
            "drop --echo-backend, or pass --echo-backend python.")
    if a.echo_backend is None:
        a.echo_backend = "python"

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
    if jam_offsets:
        if a.jam_dwell <= 0:
            raise SystemExit("--jam-freq-offsets needs --jam-dwell >= 1")
        a.pulses = len(jam_offsets) * int(a.jam_dwell)
        print("[jam] offset scan: %d offsets x dwell %d -> %d pulses"
              % (len(jam_offsets), a.jam_dwell, a.pulses), flush=True)
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
        print("[jam] mode=%s channel=%d ant=%s gain=%.1f freq=%.6f MHz "
              "offset=%+.6f kHz scale=%.3f delay=%.3f us"
              % (a.jam_mode, a.jam_channel, a.jam_antenna, a.jam_gain_tx,
                 (a.freq + off0) / 1e6, off0 / 1e3, a.jam_scale,
                 a.jam_delay_us), flush=True)
        if jam_offsets:
            print("[jam] offsets(kHz)=%s"
                  % ",".join("%+.3f" % (o / 1e3) for o in jam_offsets),
                  flush=True)
    else:
        print("jam_enabled=False: single-TX sensing only", flush=True)

    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f"
          % (a.pulses, a.pri_s, 1.0 / a.pri_s, a.pulses * a.pri_s), flush=True)

    if a.dry_run:
        delay_native = jp.placement_native(a.jam_delay_us, profile.hz)
        jam_len = 0 if jam_native is None else int(jam_native.size)
        length = jp.combined_tx_len(int(native.size), jam_len, delay_native)
        print("[jam] dry-run: sense_native=%d jam_native=%d delay_native=%d "
              "composite_len=%d (tx_channels=%s)"
              % (native.size, jam_len, delay_native, length,
                 [a.tx_channel, a.jam_channel] if jam_enabled
                 else [a.tx_channel]), flush=True)
        print("[jam] dry-run complete", flush=True)
        raise SystemExit(0)

    base.dpdk_preflight(a)

    if a.require_sfd and a.publish_native < 0:
        print("WARN --require-sfd with --publish-native -1 (auto ROI): "
              "falling back to full window (0)", flush=True)
        a.publish_native = 0
    dump_dir = os.path.join(a.output, "rx_iq") if a.dump_rx else ""
    sc16_dir = a.output if a.dump_sc16 else ""

    off0 = jam_offsets[0] if jam_offsets else a.jam_freq_offset
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
        jam_dwell=a.jam_dwell, freq_settle_s=a.jam_freq_settle_s)
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
    else:  # python backend -> raw integer like the legacy chain
        sc16_scale = base.uwb.Sc16ScalePolicy.RawInteger
    res = profile.make_pdu_resampler(taps, a.res_workers, sc16_scale)
    est_q = max(8, int(a.est_queue))
    use_pred = not a.require_sfd
    est = base.uwb.radar_cir_estimator(
        tmpl_path, a.sync_reps, base.SFD_MODE, a.code_index, 16, 100, 10, 0,
        a.sfd_search_margin, a.sync_refine_margin, a.sfd_threshold,
        a.sync_refine_threshold, True, est_q, use_pred)
    wr = base.uwb.cir_writer(a.output, "cir", True, 64)
    udp = None
    if (not a.no_udp) and bool(a.udp_host):
        udp = base.CirUdpSink(
            a.udp_host, int(a.udp_port), base.CIR_UDP_TAPS,
            freq_lookup=lambda pid: echo.jam_by_pulse.get(
                int(pid), (echo.jam_freq_hz, echo.jam_freq_offset)))
        print("udp_cir %s:%s framed=UCR2 jam_freq+freq_offset from "
              "jam_by_pulse" % (a.udp_host, a.udp_port), flush=True)

    tb = gr.top_block("x410_cg400_hrp_echo_cir_jam")
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
    cir_stats = base.analyze_cir(jsonl, a.pulses)
    timing_stats = base.analyze_timing(timing_path)
    jam_sweep = sorted(echo.jam_records, key=lambda r: r["pulse_id"])
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
        "sfd_mode": base.SFD_MODE,
        "gain_tx": a.gain_tx,
        "gain_rx": a.gain_rx,
        "tx_channel": a.tx_channel,
        "rx_channel": a.rx_channel,
        "tx_channels": echo.tx_channels,
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
        "jam_delay_us": a.jam_delay_us,
        "jam_delay_native": echo.jam_delay_native,
        "jam_tx_samples": int(echo._tx_payload.shape[-1]),
        "jam_samples_work": int(jam_work_samples),
        "jam_native_samples": int(0 if jam_native is None else jam_native.size),
        "jam_freq_offset_hz": echo.jam_freq_offset,
        "jam_freq_actual_hz": echo.jam_freq_actual,
        "jam_offsets_hz": jam_offsets,
        "jam_dwell": int(a.jam_dwell),
        "jam_retune": echo.jam_retune_count,
        "jam_retune_fail": echo.jam_retune_fail,
        "jam_sweep": jam_sweep,
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
    ok = (summary["wr_ok"] == a.pulses and summary["echo_ok"] == a.pulses
          and summary["echo_late"] == 0)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
