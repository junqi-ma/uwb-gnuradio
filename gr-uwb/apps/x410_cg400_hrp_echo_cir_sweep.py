#!/usr/bin/env python3
"""X410 CG400 HRP echo CIR with runtime frequency tuning.

Built on the validated chain in ``x410_cg400_hrp_echo_cir.py`` (imported as
``base``).  Only the RF centre frequency is made dynamic; the TX waveform,
the 65/32 resampler and the 998.4 MS/s CIR estimator are unchanged.

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


class SweepTimedUhdEcho(base.TimedUhdEcho):
    """TimedUhdEcho that can retune between bursts and log per-pulse freq."""

    def __init__(self, *args, plan=None, freq_settle_s=0.05, **kwargs):
        super().__init__(*args, **kwargs)
        self.plan = plan
        self.nominal = float(self.freq)
        self.freq_settle_s = float(freq_settle_s)
        self.tx_freq_actual = float(self.freq)
        self.rx_freq_actual = float(self.freq)
        self.freq_records = []
        self.retune_count = 0
        self.retune_fail = 0

    def retune(self, freq_hz, next_pulse_id):
        """Shift TX+RX to freq_hz and re-arm the schedule at next_pulse_id."""
        freq_hz = float(freq_hz)
        if abs(freq_hz - self.freq) < 1.0:
            return False
        uhd = self._uhd
        self._usrp.set_tx_freq(uhd.types.TuneRequest(freq_hz), self.tx_ch)
        self._usrp.set_rx_freq(uhd.types.TuneRequest(freq_hz), self.rx_ch)
        self.tx_freq_actual = float(self._usrp.get_tx_freq(self.tx_ch))
        self.rx_freq_actual = float(self._usrp.get_rx_freq(self.rx_ch))
        self.freq = 0.5 * (self.tx_freq_actual + self.rx_freq_actual)
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
        ok = super()._one_burst(pulse_id)
        offset = self.freq - self.nominal
        dwell_index = (self.plan.dwell_index(pulse_id)
                       if self.plan is not None else 0)
        # base._one_burst always appends its timing record last.
        if self._timing:
            rec = self._timing[-1]
            rec["freq_hz"] = self.freq
            rec["freq_offset_hz"] = offset
            rec["tx_freq_actual"] = self.tx_freq_actual
            rec["rx_freq_actual"] = self.rx_freq_actual
            rec["dwell_index"] = dwell_index
        self.freq_records.append({
            "pulse_id": int(pulse_id),
            "freq_hz": self.freq,
            "freq_offset_hz": offset,
            "tx_freq_actual": self.tx_freq_actual,
            "rx_freq_actual": self.rx_freq_actual,
            "dwell_index": dwell_index,
        })
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


def parse_args():
    p = argparse.ArgumentParser(
        parents=[base.build_parser(add_help=False)],
        description="X410 CG400 HRP echo CIR with runtime frequency tuning")
    p.add_argument("--freq-mode", choices=["fixed", "scan", "manual"],
                   default="fixed",
                   help="fixed = base app; scan = step sweep; manual = stdin")
    p.add_argument("--freq-start", type=float, default=None,
                   help="scan start in Hz (default: --freq)")
    p.add_argument("--freq-stop", type=float, default=None,
                   help="scan stop in Hz (single sweep, inclusive)")
    p.add_argument("--freq-step", type=float, default=0.0,
                   help="scan step in Hz (>0)")
    p.add_argument("--freq-dwell", type=int, default=20,
                   help="pulses per scan frequency (>=1)")
    p.add_argument("--freq-scan", choices=["once", "cycle"], default="once",
                   help="scan once then stop, or cycle until pulses")
    p.add_argument("--freq-settle-s", type=float, default=0.05,
                   help="delay after each retune before the next timed burst")
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
        once=(a.freq_scan == "once"), manual_q=manual_q)


def main():
    base.bootstrap_uhd_env()
    a = parse_args()
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
    elif a.freq_mode == "manual" and a.pulses <= 0:
        a.pulses = 1 << 31
        print("[freq] manual: run until 'q'", flush=True)

    print("[freq] mode=%s nominal=%.6fMHz points=%d dwell=%d total_pulses=%d"
          % (a.freq_mode, a.freq / 1e6, plan.n_points, plan.dwell, a.pulses),
          flush=True)
    if a.freq_mode == "scan":
        print("[freq] sweep %.3f -> %.3f MHz step %.3f MHz"
              % (plan.freqs[0] / 1e6, plan.freqs[-1] / 1e6,
                 a.freq_step / 1e6), flush=True)
        for i, f in enumerate(plan.freqs):
            print("[freq]   #%03d %.6f MHz (offset %+.3f MHz)"
                  % (i, f / 1e6, (f - a.freq) / 1e6), flush=True)

    if a.dry_run:
        print("[freq] dry-run complete", flush=True)
        raise SystemExit(0)

    repo = base.find_repo_root()
    os.makedirs(a.output, exist_ok=True)
    taps = a.taps or os.path.join(
        repo, "testdata", "resampler_65_32", "taps_quality_minorder.txt")
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
    native = base.resample_poly(samples.astype(np.complex128),
                                32, 65).astype(np.complex64)
    print("hrp_tx_998p4_samples=%d native_491p52=%d resample_ms=%.2f "
          "insert_sts=%s pulse_shape=%s pulse_taps=%d pulse_center=%d "
          "code_index=%d preamble_length=%d sfd_mode=%s"
          % (samples.size, native.size, (time.perf_counter() - t_rs) * 1e3,
             insert_sts, src.pulse_shape(), src.pulse_taps(),
             src.pulse_center_taps(), a.code_index, a.sync_reps,
             base.SFD_MODE),
          flush=True)
    print("schedule pulses=%d pri_s=%.6f rate_hz=%.3f duration_s=%.3f" % (
        a.pulses, a.pri_s, (1.0 / a.pri_s), a.pulses * a.pri_s), flush=True)

    dump_dir = os.path.join(a.output, "rx_iq") if a.dump_rx else ""
    sc16_dir = a.output if a.dump_sc16 else ""
    echo = SweepTimedUhdEcho(
        a.args, base.CG400_HZ, a.freq, a.tx_channel, a.rx_channel,
        a.tx_antenna, a.rx_antenna, a.gain_tx, a.gain_rx,
        a.pre_guard_us, 15.0, a.tail_guard_us, a.sync_reps,
        a.cal_delay_native, a.arm_delay_s, a.pri_s, a.pulses, dump_dir,
        a.min_lead_s, timing_path, sc16_dir, a.rx_pad_us, a.code_index,
        base.SFD_MODE, plan=plan, freq_settle_s=a.freq_settle_s)
    echo.set_tx_native(native)
    print("uhd_probe", echo.status, flush=True)
    if a.dump_sc16:
        tx_sc16_path = os.path.join(a.output, "tx_491p52.sc16")
        base.fc32_to_sc16(echo._native).tofile(tx_sc16_path)
        print("wrote_tx_sc16", tx_sc16_path, "samples=%d" % echo._native.size,
              flush=True)

    res = base.uwb.pdu_rational_resampler_ccf_65_32(
        taps, 998.4e6, True, 2097152)
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
        udp = base.CirUdpSink(a.udp_host, int(a.udp_port), base.CIR_UDP_TAPS)
        print("udp_cir %s:%s framed=UCR1 always_send_taps=%d nonblock" % (
            a.udp_host, a.udp_port, base.CIR_UDP_TAPS), flush=True)

    tb = base.gr.top_block("x410_cg400_hrp_echo_cir_sweep")
    tb.msg_connect((echo, "rx"), (res, "packet"))
    tb.msg_connect((res, "packet"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if udp is not None:
        tb.msg_connect((est, "cir"), (udp, "cir"))

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
        "freq_mode": a.freq_mode,
        "freq_nominal_hz": a.freq,
        "freq_start_hz": plan.start,
        "freq_stop_hz": plan.stop,
        "freq_step_hz": a.freq_step,
        "freq_dwell": a.freq_dwell,
        "freq_scan": a.freq_scan,
        "freq_settle_s": a.freq_settle_s,
        "freq_points": plan.n_points,
        "freq_plan_hz": plan.freqs,
        "freq_retune_count": echo.retune_count,
        "freq_retune_fail": echo.retune_fail,
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
        "iq_scale": base.IQ_SCALE,
        "sample_format": "sc16",
        "rx_window": echo.rx_len,
        "rx_window_us": echo.rx_len / base.CG400_HZ * 1e6,
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
                    "X410 CG400 monostatic HRP echo, native SC16 RX windows, "
                    "runtime frequency sweep"
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
                "rate_native_hz": base.CG400_HZ,
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
                "rx_window_us": echo.rx_len / base.CG400_HZ * 1e6,
                "files": {
                    "capture.iq": "concatenated native SC16 packets",
                    "capture.jsonl": "one JSON object per packet (has freq)",
                    "freq_sweep.jsonl": "one JSON object per packet (freq)",
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
          "udp_ok=%d udp_fail=%d retune=%d retune_fail=%d "
          "(radio ok is not CIR/UDP ok)" % (
              summary["echo_ok"], cir_stats.get("ok", 0),
              cir_stats.get("fail", 0), summary["est_drop"],
              summary["udp_sent"], summary["udp_sent_ok"],
              summary["udp_sent_fail"], echo.retune_count,
              echo.retune_fail),
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
