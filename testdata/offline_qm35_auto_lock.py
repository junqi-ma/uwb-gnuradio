#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Blind QM35 acquire + lock on a native 737.28 MS/s SC16 capture.

No first_packet_sample / t0.  Flow:

  file_source SC16
    -> UwbAutoScheduledExtractorSc16   (energy + code-9 verify, then 5 ms schedule)
    -> UwbPduRationalResamplerCcf65_48 (737.28 -> 998.4)
    -> UwbRealtimeDemodulator          (code 9 / 64 SYNC / 4z2 / bypass)
    -> schedule_feedback -> extractor.lock_obs

Default input is the 0.5 s mixed DW1000+QM35 X410 capture.
Pass the no-interference file to run the same flow on clean QM35:

  /mnt/f/UWB基带数据/qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import sys
import time

import numpy as np
from gnuradio import blocks, gr
import pmt as p

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))

DEFAULT_DAT = (
    "/mnt/f/UWB基带数据/"
    "dw1000_qm35_mixed_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat"
)
TMPL_737 = os.path.join(HERE, "reference_preamble_code9_737p28.cf32")
TMPL_998 = os.path.join(HERE, "reference_preamble.bin")
TAPS_MIN = os.path.join(HERE, "resampler_65_48", "taps_quality_minorder.txt")

FS737 = 737.28e6
FS998 = 998.4e6
PERIOD_S = 0.005
PRE = 7373
CAP = 140083
POST = 3023
# Native dump: full DW1000 airtime before QM35, short tail after the body.
# CIR is estimated on the preamble, so a late-only interferer is not kept.
PRE_DW = 221184   # 300 µs
POST_DW = 73728   # 100 µs
ACQ_PRE = 2032
ACQ_CAP = 200000


def as_int(v, default=-1):
    for f in (p.to_long, p.to_uint64):
        try:
            return int(f(v))
        except Exception:
            pass
    return default


def as_num(v, default=float("nan")):
    for f in (p.to_double, p.to_long, p.to_uint64):
        try:
            return float(f(v))
        except Exception:
            pass
    return default


def as_str(v, default=""):
    try:
        if p.is_symbol(v):
            return p.symbol_to_string(v)
        if p.is_bool(v):
            return "true" if p.to_bool(v) else "false"
    except Exception:
        pass
    return default


def dict_get(md, key, default=None):
    return p.dict_ref(md, p.intern(key),
                      default if default is not None else p.PMT_NIL)


def parse_result(msg):
    md = p.car(msg) if p.is_pair(msg) else msg
    status = as_str(dict_get(md, "status", p.intern("?")))
    return {
        "packet_id": as_int(dict_get(md, "packet_id", p.from_uint64(0)), 0),
        "status": status,
        "fcs": bool(p.to_bool(dict_get(md, "fcs_pass", p.from_bool(False)))),
        "det_start": as_int(dict_get(md, "detected_start_sample",
                                     p.from_long(-1))),
        "pred_start": as_int(dict_get(md, "predicted_start_sample",
                                      p.from_long(-1))),
        "win_start": as_int(dict_get(md, "window_start_sample",
                                     p.from_long(-1))),
        "timing_metric": as_num(dict_get(md, "timing_metric",
                                         p.from_double(-1))),
        "timing_peaks": as_int(dict_get(md, "timing_peaks", p.from_uint64(0)),
                               0),
        "cfo_hz": as_num(dict_get(md, "cfo_hz", p.from_double(0))),
        "sfd_metric": as_num(dict_get(md, "sfd_metric", p.from_double(-1))),
        "payload_n": as_int(dict_get(md, "payload_nbytes", p.from_long(-1))),
        "t_total_us": as_num(dict_get(md, "stage_total_us", p.from_double(-1))),
        "t_timing_us": as_num(dict_get(md, "stage_timing_us", p.from_double(-1))),
        "t_timing_coarse_us": as_num(dict_get(md, "stage_timing_coarse_us",
                                              p.from_double(-1))),
        "t_timing_fine_us": as_num(dict_get(md, "stage_timing_fine_track_us",
                                            p.from_double(-1))),
        "t_cfo_us": as_num(dict_get(md, "stage_cfo_us", p.from_double(-1))),
        "t_sfd_us": as_num(dict_get(md, "stage_sfd_us", p.from_double(-1))),
        "t_cir_us": as_num(dict_get(md, "stage_cir_us", p.from_double(-1))),
        "t_payload_us": as_num(dict_get(md, "stage_payload_us",
                                        p.from_double(-1))),
        "t_resample_us": as_num(dict_get(md, "resample_us", p.from_double(-1))),
        "t_queue_us": as_num(dict_get(md, "queue_delay_us", p.from_double(-1))),
        "t_demod_lat_us": as_num(dict_get(md, "demod_latency_us",
                                          p.from_double(-1))),
        "capture_mode": as_str(dict_get(md, "capture_mode", p.intern(""))),
        "sample_rate": as_num(dict_get(md, "sample_rate", p.from_double(0))),
        "native_rate": as_num(dict_get(md, "native_sample_rate",
                                       p.from_double(0))),
    }


def parse_status(msg):
    md = msg
    if p.is_dict(md):
        pass
    elif p.is_pair(md) and p.is_dict(p.car(md)):
        md = p.car(md)
    elif p.is_pair(md) and p.is_dict(p.cdr(md)):
        md = p.cdr(md)
    else:
        return {"event": "?", "lock_state": "?", "abs_sample": -1, "reason": ""}
    return {
        "event": as_str(dict_get(md, "event", p.intern("?"))),
        "lock_state": as_str(dict_get(md, "lock_state", p.intern("?"))),
        "abs_sample": as_int(dict_get(md, "abs_sample", p.from_uint64(0)), -1),
        "reason": as_str(dict_get(md, "reason", p.intern(""))),
    }


def parse_packet_meta(msg):
    md = p.car(msg) if p.is_pair(msg) else msg
    if not p.is_dict(md):
        return {}
    return {
        "packet_id": as_int(dict_get(md, "packet_id", p.from_uint64(0)), 0),
        "capture_mode": as_str(dict_get(md, "capture_mode", p.intern(""))),
        "start_sample": as_int(dict_get(md, "start_sample", p.from_long(-1))),
        "predicted_start_sample": as_int(
            dict_get(md, "predicted_start_sample", p.from_long(-1))),
        "lock_state": as_str(dict_get(md, "lock_state", p.intern(""))),
        "detection_metric": as_num(dict_get(md, "detection_metric",
                                            p.from_double(-1))),
        "sample_count": as_int(dict_get(md, "sample_count", p.from_long(-1))),
        "schedule_index": as_int(dict_get(md, "schedule_index",
                                          p.from_uint64(0)), -1),
        "window_start_sample": as_int(
            dict_get(md, "window_start_sample", p.from_long(-1))),
        "pre_guard_samples": as_int(
            dict_get(md, "pre_guard_samples", p.from_long(-1))),
        "capture_samples": as_int(
            dict_get(md, "capture_samples", p.from_long(-1))),
        "post_guard_samples": as_int(
            dict_get(md, "post_guard_samples", p.from_long(-1))),
    }


def parser():
    ap = argparse.ArgumentParser()
    ap.add_argument("dat", nargs="?", default=DEFAULT_DAT)
    ap.add_argument("--max-seconds", type=float, default=0.08,
                    help="stream prefix to process; 0 = whole file")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--energy-threshold", type=float, default=0.001)
    ap.add_argument("--lock-observations", type=int, default=3)
    ap.add_argument("--cir-filter-mode", default="bypass")
    ap.add_argument(
        "--write-sc16", default="", metavar="DIR",
        help="after lock, write native SC16 windows to DIR/capture.iq "
             "(uses 300 µs head / 100 µs tail unless --pre/--post override)")
    ap.add_argument("--pre", type=int, default=0,
                    help="scheduled pre-guard samples; 0 = default "
                         "(7373, or 221184 with --write-sc16)")
    ap.add_argument("--post", type=int, default=0,
                    help="scheduled post-guard samples; 0 = default "
                         "(3023, or 73728 with --write-sc16)")
    ap.add_argument("--capture", type=int, default=CAP,
                    help="scheduled QM35 body samples")
    ap.add_argument("--no-notch-tone", action="store_true",
                    help="do not notch the ~6200 MHz CW after --write-sc16")
    return ap


def main():
    args = parser().parse_args()
    dat = args.dat
    if not os.path.isfile(dat):
        print(f"ERROR: file not found: {dat}", file=sys.stderr)
        return 2
    if not os.path.isfile(TMPL_737):
        print(f"ERROR: missing native template {TMPL_737}", file=sys.stderr)
        return 2
    if not os.path.isfile(TMPL_998):
        print(f"ERROR: missing 998.4 template {TMPL_998}", file=sys.stderr)
        return 2

    from gnuradio import uwb

    nbytes = os.path.getsize(dat)
    n_complex = nbytes // 4
    duration_s = n_complex / FS737
    n_stream = n_complex
    if args.max_seconds and args.max_seconds > 0:
        n_stream = min(n_complex, int(round(args.max_seconds * FS737)))
    stream_s = n_stream / FS737

    write_dir = args.write_sc16.strip()
    pre = int(args.pre) if args.pre > 0 else (PRE_DW if write_dir else PRE)
    post = int(args.post) if args.post > 0 else (POST_DW if write_dir else POST)
    cap = int(args.capture)

    tmpl737 = np.fromfile(TMPL_737, np.complex64)
    tmpl998 = np.fromfile(TMPL_998, np.complex64)
    print("=== Offline auto-lock: QM35 on native SC16 (no t0) ===")
    print(f"file: {dat}")
    print(f"file_samples={n_complex} file_s={duration_s:.4f} "
          f"stream_samples={n_stream} stream_s={stream_s:.4f}")
    print("first_packet_sample: NOT PROVIDED")
    print(f"period={PERIOD_S*1e3:.1f} ms  "
          f"window=[{pre},{cap},{post}]  "
          f"acquire=[{ACQ_PRE},{ACQ_CAP}]")
    if write_dir:
        print(f"write_sc16={write_dir}  "
              f"head={pre/FS737*1e6:.1f} us  "
              f"body={cap/FS737*1e6:.1f} us  "
              f"tail={post/FS737*1e6:.1f} us")
    print(f"template_737={len(tmpl737)}  template_998={len(tmpl998)}  "
          f"energy_th={args.energy_threshold}")

    ext = uwb.auto_scheduled_extractor_sc16(
        [complex(x) for x in tmpl737],
        FS737,
        PERIOD_S,
        pre,
        cap,
        post,
        float(args.energy_threshold),
        100,
        4,
        1,
        16,
        int(args.lock_observations),
        3,
        8,
        25.0,
        ACQ_PRE,
        ACQ_CAP,
        8,
    )
    resampler = uwb.pdu_rational_resampler_ccf_65_48("quality_minorder")
    demod = uwb.realtime_demodulator.make_from_template(
        tmpl998.tolist(),
        max(1, args.workers),
        64,
        "4z2",
        0,
        args.cir_filter_mode,
        9,
        64,
        14,
    )
    dbg_pkt = blocks.message_debug()
    dbg_res = blocks.message_debug()
    dbg_st = blocks.message_debug()
    status_wall = []

    class StatusClock(gr.sync_block):
        def __init__(self, t_origin):
            gr.sync_block.__init__(
                self, name="status_clock", in_sig=None, out_sig=None)
            self.t_origin = t_origin
            self.message_port_register_in(p.intern("status"))
            self.set_msg_handler(p.intern("status"), self._on)

        def _on(self, msg):
            s = parse_status(msg)
            s["wall_ms"] = (time.time() - self.t_origin) * 1e3
            status_wall.append(s)

    src = blocks.file_source(2 * gr.sizeof_short, dat, False)
    head = blocks.head(2 * gr.sizeof_short, n_stream)
    tb = gr.top_block("offline_qm35_auto_lock")
    clock = StatusClock(time.time())
    writer = None
    tb.connect(src, head, ext)
    tb.msg_connect(ext, "packet", resampler, "packet")
    tb.msg_connect(resampler, "packet", demod, "samples")
    tb.msg_connect(demod, "result", dbg_res, "store")
    tb.msg_connect(demod, "schedule_feedback", ext, "lock_obs")
    tb.msg_connect(ext, "packet", dbg_pkt, "store")
    tb.msg_connect(ext, "status", dbg_st, "store")
    tb.msg_connect(ext, "status", clock, "status")
    if write_dir:
        os.makedirs(write_dir, exist_ok=True)
        writer = uwb.packet_writer(write_dir, "capture", False)
        tb.msg_connect(ext, "packet", writer, "packet")

    print("starting flowgraph (no t0 seed)...", flush=True)
    t0 = time.time()
    clock.t_origin = t0
    tb.start()
    deadline = time.time() + max(180.0, stream_s * 40.0 + 60.0)
    last_print = 0.0
    idle_since = None
    prev = None
    locked_at = None
    last_progress_wall = None
    last_ext_wall = None
    prev_ext = None
    while time.time() < deadline:
        state = ext.lock_state_name()
        ident = ext.identity_confirmed()
        snap = (
            state, ident, ext.energy_regions(), ext.candidates_emitted(),
            ext.emitted_windows(), demod.jobs_received(),
            demod.jobs_completed(), demod.jobs_failed(),
            dbg_res.num_messages(),
        )
        if snap != prev:
            prev = snap
            idle_since = time.time()
            last_progress_wall = idle_since - t0
            if ident and locked_at is None:
                locked_at = last_progress_wall
        ext_snap = (ext.energy_regions(), ext.scheduled_windows(),
                    ext.emitted_windows())
        if ext_snap != prev_ext:
            prev_ext = ext_snap
            last_ext_wall = time.time() - t0
        if time.time() - last_print > 2.0:
            print(
                f"  ... state={state} ident={ident} "
                f"t0={ext.locked_t0():.1f} "
                f"energy={ext.energy_regions()} "
                f"cand={ext.candidates_emitted()} "
                f"rej={ext.candidates_rejected()} "
                f"sched={ext.scheduled_windows()} "
                f"emit={ext.emitted_windows()} "
                f"demod_rx={demod.jobs_received()} "
                f"comp={demod.jobs_completed()} "
                f"fail={demod.jobs_failed()} "
                f"drop={demod.jobs_dropped()} "
                f"wall={time.time()-t0:.1f}s",
                flush=True,
            )
            last_print = time.time()
        # After identity, wait until scheduled windows + demod drain, or idle.
        in_flight = max(0, demod.jobs_received() - (
            demod.jobs_completed() + demod.jobs_failed() + demod.jobs_dropped()))
        if ident and state in ("locked", "provisional") and in_flight == 0 \
                and idle_since is not None and (time.time() - idle_since) >= 1.5:
            break
        # File consumed and everything idle even if never locked.
        if in_flight == 0 and idle_since is not None \
                and (time.time() - idle_since) >= 4.0:
            print(f"  ... idle-stop state={state} ident={ident}")
            break
        time.sleep(0.05)
    try:
        demod.drain()
    except Exception:
        pass
    time.sleep(0.3)
    tb.stop()
    tb.wait()
    wall = time.time() - t0

    packets = [parse_packet_meta(dbg_pkt.get_message(i))
               for i in range(dbg_pkt.num_messages())]
    results = [parse_result(dbg_res.get_message(i))
               for i in range(dbg_res.num_messages())]
    statuses = [parse_status(dbg_st.get_message(i))
                for i in range(dbg_st.num_messages())]
    events = [s["event"] for s in statuses]
    event_hist = collections.Counter(events)
    mode_hist = collections.Counter(x.get("capture_mode", "") for x in packets)
    res_hist = collections.Counter(r["status"] for r in results)
    fcs_ok = sum(1 for r in results if r["fcs"])
    acq = [x for x in packets if x.get("capture_mode") == "acquisition"]
    sched = [x for x in packets if x.get("capture_mode") in
             ("provisional", "scheduled")]

    print()
    print("=== Auto-lock outcome ===")
    print(f"wall_s={wall:.2f}")
    print(f"lock_state={ext.lock_state_name()}")
    print(f"identity_confirmed={ext.identity_confirmed()}")
    print(f"locked_t0_native={ext.locked_t0():.3f} samples "
          f"({ext.locked_t0()/FS737*1e3:.3f} ms)")
    print(f"locked_period_s={ext.locked_period_s():.9f} "
          f"({ext.locked_period_s()*FS737:.3f} native samples)")
    print(f"identity_wall_s={locked_at if locked_at is not None else -1}")
    print(f"energy_regions={ext.energy_regions()} "
          f"after_lock={ext.energy_regions_after_lock()}")
    print(f"candidates_emitted={ext.candidates_emitted()} "
          f"rejected={ext.candidates_rejected()}")
    print(f"scheduled_windows={ext.scheduled_windows()} "
          f"emitted={ext.emitted_windows()} "
          f"dropped={ext.dropped_windows()}")
    print(f"stale_feedback={ext.stale_feedback()} "
          f"unmapped={ext.unmapped_feedback()} "
          f"disc={ext.discontinuities()}")
    print(f"resampler emitted={resampler.pdus_emitted()} "
          f"dropped={resampler.pdus_dropped()}")
    print(f"demod rx={demod.jobs_received()} "
          f"completed={demod.jobs_completed()} "
          f"failed={demod.jobs_failed()} "
          f"dropped={demod.jobs_dropped()}")
    print(f"packet_mode_hist={dict(mode_hist)}")
    print(f"status_event_hist={dict(event_hist)}")
    print(f"demod_status_hist={dict(res_hist)}  "
          f"fcs_pass={fcs_ok}/{len(results)}")

    print()
    print("=== Status timeline (first 30) ===")
    for s in statuses[:30]:
        extra = f" reason={s['reason']}" if s["reason"] else ""
        print(f"  abs={s['abs_sample']:>12}  {s['event']:<28}  "
              f"{s['lock_state']}{extra}")
    if len(statuses) > 30:
        print(f"  ... {len(statuses)-30} more status messages")

    print()
    print("=== Acquisition PDUs ===")
    if not acq:
        print("  (none)")
    for x in acq[:20]:
        print(f"  id={x['packet_id']} start={x['start_sample']} "
              f"metric={x['detection_metric']:.3f} "
              f"n={x['sample_count']} lock={x['lock_state']}")
    if len(acq) > 20:
        print(f"  ... {len(acq)-20} more")

    print()
    print("=== First / last scheduled PDUs ===")
    if not sched:
        print("  (none)")
    else:
        show = sched[:8] + (sched[-4:] if len(sched) > 8 else [])
        seen = set()
        for x in show:
            key = (x["packet_id"], x["predicted_start_sample"])
            if key in seen:
                continue
            seen.add(key)
            print(f"  id={x['packet_id']} mode={x['capture_mode']} "
                  f"k={x['schedule_index']} pred={x['predicted_start_sample']} "
                  f"win={x.get('window_start_sample', -1)} "
                  f"pre/cap/post="
                  f"{x.get('pre_guard_samples', -1)}/"
                  f"{x.get('capture_samples', -1)}/"
                  f"{x.get('post_guard_samples', -1)} "
                  f"n={x['sample_count']}")
        print(f"  scheduled_pdu_count={len(sched)}")

    print()
    print("=== Demod results ===")
    if not results:
        print("  (none)")
    for r in results[:24]:
        dpred = (r["det_start"] - r["pred_start"]
                 if r["det_start"] >= 0 and r["pred_start"] >= 0 else None)
        print(f"  id={r['packet_id']:<4} {r['status']:<16} "
              f"fcs={int(r['fcs'])} peaks={r['timing_peaks']:<3} "
              f"tmet={r['timing_metric']:6.3f} "
              f"cfo={r['cfo_hz']:8.1f} "
              f"det={r['det_start']} "
              f"dpred={(dpred if dpred is not None else -999999):+d} "
              f"mode={r['capture_mode']}")
    if len(results) > 24:
        print(f"  ... {len(results)-24} more")

    print()
    print("=== Lock-time analysis ===")
    print("rf_ms = abs_sample / 737.28e6; wall_ms from status handler")
    ev_order = ("acquisition_started", "candidate_emitted",
                "qm35_identity_confirmed", "provisional_schedule_started",
                "schedule_locked")
    prev_wall = 0.0
    for ev in ev_order:
        hits = [s for s in status_wall if s["event"] == ev] or \
               [s for s in statuses if s["event"] == ev]
        if not hits:
            print(f"  {ev}: missing")
            continue
        s = hits[0]
        rf_ms = (s["abs_sample"] / FS737) * 1e3 if s["abs_sample"] >= 0 else -1
        w = s.get("wall_ms", float("nan"))
        dw = (w - prev_wall) if w == w else float("nan")
        if w == w:
            prev_wall = w
        print(f"  {ev:<28} abs={s['abs_sample']:>12}  "
              f"rf={rf_ms:7.3f} ms  wall={w:7.2f} ms  dwall={dw:7.2f} ms")
    if locked_at is not None:
        print(f"  poll identity_confirmed     wall={locked_at*1e3:7.1f} ms")
    print(f"  last_progress               wall="
          f"{(last_progress_wall or 0)*1e3:7.1f} ms")
    print(f"  whole-run                   wall={wall*1e3:7.1f} ms")
    lock_hits = [s for s in status_wall if s["event"] == "schedule_locked"]
    cand_hits = [s for s in status_wall if s["event"] == "candidate_emitted"]
    id_hits = [s for s in status_wall if s["event"] == "qm35_identity_confirmed"]
    rt = FS737 / 1e6

    def rate_line(name, n_samp, t_s):
        if t_s is None or t_s <= 0 or n_samp <= 0:
            print(f"  {name}: n/a")
            return
        msps = (n_samp / t_s) / 1e6
        print(f"  {name}: samples={n_samp}  wall={t_s*1e3:.2f} ms  "
              f"{msps:.1f} MS/s  {msps/rt:.3f}x RT")

    print()
    print("=== Compute-only realtime check (exclude idle drain) ===")
    print(f"  required = {rt:.2f} MS/s  (737.28)")
    if cand_hits:
        rate_line("acquire_to_candidate",
                  cand_hits[0]["abs_sample"],
                  cand_hits[0].get("wall_ms", 0) / 1e3)
    if id_hits:
        rate_line("acquire_to_identity",
                  id_hits[0]["abs_sample"],
                  id_hits[0].get("wall_ms", 0) / 1e3)
    if lock_hits:
        rate_line("start_to_locked",
                  lock_hits[0]["abs_sample"],
                  lock_hits[0].get("wall_ms", 0) / 1e3)
        lock_w = lock_hits[0].get("wall_ms", 0) / 1e3
        if last_ext_wall and last_ext_wall > lock_w:
            rate_line("after_lock_extractor_only",
                      n_stream - lock_hits[0]["abs_sample"],
                      last_ext_wall - lock_w)
        if last_progress_wall and last_progress_wall > lock_w:
            rate_line("after_lock_incl_demod_drain",
                      n_stream - lock_hits[0]["abs_sample"],
                      last_progress_wall - lock_w)
    if last_ext_wall:
        rate_line("start_to_extractor_idle", n_stream, last_ext_wall)
    if last_progress_wall:
        rate_line("start_to_last_progress", n_stream, last_progress_wall)

    print()
    print("=== First-packet compute (demod metadata, µs) ===")
    keys = (
        ("resample_us", "t_resample_us"),
        ("queue_us", "t_queue_us"),
        ("timing", "t_timing_us"),
        ("  coarse", "t_timing_coarse_us"),
        ("  fine", "t_timing_fine_us"),
        ("cfo", "t_cfo_us"),
        ("sfd", "t_sfd_us"),
        ("cir", "t_cir_us"),
        ("payload", "t_payload_us"),
        ("demod_core", "t_total_us"),
        ("demod_lat", "t_demod_lat_us"),
    )
    show_n = min(4, len(results))
    hdr = "".join(f"{name:>10}" for name, _ in keys)
    print(f"  {'id':>4} {'status':<14}{hdr}")
    for r in results[:show_n]:
        cols = "".join(
            f"{r[k]:10.0f}" if r[k] == r[k] else f"{'nan':>10}"
            for _, k in keys)
        print(f"  {r['packet_id']:4d} {r['status']:<14}{cols}")
    if results:
        fir_ms = resampler.resample_total_us() / 1000.0
        n_rs = max(1, resampler.pdus_emitted())
        print(f"  resampler FIR total={fir_ms:.2f} ms  "
              f"mean={fir_ms/n_rs*1000:.0f} us/pdu  "
              f"max={resampler.resample_max_us()} us")
        print(f"  resampler handler_total="
              f"{resampler.handler_total_us()/1000:.2f} ms  "
              f"sc16_convert={resampler.input_convert_total_us()/1000:.2f} ms")

    dump_ok = True
    if write_dir:
        dump_ok = verify_sc16_dump(
            dat, write_dir, n_stream, writer, pre, cap, post)
        if dump_ok and not args.no_notch_tone:
            from cancel_capture_tone import process_dump
            print()
            print("=== Notch ~6200 MHz CW on dump ===")
            process_dump(write_dir, fs=FS737, fc=6489.6e6,
                         rf_hint=6200e6, search_hz=80e6,
                         auto=False)

    ok = (
        ext.identity_confirmed()
        and ext.lock_state_name() in ("provisional", "locked")
        and fcs_ok >= 1
        and any(x.get("capture_mode") == "acquisition" for x in packets)
        and ext.scheduled_windows() >= 1
        and dump_ok
    )
    print()
    if ok:
        print("SUCCESS: locked QM35 without first_packet_sample")
        if write_dir:
            print(f"SUCCESS: native SC16 dump in {write_dir}")
        return 0
    print("FAIL: did not confirm QM35 identity and start scheduled capture")
    return 1


def verify_sc16_dump(dat, write_dir, n_stream, writer, pre, cap, post):
    iq_path = os.path.join(write_dir, "capture.iq")
    jsonl_path = os.path.join(write_dir, "capture.jsonl")
    print()
    print("=== Native SC16 dump check ===")
    if writer is not None:
        print(f"  writer received={writer.packets_received()} "
              f"written={writer.packets_written()} "
              f"dropped={writer.packets_dropped()} "
              f"samples={writer.samples_written()} "
              f"q_hi={writer.queue_high_watermark()}")
        if writer.packets_dropped() != 0:
            print("  FAIL: writer dropped packets")
            return False
    if not os.path.isfile(iq_path) or not os.path.isfile(jsonl_path):
        print(f"  FAIL: missing {iq_path} or {jsonl_path}")
        return False
    with open(jsonl_path) as f:
        metas = [json.loads(line) for line in f if line.strip()]
    if not metas:
        print("  FAIL: empty jsonl")
        return False
    src = np.memmap(dat, dtype=np.int16, mode="r")
    dumped = np.memmap(iq_path, dtype=np.int16, mode="r")
    mismatches = 0
    checked = 0
    sched_n = 0
    geom_ok = 0
    for m in metas:
        n = int(m.get("sample_count", 0))
        wstart = int(m.get("window_start_sample", m.get("start_sample", -1)))
        off = int(m.get("file_offset_samples", 0))
        if n <= 0 or wstart < 0:
            mismatches += 1
            continue
        if wstart + n > n_stream:
            continue
        src_i = np.array(src[wstart * 2:(wstart + n) * 2])
        dst_i = np.array(dumped[off * 2:(off + n) * 2])
        if src_i.shape != dst_i.shape or not np.array_equal(src_i, dst_i):
            mismatches += 1
        else:
            checked += 1
        if m.get("capture_mode") in ("scheduled", "provisional"):
            sched_n += 1
            mpre = int(m.get("pre_guard_samples", -1))
            mcap = int(m.get("capture_samples", -1))
            mpost = int(m.get("post_guard_samples", -1))
            pred = int(m.get("predicted_start_sample", -1))
            # Provisional/holdover adds ~25 µs extra on each side.
            if (mpre >= pre and mcap == cap and mpost >= post and pred >= 0
                    and pred - int(m.get("pre_trigger_samples", 0)) == wstart):
                geom_ok += 1
    print(f"  packets={len(metas)} bit_exact={checked} "
          f"mismatch={mismatches} scheduled={sched_n} geom_ok={geom_ok}")
    if metas:
        m0 = metas[0]
        print(f"  first: mode={m0.get('capture_mode')} "
              f"win={m0.get('window_start_sample')} "
              f"pred={m0.get('predicted_start_sample')} "
              f"pre/cap/post={m0.get('pre_guard_samples')}/"
              f"{m0.get('capture_samples')}/{m0.get('post_guard_samples')} "
              f"n={m0.get('sample_count')}")
        ms = next((m for m in metas
                   if m.get("capture_mode") in ("scheduled", "provisional")),
                  None)
        if ms:
            print(f"  scheduled: k={ms.get('schedule_index')} "
                  f"win={ms.get('window_start_sample')} "
                  f"pred={ms.get('predicted_start_sample')} "
                  f"head={ms.get('pre_guard_samples')} "
                  f"body={ms.get('capture_samples')} "
                  f"tail={ms.get('post_guard_samples')} "
                  f"n={ms.get('sample_count')}")
    ok = checked >= 1 and mismatches == 0 and writer.packets_dropped() == 0
    if sched_n:
        ok = ok and geom_ok >= 1
    print("  PASS" if ok else "  FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(main())
