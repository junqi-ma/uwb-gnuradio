#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Parse a scheduled-extractor SC16 dump and demodulate each window.

Dump contract (UwbPacketWriter after Auto/Scheduled extractor):
  DIR/capture.iq     concatenated native SC16 (int16 I/Q)
  DIR/capture.jsonl  one JSON object per window, including
                     window_start_sample, predicted_start_sample,
                     pre_guard_samples / capture_samples / post_guard_samples

Algorithm follows testdata/analyze_qm35_sc16_matlab.m:
  slice head / QM35 body / tail using JSONL geometry
  PDU 65/48 (quality_minorder) 737.28 -> 998.4
  seed QM35 timing at the mapped predicted start
  UwbRealtimeDemodulator  code 9 / 64 SYNC / 4z2 / bypass

Optional --dw1000 runs a second demod (code 10 / 256 SYNC / decawave)
on the same resampled window with no QM35 seed (full-window search).

Usage (repo root):
  PYTHONPATH=$PWD/gr-uwb/build/test_modules \\
  LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \\
    python3 testdata/decode_scheduled_sc16_dump.py DIR \\
      [--max-slots N] [--dw1000] [--csv OUT.csv]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

import numpy as np
from gnuradio import blocks, gr
import pmt as p

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
TMPL_998 = os.path.join(HERE, "reference_preamble.bin")
CODE10_CFILE = os.path.join(HERE, "uwb_code10_preamble16_payload8.cfile")
FS737 = 737.28e6
FS998 = 998.4e6
SYNC_LEN = 1016


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


def load_jsonl(path):
    metas = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                metas.append(json.loads(line))
    return metas


def geometry_of(meta):
    n = int(meta.get("sample_count", 0))
    pre = meta.get("pre_trigger_samples")
    if pre is None:
        pre = meta.get("pre_guard_samples", 0)
    pre = int(pre)
    body = meta.get("capture_samples")
    post = meta.get("post_guard_samples")
    if body is None:
        body = n - pre - (int(post) if post is not None else 0)
    body = int(body)
    if post is None:
        post = n - pre - body
    post = int(post)
    pre = max(0, min(n, pre))
    body = max(0, min(n - pre, body))
    post = max(0, n - pre - body)
    return pre, body, post


def read_window(iq_path, meta):
    n = int(meta["sample_count"])
    if "file" in meta and meta["file"]:
        path = os.path.join(os.path.dirname(iq_path), meta["file"])
        off = 0
    else:
        path = iq_path
        off = int(meta.get("file_offset_samples", 0))
    raw = np.memmap(path, dtype=np.int16, mode="r")
    sl = raw[off * 2:(off + n) * 2]
    if sl.size != n * 2:
        raise RuntimeError(
            f"short IQ packet_id={meta.get('packet_id')} "
            f"need={n * 2} got={sl.size}")
    return np.array(sl, dtype=np.int16)


def add_i64(md, key, value):
    if value is None:
        return md
    return p.dict_add(md, p.intern(key), p.from_long(int(value)))


def add_u64(md, key, value):
    if value is None:
        return md
    return p.dict_add(md, p.intern(key), p.from_uint64(int(value)))


def add_sym(md, key, value):
    if not value:
        return md
    return p.dict_add(md, p.intern(key), p.intern(str(value)))


def meta_to_pmt(meta, *, drop_predicted=False):
    md = p.make_dict()
    md = add_u64(md, "packet_id", meta.get("packet_id", 0))
    md = add_u64(md, "schedule_index", meta.get("schedule_index", 0))
    wstart = meta.get("window_start_sample", meta.get("start_sample", 0))
    pred = meta.get("predicted_start_sample", wstart)
    md = add_i64(md, "window_start_sample", wstart)
    md = add_i64(md, "start_sample", meta.get("start_sample", pred))
    if not drop_predicted:
        md = add_i64(md, "predicted_start_sample", pred)
    md = add_i64(md, "trigger_sample", meta.get("trigger_sample", pred))
    pre, body, post = geometry_of(meta)
    md = add_i64(md, "pre_guard_samples", pre)
    md = add_i64(md, "capture_samples", body)
    md = add_i64(md, "post_guard_samples", post)
    md = add_i64(md, "pre_trigger_samples", pre)
    md = add_i64(md, "sample_count", meta.get("sample_count", 0))
    md = p.dict_add(md, p.intern("sample_rate"), p.from_double(FS737))
    md = p.dict_add(md, p.intern("sample_format"), p.intern("sc16"))
    md = add_sym(md, "capture_mode", meta.get("capture_mode"))
    md = add_sym(md, "lock_state", meta.get("lock_state"))
    return md


def parse_result(msg):
    md = p.car(msg) if p.is_pair(msg) else msg
    payload = p.cdr(msg) if p.is_pair(msg) else p.PMT_NIL
    hexstr = ""
    if p.is_u8vector(payload):
        n = 0
        buf = p.u8vector_elements(payload)
        if buf:
            hexstr = "".join(f"{int(b):02X}" for b in buf)
            n = len(buf)
    else:
        n = as_int(dict_get(md, "payload_nbytes", p.from_long(0)), 0)
    return {
        "packet_id": as_int(dict_get(md, "packet_id", p.from_uint64(0)), 0),
        "schedule_index": as_int(
            dict_get(md, "schedule_index", p.from_uint64(0)), -1),
        "status": as_str(dict_get(md, "status", p.intern("?"))),
        "fcs": bool(p.to_bool(dict_get(md, "fcs_pass", p.from_bool(False)))),
        "det_start": as_int(dict_get(md, "detected_start_sample",
                                     p.from_long(-1))),
        "pred_start": as_int(dict_get(md, "predicted_start_sample",
                                      p.from_long(-1))),
        "win_start": as_int(dict_get(md, "window_start_sample",
                                     p.from_long(-1))),
        "timing_metric": as_num(dict_get(md, "timing_metric",
                                         p.from_double(-1))),
        "timing_peaks": as_int(dict_get(md, "timing_peaks",
                                        p.from_uint64(0)), 0),
        "cfo_hz": as_num(dict_get(md, "cfo_hz", p.from_double(0))),
        "sfd_metric": as_num(dict_get(md, "sfd_metric", p.from_double(-1))),
        "payload_n": n,
        "payload_hex": hexstr,
        "t_total_us": as_num(dict_get(md, "stage_total_us",
                                      p.from_double(-1))),
        "fcs_received": as_int(dict_get(md, "fcs_received",
                                        p.from_uint64(0)), 0),
        "fcs_calculated": as_int(dict_get(md, "fcs_calculated",
                                          p.from_uint64(0)), 0),
    }


def load_code10_template():
    if not os.path.isfile(CODE10_CFILE):
        raise FileNotFoundError(CODE10_CFILE)
    x = np.fromfile(CODE10_CFILE, np.complex64)
    mag = np.abs(x)
    start = int(np.argmax(mag > 0.05 * np.max(mag)))
    tmpl = x[start:start + SYNC_LEN].astype(np.complex64)
    if tmpl.size != SYNC_LEN:
        raise RuntimeError(f"code-10 template short: {tmpl.size}")
    e = float(np.sum(np.abs(tmpl) ** 2))
    if e > 0:
        tmpl = tmpl / np.sqrt(e)
    return tmpl


def parser():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump_dir",
                    help="directory with capture.iq + capture.jsonl")
    ap.add_argument("--max-slots", type=int, default=0,
                    help="limit scheduled/provisional windows (0 = all)")
    ap.add_argument("--dw1000", action="store_true",
                    help="also demod DW1000 (code 10 / 256 / decawave)")
    ap.add_argument("--csv", default="",
                    help="output CSV path (default: DUMP/scheduled_dump.csv)")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--cir-filter-mode", default="bypass")
    return ap


def main():
    args = parser().parse_args()
    dump = os.path.abspath(args.dump_dir)
    iq_path = os.path.join(dump, "capture.iq")
    jsonl_path = os.path.join(dump, "capture.jsonl")
    if not os.path.isfile(iq_path) or not os.path.isfile(jsonl_path):
        print(f"ERROR: need {iq_path} and {jsonl_path}", file=sys.stderr)
        return 2
    if not os.path.isfile(TMPL_998):
        print(f"ERROR: missing {TMPL_998}", file=sys.stderr)
        return 2

    from gnuradio import uwb

    metas = load_jsonl(jsonl_path)
    if args.max_slots > 0:
        kept, n_sched = [], 0
        for m in metas:
            mode = m.get("capture_mode", "")
            if mode in ("scheduled", "provisional"):
                if n_sched >= args.max_slots:
                    continue
                n_sched += 1
            kept.append(m)
        metas = kept

    tmpl = np.fromfile(TMPL_998, np.complex64)
    print("=== Decode scheduled SC16 dump ===")
    print(f"dir: {dump}")
    print(f"packets={len(metas)}  qm35=code9/64/4z2  "
          f"dw1000={'on' if args.dw1000 else 'off'}")

    resampler = uwb.pdu_rational_resampler_ccf_65_48("quality_minorder")
    demod = uwb.realtime_demodulator.make_from_template(
        tmpl.tolist(),
        max(1, args.workers),
        64,
        "4z2",
        0,
        args.cir_filter_mode,
        9,
        64,
        14,
    )
    dbg = blocks.message_debug()
    tb = gr.top_block("decode_scheduled_sc16_dump")
    tb.msg_connect(resampler, "packet", demod, "samples")
    tb.msg_connect(demod, "result", dbg, "store")

    demod_dw = None
    dbg_dw = None
    if args.dw1000:
        tmpl10 = load_code10_template()
        demod_dw = uwb.realtime_demodulator.make_from_template(
            tmpl10.tolist(),
            max(1, args.workers),
            64,
            "decawave",
            0,
            args.cir_filter_mode,
            10,
            256,
            14,
        )
        dbg_dw = blocks.message_debug()

        class ClearQm35Seed(gr.sync_block):
            """DW1000 may start in the head; do not seed at the QM35 t0."""

            def __init__(self):
                gr.sync_block.__init__(
                    self, name="clear_qm35_seed", in_sig=None, out_sig=None)
                self.message_port_register_in(p.intern("in"))
                self.message_port_register_out(p.intern("out"))
                self.set_msg_handler(p.intern("in"), self._on)

            def _on(self, msg):
                if not p.is_pair(msg):
                    return
                md = p.car(msg)
                data = p.cdr(msg)
                if p.is_dict(md):
                    md = p.dict_delete(md, p.intern("predicted_start_sample"))
                    md = p.dict_add(md, p.intern("predicted_start_sample"),
                                    p.from_long(-1))
                self.message_port_pub(p.intern("out"), p.cons(md, data))

        strip = ClearQm35Seed()
        tb.msg_connect(resampler, "packet", strip, "in")
        tb.msg_connect(strip, "out", demod_dw, "samples")
        tb.msg_connect(demod_dw, "result", dbg_dw, "store")

    tb.start()
    posted = 0
    slices = []
    t0 = time.time()
    for meta in metas:
        pre, body, post = geometry_of(meta)
        iq = read_window(iq_path, meta)
        n = len(iq) // 2
        slices.append({
            "packet_id": int(meta.get("packet_id", posted)),
            "schedule_index": int(meta.get("schedule_index", -1))
            if meta.get("schedule_index") is not None else -1,
            "capture_mode": meta.get("capture_mode", ""),
            "lock_state": meta.get("lock_state", ""),
            "window_start": int(meta.get("window_start_sample",
                                         meta.get("start_sample", -1))),
            "predicted_start": int(meta.get("predicted_start_sample", -1)),
            "pre": pre,
            "body": body,
            "post": post,
            "n": n,
        })
        pdu = p.cons(meta_to_pmt(meta), p.init_s16vector(len(iq), iq))
        resampler._post(p.intern("packet"), pdu)
        posted += 1

    deadline = time.time() + max(60.0, posted * 2.0 + 15.0)
    while time.time() < deadline:
        done = dbg.num_messages()
        dw_done = dbg_dw.num_messages() if dbg_dw else posted
        if done >= posted and dw_done >= posted:
            break
        time.sleep(0.05)
    try:
        demod.drain()
        if demod_dw is not None:
            demod_dw.drain()
    except Exception:
        pass
    time.sleep(0.2)
    tb.stop()
    tb.wait()
    wall = time.time() - t0

    qm35 = [parse_result(dbg.get_message(i))
            for i in range(dbg.num_messages())]
    dw = []
    if dbg_dw is not None:
        dw = [parse_result(dbg_dw.get_message(i))
              for i in range(dbg_dw.num_messages())]
    qm35_by_id = {r["packet_id"]: r for r in qm35}
    dw_by_id = {r["packet_id"]: r for r in dw}

    print()
    print(f"wall_s={wall:.2f}  posted={posted}  "
          f"qm35_results={len(qm35)}  dw_results={len(dw)}")
    print(f"resampler emitted={resampler.pdus_emitted()} "
          f"dropped={resampler.pdus_dropped()}")
    print(f"qm35 rx={demod.jobs_received()} "
          f"comp={demod.jobs_completed()} "
          f"fail={demod.jobs_failed()} "
          f"drop={demod.jobs_dropped()}")
    if demod_dw is not None:
        print(f"dw   rx={demod_dw.jobs_received()} "
              f"comp={demod_dw.jobs_completed()} "
              f"fail={demod_dw.jobs_failed()} "
              f"drop={demod_dw.jobs_dropped()}")

    print()
    print(f"{'id':>4} {'mode':<12} {'k':>4} "
          f"{'pre/body/post':<22} {'qm35':<16} {'fcs':>3} "
          f"{'peaks':>5} {'cfo':>8} {'hex':<24}")
    rows = []
    fcs_ok = 0
    for sl in slices:
        pid = sl["packet_id"]
        r = qm35_by_id.get(pid, {})
        d = dw_by_id.get(pid, {})
        status = r.get("status", "missing")
        fcs = bool(r.get("fcs", False))
        fcs_ok += int(fcs)
        hexstr = r.get("payload_hex", "")
        geom = f"{sl['pre']}/{sl['body']}/{sl['post']}"
        print(f"{pid:4d} {sl['capture_mode']:<12} {sl['schedule_index']:4d} "
              f"{geom:<22} {status:<16} {int(fcs):3d} "
              f"{r.get('timing_peaks', 0):5d} "
              f"{r.get('cfo_hz', float('nan')):8.1f} "
              f"{hexstr[:24]}")
        det = r.get("det_start", -1)
        pred = r.get("pred_start", sl["predicted_start"])
        row = {
            "decoder": "gnuradio_cpp",
            "packet_id": pid,
            "schedule_index": sl["schedule_index"],
            "capture_mode": sl["capture_mode"],
            "lock_state": sl["lock_state"],
            "window_start_native": sl["window_start"],
            "predicted_start_native": sl["predicted_start"],
            "pre_samples": sl["pre"],
            "body_samples": sl["body"],
            "post_samples": sl["post"],
            "sample_count": sl["n"],
            "qm35_status": status,
            "qm35_decode_ok": int(status != "missing"),
            "qm35_fcs_pass": int(fcs),
            "qm35_detected_start": det,
            "qm35_predicted_start_out": pred,
            "qm35_det_minus_pred": (det - pred) if det is not None
            and pred is not None and int(det) >= 0 and int(pred) >= 0
            else "",
            "qm35_timing_metric": r.get("timing_metric", ""),
            "qm35_timing_peaks": r.get("timing_peaks", ""),
            "qm35_cfo_hz": r.get("cfo_hz", ""),
            "qm35_sfd_metric": r.get("sfd_metric", ""),
            "qm35_payload_hex": hexstr,
            "qm35_fcs_received": r.get("fcs_received", ""),
            "qm35_fcs_calculated": r.get("fcs_calculated", ""),
            "dw_status": d.get("status", ""),
            "dw_fcs_pass": int(bool(d.get("fcs", False))) if d else "",
            "dw_detected_start": d.get("det_start", ""),
            "dw_timing_peaks": d.get("timing_peaks", ""),
            "dw_cfo_hz": d.get("cfo_hz", ""),
            "dw_payload_hex": d.get("payload_hex", ""),
        }
        rows.append(row)

    csv_path = args.csv or os.path.join(dump, "scheduled_dump.csv")
    if rows:
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {csv_path}")
        summary = {
            "decoder": "gnuradio_cpp",
            "block": "UwbRealtimeDemodulator",
            "resampler": "UwbPduRationalResamplerCcf65_48 quality_minorder",
            "phy": {
                "code_index": 9,
                "preamble_repetitions": 64,
                "sfd_mode": "4z2",
                "cir_filter_mode": args.cir_filter_mode,
                "data_rate_mbps": 6.81,
            },
            "input_rate_hz": FS737,
            "demod_rate_hz": FS998,
            "dump_dir": dump,
            "n_windows": len(slices),
            "qm35_results": len(qm35),
            "qm35_fcs_pass": fcs_ok,
            "seed_rule": "predicted_start mapped 65/48; "
                         "seededStartOne = round(pre*65/48)+1",
        }
        js_path = os.path.splitext(csv_path)[0] + "_summary.json"
        with open(js_path, "w") as f:
            json.dump(summary, f, indent=2)
            f.write("\n")
        print(f"wrote {js_path}")

    print(f"\nQM35 FCS {fcs_ok}/{len(slices)}")
    if args.dw1000:
        dw_fcs = sum(1 for sl in slices
                     if dw_by_id.get(sl["packet_id"], {}).get("fcs"))
        print(f"DW1000 FCS {dw_fcs}/{len(slices)}")

    ok = len(qm35) == posted and fcs_ok >= 1 and resampler.pdus_dropped() == 0
    print("SUCCESS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
