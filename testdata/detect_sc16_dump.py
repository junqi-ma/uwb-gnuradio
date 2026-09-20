#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Raw native SC16 capture -> UwbDetectorSc16 -> packet_writer dump.

For DW3000-style continuous TX with unknown period (no t0/T): the
scheduled/auto extractors do not apply (they assume QM35 5 ms or known
schedule). The energy-gated detector emits one window PDU per preamble
candidate; the dump DIR/capture.iq+jsonl feeds
testdata/decode_scheduled_sc16_dump.py with --code-index/--reps/--sfd-mode.

Defaults target DW3000 code-10 / 512-SYNC @491.52 MHz:
  capture body covers 512*1016*32/65 = 256094 (preamble) + SFD + PSDU margin.

Usage (repo root):
  LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
  PYTHONPATH=/tmp/opencode/uwbshim:$PWD/gr-uwb/build/test_modules \
    python3 testdata/detect_sc16_dump.py RAW.sc16 OUTDIR \
      [--native-rate 491520000.0] [--max-seconds 0.2]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FS737 = 737.28e6
FS491 = 491.52e6
NATIVE_RATES = (737280000.0, 491520000.0)
TMPL = {
    737280000.0: os.path.join(HERE, "reference_preamble_code10_737p28.cf32"),
    491520000.0: os.path.join(HERE, "reference_preamble_code10_491p52.cf32"),
}
# Wide capture: full 512-SYNC preamble + SFD + PSDU headroom.
CAPTURE = {
    # 512*1016*48/65=384141 + SFD 6007 + margin -> 450000 (~610 us)
    737280000.0: (7373, 450000),
    # 512*1016*32/65=256094 + SFD 4005 + margin -> 300000 (~610 us)
    491520000.0: (4915, 300000),
}


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("input", help="raw interleaved int16 SC16 file")
    ap.add_argument("output", help="dump DIR (capture.iq + capture.jsonl)")
    ap.add_argument("--native-rate", type=float, default=FS491,
                    help="737280000.0 or 491520000.0 (default 491520000.0)")
    ap.add_argument("--template", default="",
                    help="native CF32 one-SYNC template (default: code-10 per rate)")
    ap.add_argument("--pre-trigger", type=int, default=0)
    ap.add_argument("--capture", type=int, default=0)
    ap.add_argument("--energy-threshold", type=float, default=0.02)
    ap.add_argument("--max-seconds", type=float, default=0.2,
                    help="stream prefix; 0 = whole file")
    ap.add_argument("--file-offset", type=int, default=0,
                    help="skip this many complex samples before streaming")
    return ap.parse_args()


def main():
    a = parse_args()
    if a.native_rate not in NATIVE_RATES:
        print(f"ERROR: --native-rate must be one of {NATIVE_RATES}, "
              f"got {a.native_rate}", file=sys.stderr)
        return 2
    if not os.path.isfile(a.input):
        print(f"ERROR: file not found: {a.input}", file=sys.stderr)
        return 2
    tmpl_path = a.template or TMPL[a.native_rate]
    if not os.path.isfile(tmpl_path):
        print(f"ERROR: missing template {tmpl_path}", file=sys.stderr)
        return 2

    import ctypes
    import glob
    import importlib.util
    repo = os.path.abspath(os.path.join(HERE, ".."))
    libdir = os.path.join(repo, "gr-uwb", "build", "lib")
    so_lib = os.path.join(libdir, "libgnuradio-uwb.so")
    if os.path.isfile(so_lib):
        ctypes.CDLL(so_lib, mode=ctypes.RTLD_GLOBAL)
    matches = glob.glob(os.path.join(
        repo, "gr-uwb", "build", "python", "uwb", "bindings",
        "uwb_python*.so"))
    if matches:
        import gnuradio.gr  # noqa: F401
        spec = importlib.util.spec_from_file_location(
            "uwb_python", matches[0])
        uwb = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(uwb)
    else:
        from gnuradio import uwb
    from gnuradio import blocks, gr
    import pmt as p

    tmpl = np.fromfile(tmpl_path, np.complex64)
    d_pre, d_cap = CAPTURE[a.native_rate]
    pre = a.pre_trigger if a.pre_trigger > 0 else d_pre
    cap = a.capture if a.capture > 0 else d_cap

    nbytes = os.path.getsize(a.input)
    n_complex = nbytes // 4
    skip = max(0, int(a.file_offset))
    avail = max(0, n_complex - skip)
    n_stream = avail
    if a.max_seconds and a.max_seconds > 0:
        n_stream = min(avail, int(round(a.max_seconds * a.native_rate)))
    print(f"input: {a.input} samples={n_complex} skip={skip} "
          f"stream={n_stream} ({n_stream / a.native_rate * 1e3:.2f} ms)")
    print(f"template: {tmpl_path} n={tmpl.size} "
          f"energy={float(np.sum(np.abs(tmpl) ** 2)):.6f}")
    print(f"detector: pre={pre} cap={cap} energy_th={a.energy_threshold} "
          f"rate={a.native_rate:.0f}")

    det = uwb.detector_sc16(
        [complex(x) for x in tmpl], pre, cap,
        float(a.energy_threshold), 100, 4, 1, 16, float(a.native_rate))
    dbg = blocks.message_debug()
    os.makedirs(a.output, exist_ok=True)
    writer = uwb.packet_writer(a.output, "capture", False)

    tb = gr.top_block("detect_sc16_dump")
    src = blocks.file_source(2 * gr.sizeof_short, a.input, False)
    if skip > 0:
        tb.connect(src, blocks.skiphead(2 * gr.sizeof_short, skip), det)
    else:
        tb.connect(src, det)
    if n_stream < avail:
        # head limits items pulled through the chain
        head = blocks.head(2 * gr.sizeof_short, n_stream)
        tb.disconnect_all()
        if skip > 0:
            tb.connect(src, blocks.skiphead(2 * gr.sizeof_short, skip),
                       head, det)
        else:
            tb.connect(src, head, det)
    tb.msg_connect(det, "packet", dbg, "store")
    tb.msg_connect(det, "packet", writer, "packet")
    t0 = time.time()
    tb.run()
    wall = time.time() - t0

    def num(meta, key, default=-1):
        v = p.dict_ref(meta, p.intern(key), p.PMT_NIL)
        for f in (p.to_long, p.to_uint64, p.to_double):
            try:
                return int(f(v))
            except Exception:
                pass
        return default

    def flt(meta, key, default=float("nan")):
        v = p.dict_ref(meta, p.intern(key), p.PMT_NIL)
        for f in (p.to_double, p.to_long, p.to_uint64):
            try:
                return float(f(v))
            except Exception:
                pass
        return default

    rows = []
    for i in range(dbg.num_messages()):
        md = p.car(dbg.get_message(i))
        rows.append({
            "id": i,
            "start": num(md, "start_sample"),
            "n": num(md, "sample_count"),
            "metric": flt(md, "detection_metric"),
        })
    print(f"wall_s={wall:.2f} detections={len(rows)} "
          f"written={writer.packets_written()} "
          f"dropped={writer.packets_dropped()}")
    for r in rows[:20]:
        print(f"  id={r['id']} start={r['start']} n={r['n']} "
              f"metric={r['metric']:.3f}")
    if len(rows) > 20:
        print(f"  ... {len(rows) - 20} more")
    if len(rows) >= 2:
        ds = [b["start"] - a["start"] for a, b in zip(rows, rows[1:])]
        ds = [d for d in ds if d > 0]
        if ds:
            print(f"spacing min/med/max = {min(ds)}/{sorted(ds)[len(ds)//2]}/"
                  f"{max(ds)} samples "
                  f"({min(ds)/a.native_rate*1e6:.1f}/"
                  f"{sorted(ds)[len(ds)//2]/a.native_rate*1e6:.1f}/"
                  f"{max(ds)/a.native_rate*1e6:.1f} us)")
    summary = {
        "input": a.input,
        "native_rate": a.native_rate,
        "template": tmpl_path,
        "stream_samples": n_stream,
        "detections": len(rows),
        "written": writer.packets_written(),
        "dropped": writer.packets_dropped(),
    }
    with open(os.path.join(a.output, "detect_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    return 0 if (len(rows) > 0 and writer.packets_dropped() == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
