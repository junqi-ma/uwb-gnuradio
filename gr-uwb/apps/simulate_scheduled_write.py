#!/usr/bin/env python3
"""Real-time paced sustained-writer test without allocating full-rate IQ."""
import argparse
import json
import os
import sys
import time
import numpy as np
import pmt
from gnuradio import blocks, gr, uwb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--slot-rate", type=float, default=200.0)
    ap.add_argument("--samples", type=int, default=203776)
    a = ap.parse_args()
    if a.duration <= 0 or a.slot_rate <= 0 or a.samples <= 0:
        ap.error("duration, slot-rate and samples must be positive")

    os.makedirs(a.output, exist_ok=True)
    w = uwb.packet_writer(a.output, "capture", False)
    # A dormant strobe registers the message-only writer in the flowgraph;
    # test PDUs themselves are posted explicitly at the configured slot rate.
    owner = blocks.message_strobe(pmt.PMT_NIL, 24 * 60 * 60 * 1000)
    tb = gr.top_block("scheduled_writer_sustained_test")
    tb.msg_connect(owner, "strobe", w, "packet")
    tb.start()
    # Deterministic non-silent window; immutable PMT payload is safely reused.
    t = np.arange(a.samples, dtype=np.float32)
    iq = (0.25*np.sin(t*0.013) + 0.25j*np.cos(t*0.017)).astype(np.complex64)
    payload = pmt.init_c32vector(a.samples, iq.tolist())
    count = int(round(a.duration * a.slot_rate))
    period = 1.0 / a.slot_rate
    started = time.monotonic()
    for k in range(count):
        meta = pmt.make_dict()
        meta = pmt.dict_add(meta, pmt.intern("packet_id"), pmt.from_long(k))
        for key, val in (("schedule_index", k),
                         ("start_sample", int(round(k*998.4e6/a.slot_rate))),
                         ("sample_count", a.samples)):
            meta = pmt.dict_add(meta, pmt.intern(key), pmt.from_uint64(val))
        meta = pmt.dict_add(meta, pmt.intern("sample_rate"), pmt.from_double(998.4e6))
        meta = pmt.dict_add(meta, pmt.intern("detection_metric"), pmt.from_double(1.0))
        w.to_basic_block()._post(pmt.intern("packet"), pmt.cons(meta, payload))
        deadline = started + (k + 1) * period
        delay = deadline - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    tb.stop(); tb.wait()
    elapsed = time.monotonic() - started

    iq_path = os.path.join(a.output, "capture.iq")
    json_path = os.path.join(a.output, "capture.jsonl")
    size = os.path.getsize(iq_path) if os.path.exists(iq_path) else 0
    lines = 0
    if os.path.exists(json_path):
        with open(json_path, encoding="utf-8") as f:
            lines = sum(1 for line in f if line.strip())
    expected = count * a.samples * 4
    print(json.dumps({"duration_requested_s": a.duration,
                      "elapsed_s": elapsed, "slots_expected": count,
                      "slots_written": w.packets_written(),
                      "slots_received": w.packets_received(),
                      "slots_dropped": w.packets_dropped(),
                      "queue_high_watermark": w.queue_high_watermark(),
                      "bytes_expected": expected, "bytes_written": size,
                      "jsonl_lines": lines,
                      "write_MB_s": size/elapsed/1e6}, indent=2))
    ok = (w.packets_written() == count and w.packets_dropped() == 0 and
          size == expected and lines == count)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
