#!/usr/bin/env python3
"""Multi-packet and adjacent-packet tests for uwb.detector (开发需求参考.md #2/#5).

  #2  N packets inserted at known positions → every one detected, start ~= ground
      truth, PDU IQ identical to the stream region, no false positives.
  #5  Two adjacent packets with a small inter-packet gap → both detected
      (the silence-based region holdoff must not swallow the second packet).

Run from the build tree:
  PYTHONPATH=$GR_UWB/build/test_modules LD_LIBRARY_PATH=$GR_UWB/build/lib \
    python3 test_multi_packet.py
"""
import os, sys
import numpy as np, pmt
from gnuradio import gr, blocks, uwb

ROOT = os.path.dirname(os.path.abspath(__file__))
CFILE = os.path.join(ROOT, "testdata", "uwb_code9_preamble64_payload128_standard_sfd.cfile")
L = 1016
PK = 4992000
PRE = 2032
FAIL = []

def check(cond, msg):
    print(f"[{'PASS' if cond else 'FAIL'}] {msg}")
    if not cond: FAIL.append(msg)

def load():
    x = np.fromfile(CFILE, dtype=np.complex64)
    pkt = x[PK:PK+256448]                     # real packet waveform (256k samples)
    tmpl = x[PK:PK+L]; tmpl = tmpl/np.sqrt(np.sum(np.abs(tmpl)**2))
    return x, pkt, tmpl

def build_stream(packets, positions, gap):
    # packets: list of (waveform, length). positions: list of start indices.
    last = max(p + len(pkt) for p, pkt in zip(positions, packets))
    s = np.zeros(last + 5000, dtype=np.complex64)     # trailing silence
    for p, pkt in zip(positions, packets):
        s[p:p+len(pkt)] = pkt
    return s

def run_detector(stream, tmpl, capture=200000):
    det = uwb.detector(tmpl.tolist(), PRE, capture)
    src = blocks.vector_source_c(stream.tolist(), False)
    dbg = blocks.message_debug()
    tb = gr.top_block(); tb.connect(src, det); tb.msg_connect(det, "packet", dbg, "store")
    tb.run()
    out = []
    for i in range(dbg.num_messages()):
        m = dbg.get_message(i)
        meta = pmt.to_python(pmt.car(m))
        iq = np.asarray(pmt.c32vector_elements(pmt.cdr(m)), dtype=np.complex64)
        out.append((meta, iq))
    return out

def verify_packets(detected, positions, stream, label):
    """detected: [(meta, iq)]. positions: ground-truth packet starts (ascending)."""
    check(len(detected) == len(positions),
          f"{label}: detected {len(detected)} / {len(positions)} packets")
    # sort detected by start
    detected = sorted(detected, key=lambda d: d[0]["start_sample"])
    for (meta, iq), pos in zip(detected, positions):
        lo = meta["start_sample"]
        n = meta["sample_count"]
        ok_start = abs(lo - pos) < 50
        ok_metric = meta["detection_metric"] > 0.9
        gt = stream[lo-PRE : lo-PRE+n]
        err = float(np.mean(np.abs(iq - gt))) if n == len(gt) else 1.0
        ok_iq = err < 1e-6
        check(ok_start and ok_metric and ok_iq,
              f"{label} pkt@{pos}: start={lo} metric={meta['detection_metric']:.3f} "
              f"n={n} iq_diff={err:.2e}")
    # no extra (already covered by count == len)

def test_multi_real(n=5):
    x, pkt, tmpl = load()
    # random gaps in [300k, 700k) samples (packet ~256k, capture ~202k)
    rng = np.random.RandomState(42)
    gaps = rng.randint(300000, 700000, n-1)
    positions = [20000]
    for g in gaps: positions.append(positions[-1] + len(pkt) + g)
    stream = build_stream([pkt]*n, positions, 0)
    verify_packets(run_detector(stream, tmpl), positions, stream, f"multi-real({n})")

def test_multi_100():
    x, pkt, tmpl = load()
    # compact synthetic packet: 8 SYNC symbols + short tail
    comp = np.concatenate([pkt[:8*L], np.zeros(2000, dtype=np.complex64)])
    n = 100
    rng = np.random.RandomState(7)
    gaps = rng.randint(20000, 60000, n-1)
    positions = [5000]
    for g in gaps: positions.append(positions[-1] + len(comp) + g)
    stream = build_stream([comp]*n, positions, 0)
    # Keep the requested capture shorter than the minimum packet spacing.
    # Overlapping fixed capture windows require a separate multi-active-capture
    # design; this case validates detector/holdoff behavior, not overlap fanout.
    verify_packets(run_detector(stream, tmpl, capture=10000), positions, stream,
                   f"multi-100")

def test_adjacent():
    x, pkt, tmpl = load()
    # The decimated energy gate's window is 32 blocks x 100 = 3200 samples, so
    # two packets must be separated by more than ~4000 samples (≈4 µs at
    # 998.4 MHz) for the gate to drop below threshold between them.  A 6000
    # sample gap is a realistic inter-packet spacing and verifies the holdoff
    # does not swallow the second packet (需求 #5).
    for gap, label in [(6000, "adjacent-gap6000"), (50000, "adjacent-gap50000")]:
        p0 = 20000
        p1 = p0 + len(pkt) + gap
        stream = build_stream([pkt, pkt], [p0, p1], gap)
        verify_packets(run_detector(stream, tmpl), [p0, p1], stream, label)
    print("  (note: gaps below ~4000 samples merge into one region / PDU)")

if __name__ == "__main__":
    test_multi_real(5)
    test_adjacent()
    test_multi_100()
    print()
    if FAIL:
        print(f"FAILED {len(FAIL)}"); sys.exit(1)
    print("MULTI-PACKET + ADJACENT TESTS OK")
