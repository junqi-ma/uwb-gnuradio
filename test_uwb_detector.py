#!/usr/bin/env python3
"""
End-to-end execution test for the GNU Radio UWB detection blocks.

Feeds the known UWB test signal
(testdata/uwb_code9_preamble64_payload128_standard_sfd.cfile, IEEE 802.15.4a
HRP code-9, 64-symbol SYNC preamble, fs = 998.4 MHz) through both blocks in a
real flowgraph and checks:

  * UWB energy detector  : metric rises from ~0 to a large value exactly at the
                           known packet start, and the gate flag is asserted.
  * UWB preamble detector: normalized correlation peaks (~1) inside the SYNC
                           preamble region (64 symbols x 1016 samples) and
                           stays near 0 in the leading silence.

Also reports per-block throughput on the full 5.25 M-sample capture.

Run against the build tree:
  PYTHONPATH=$GR_UWB/build/test_modules \
  LD_LIBRARY_PATH=$GR_UWB/build/lib \
  python3 test_uwb_detector.py

Or against a staged install:
  PYTHONPATH=<prefix>/lib/python3/dist-packages \
  LD_LIBRARY_PATH=<prefix>/lib/x86_64-linux-gnu \
  python3 test_uwb_detector.py
"""

import os
import sys
import time

import numpy as np
from gnuradio import gr, blocks, uwb

# ---------------------------------------------------------------------------
# Test constants (verified against testdata/UWB_test_signal_description.md and
# autocorrelation of the reference signal).
# ---------------------------------------------------------------------------
CFILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "testdata", "uwb_code9_preamble64_payload128_standard_sfd.cfile")
SAMPLES_PER_SYMBOL = 1016
SYNC_SYMBOLS = 64
PACKET_START = 4992000          # 0-based first non-zero sample
PACKET_LENGTH = 256448

ENERGY_WINDOW = 1016            # one SYNC symbol worth of smoothing
PREAMBLE_THRESHOLD = 0.5        # normalized correlation gate

FAIL = []


def check(cond, msg):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {msg}")
    if not cond:
        FAIL.append(msg)


def load_signal():
    data = np.fromfile(CFILE, dtype=np.complex64)
    print(f"GNU Radio version: {gr.version()}")
    print(f"loaded {data.size} samples ({data.size / 1e6:.2f} M) "
          f"from {os.path.basename(CFILE)}")
    return data


def run_block(x, make_block, threshold):
    src = blocks.vector_source_c(x.tolist(), False)
    snk_metric = blocks.vector_sink_f()
    snk_flag = blocks.vector_sink_b()
    tb = gr.top_block()
    tb.connect(src, make_block)
    tb.connect(make_block, snk_metric)
    tb.connect((make_block, 1), snk_flag)

    t0 = time.perf_counter()
    tb.run()
    dt = time.perf_counter() - t0

    metric = np.asarray(snk_metric.data(), dtype=np.float32)
    flag = np.asarray(snk_flag.data(), dtype=np.uint8)
    return metric, flag, dt


def main():
    x = load_signal()

    # ---- energy detector -------------------------------------------------
    en_metric, en_flag, dt = run_block(
        x, uwb.energy_detector(1e-3, ENERGY_WINDOW), 1e-3)
    print(f"energy detector : {en_metric.size} samples in {dt:.3f}s "
          f"({en_metric.size / 1e6 / dt:.1f} MS/s)")
    lead_max = float(en_metric[:PACKET_START - ENERGY_WINDOW].max())
    pkt_mean = float(en_metric[PACKET_START:PACKET_START + 10000].mean())
    check(lead_max < 1e-6,
          f"leading 5 ms silence: max energy {lead_max:.2e} (~0 expected)")
    check(pkt_mean > 100 * lead_max + 1e-6,
          f"packet region energy {pkt_mean:.3e} >> silence {lead_max:.2e}")
    gate_on = en_flag[PACKET_START + 10 * ENERGY_WINDOW:PACKET_START + 10000]
    check(bool(gate_on.all()),
          "energy gate flag asserted throughout the packet region")

    # ---- preamble detector -----------------------------------------------
    template = x[PACKET_START:PACKET_START + SAMPLES_PER_SYMBOL]
    template = template / np.sqrt(np.sum(np.abs(template) ** 2))
    pre_metric, pre_flag, dt = run_block(
        x, uwb.preamble_detector(template.tolist(), PREAMBLE_THRESHOLD), None)
    print(f"preamble detector: {pre_metric.size} samples in {dt:.3f}s "
          f"({pre_metric.size / 1e6 / dt:.1f} MS/s)")

    # SYNC preamble occupies [PACKET_START, PACKET_START + 64*1016).
    sync_lo = PACKET_START
    sync_hi = PACKET_START + SYNC_SYMBOLS * SAMPLES_PER_SYMBOL

    lead_metric = float(pre_metric[:sync_lo].max())
    check(lead_metric < 0.1,
          f"leading silence max correlation {lead_metric:.3f} (< 0.1 expected)")

    # Peak lattice: one peak per SYNC symbol, spaced SAMPLES_PER_SYMBOL apart.
    region = pre_metric[sync_lo:sync_hi]
    thr = max(0.5, 0.8 * float(region.max()))
    peaks = []
    for i in range(1, region.size - 1):
        if (region[i] > region[i - 1] and region[i] >= region[i + 1]
                and region[i] > thr
                and (not peaks or i - peaks[-1] >= SAMPLES_PER_SYMBOL // 2)):
            peaks.append(i)
    check(len(peaks) >= 8,
          f"found {len(peaks)} correlation peaks in the SYNC preamble "
          f"(expected >= 8)")
    if len(peaks) >= 2:
        spacing = np.diff(np.asarray(peaks))
        check(abs(float(spacing.mean()) - SAMPLES_PER_SYMBOL) < 5,
              f"peak spacing {spacing.mean():.1f} ~= {SAMPLES_PER_SYMBOL}")
    check(float(region.max()) > 0.9,
          f"max correlation inside SYNC preamble {region.max():.3f} "
          f"(> 0.9 expected)")

    # The very first correlation peak should land inside the first SYNC symbol
    # (peak appears at the trailing-window symbol end: start + 1016 - 1).
    abs_peak = sync_lo + peaks[0] if peaks else None
    check(peaks and abs_peak is not None
          and sync_lo <= abs_peak < sync_lo + 2 * SAMPLES_PER_SYMBOL,
          f"first peak at sample {abs_peak if abs_peak is not None else 'n/a'} "
          f"inside first SYNC symbol")

    # ---- merged UwbDetector PDU pipeline -----------------------------------
    # Feed the cfile + trailing silence (so the candidate region closes) and
    # check the emitted PDU: precise start, confirmed metric, and captured IQ
    # identical to the ground-truth cfile region.
    import pmt
    x2 = np.concatenate([x, np.zeros(5000, dtype=np.complex64)])
    det = uwb.detector(template.tolist())
    det_src = blocks.vector_source_c(x2.tolist(), False)
    det_dbg = blocks.message_debug()
    tb = gr.top_block()
    tb.connect(det_src, det)
    tb.msg_connect(det, "packet", det_dbg, "store")
    tb.run()
    nmsg = det_dbg.num_messages()
    check(nmsg == 1, f"UwbDetector emitted {nmsg} PDU(s) (expected 1)")
    if nmsg >= 1:
        m = det_dbg.get_message(0)
        meta = pmt.to_python(pmt.car(m))
        iq = np.asarray(pmt.c32vector_elements(pmt.cdr(m)), dtype=np.complex64)
        lo = meta["start_sample"]
        n = meta["sample_count"]
        pre = 2032
        gt = x[lo - pre:lo - pre + n]
        diff = float(np.mean(np.abs(iq - gt)))
        check(lo == PACKET_START,
              f"packet start {lo} == MATLAB ground truth {PACKET_START} (0-based)")
        check(meta.get("threshold") == 0.5,
              f"metadata threshold={meta.get('threshold')} (expected 0.5)")
        check(meta["detection_metric"] > 0.9,
              f"detection metric {meta['detection_metric']:.3f} (> 0.9)")
        check(n == pre + 200000,
              f"capture length {n} = pre_trigger + capture ({pre + 200000})")
        check(diff < 1e-6,
              f"captured IQ matches ground truth (mean diff {diff:.2e})")

    # ---- summary -----------------------------------------------------------
    print()
    if FAIL:
        print(f"FAILED {len(FAIL)} check(s)")
        sys.exit(1)
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
