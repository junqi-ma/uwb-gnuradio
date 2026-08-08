#!/usr/bin/env python3
"""End-to-end Packet Writer + reader verification (numpy equivalent of
read_uwb_packet.m).  Runs detector -> packet_writer on the real UWB signal,
then reads capture.iq / capture.jsonl (SC16) back and checks each packet's IQ
against the ground-truth cfile region within quantization tolerance."""
import json, os, shutil, sys
import numpy as np
from gnuradio import gr, blocks, uwb

ROOT = os.path.dirname(os.path.abspath(__file__))
CFILE = os.path.join(ROOT, "testdata", "uwb_code9_preamble64_payload128_standard_sfd.cfile")
OUT = os.path.join(ROOT, "testdata", "capture_out")
PK = 4992000
PRE = 2032
FAIL = []

def check(cond, msg):
    print(f"[{'PASS' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAIL.append(msg)

def read_packet(iq, jsonl, pid):
    """numpy equivalent of read_uwb_packet.m (SC16 + legacy CF32)."""
    with open(jsonl) as f:
        metas = [json.loads(l) for l in f if l.strip()]
    m = next(m for m in metas if m["packet_id"] == pid)
    if "file" in m:
        path = os.path.join(os.path.dirname(iq), m["file"])
        off = 0
    else:
        path = iq
        off = m["file_offset_samples"]
    fmt = m.get("sample_format", "sc16").lower()
    n = m["sample_count"]
    if fmt == "sc16":
        raw = np.fromfile(path, dtype=np.int16, count=n * 2, offset=off * 4)
        scale = float(m.get("iq_scale", 1.0)) or 1.0
        x = (raw[0::2].astype(np.float64) + 1j * raw[1::2].astype(np.float64)) / scale
        x = x.astype(np.complex64)
    else:
        raw = np.fromfile(path, dtype=np.float32, count=n * 2, offset=off * 8)
        x = (raw[0::2] + 1j * raw[1::2]).astype(np.complex64)
    return x, m

def run_mode(one_file, label):
    shutil.rmtree(OUT, ignore_errors=True)
    x = np.fromfile(CFILE, dtype=np.complex64)
    x2 = np.concatenate([x, np.zeros(5000, dtype=np.complex64)])
    tmpl = x[PK : PK + 1016]
    tmpl = tmpl / np.sqrt(np.sum(np.abs(tmpl) ** 2))
    det = uwb.detector(tmpl.tolist())
    w = uwb.packet_writer(OUT, "capture", one_file)
    src = blocks.vector_source_c(x2.tolist(), False)
    tb = gr.top_block()
    tb.connect(src, det)
    tb.msg_connect(det, "packet", w, "packet")
    tb.run()
    iq = os.path.join(OUT, "capture.iq")
    jsonl = os.path.join(OUT, "capture.jsonl")
    print(f"== {label}: packets_written={w.packets_written()} samples={w.samples_written()}")
    with open(jsonl) as f:
        lines = [l for l in f if l.strip()]
    check(len(lines) >= 1, f"{label}: {len(lines)} jsonl line(s)")
    for pid in range(len(lines)):
        xp, m = read_packet(iq, jsonl, pid)
        check(m.get("sample_format") == "sc16", f"{label} pkt{pid}: sample_format=sc16")
        check("iq_scale" in m and m["iq_scale"] > 0, f"{label} pkt{pid}: iq_scale>0 ({m.get('iq_scale')})")
        lo = m["start_sample"]
        gt = x[lo - PRE : lo - PRE + m["sample_count"]]
        # file size: 4 bytes per SC16 sample
        if "file" in m:
            fpath = os.path.join(OUT, m["file"])
        else:
            fpath = iq
        nbytes = os.path.getsize(fpath)
        if "file" not in m:
            # shared file may hold only this packet in e2e (1 packet)
            check(nbytes == m["sample_count"] * 4, f"{label} pkt{pid}: file size {nbytes} == n*4")
        else:
            check(nbytes == m["sample_count"] * 4, f"{label} pkt{pid}: per-packet size {nbytes} == n*4")
        # Quantization: max error ~ 0.5 / iq_scale
        err = np.abs(xp.astype(np.complex128) - gt.astype(np.complex128))
        mean_diff = float(np.mean(err))
        max_diff = float(np.max(err))
        tol = 1.0 / float(m["iq_scale"])  # ~1 LSB in float units
        check(
            abs(lo - PK) < 2000
            and m["detection_metric"] > 0.9
            and mean_diff < tol
            and max_diff < tol,
            f"{label} pkt{pid}: start={lo} n={m['sample_count']} metric={m['detection_metric']:.3f} "
            f"mean_err={mean_diff:.2e} max_err={max_diff:.2e} tol={tol:.2e}",
        )

run_mode(False, "shared capture.iq (SC16)")
run_mode(True, "one-file-per-packet (SC16)")
print()
if FAIL:
    print(f"FAILED {len(FAIL)}")
    for f in FAIL:
        print(" ", f)
    sys.exit(1)
print("PACKET WRITER + READER OK (SC16)")
