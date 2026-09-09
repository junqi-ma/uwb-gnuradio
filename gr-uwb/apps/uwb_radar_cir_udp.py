#!/usr/bin/env python3
"""HRP synth -> loopback echo -> CIR estimator -> disk + UDP.

Matches gr-uwb/examples/uwb_radar_cir_udp.grc.  Default destination
133.133.133.132:12345.  Payload is raw CIR c32vector bytes (116 taps /
928 B); PMT metadata is not sent.
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import os
import sys
import time

from gnuradio import blocks, gr, network
import pmt


def find_repo_root():
    cur = os.path.abspath(os.path.dirname(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(cur, "testdata", "uwb_radar")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise SystemExit("cannot find repo root from %s" % __file__)


def load_uwb():
    repo = find_repo_root()
    bindir = os.path.join(repo, "gr-uwb", "build", "python", "uwb", "bindings")
    matches = sorted(glob.glob(os.path.join(bindir, "uwb_python*.so")))
    if not matches:
        bindir = os.path.join(repo, "gr-uwb", "build", "test_modules",
                              "gnuradio", "uwb")
        matches = sorted(glob.glob(os.path.join(bindir, "uwb_python*.so")))
    if not matches:
        raise SystemExit("missing uwb_python*.so; rebuild gr-uwb python bindings")
    spec = importlib.util.spec_from_file_location("uwb_python", matches[-1])
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


uwb = load_uwb()


def hex_to_bytes(s):
    h = "".join(c for c in s if c in "0123456789abcdefABCDEF")
    if len(h) % 2:
        raise ValueError("psdu hex must have even length")
    return list(bytes.fromhex(h))


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--udp-host", default="133.133.133.132")
    p.add_argument("--udp-port", default="12345")
    p.add_argument("--no-udp", action="store_true")
    p.add_argument("--output", default="/tmp/uwb-radar-cir-udp")
    p.add_argument("--pri-s", type=float, default=0.005)
    p.add_argument("--seconds", type=float, default=3.0,
                   help="Run duration; 0 waits forever")
    p.add_argument("--sync-reps", type=int, default=64)
    p.add_argument("--psdu-hex",
                   default="47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221")
    p.add_argument("--template", default="")
    return p.parse_args()


def main():
    a = parse_args()
    repo = find_repo_root()
    os.makedirs(a.output, exist_ok=True)
    tmpl = a.template or os.path.join(
        repo, "testdata", "uwb_radar", "sync_template_998p4.cf32")
    if not os.path.isfile(tmpl):
        raise SystemExit("missing SYNC template %s" % tmpl)

    src = uwb.hrp_packet_source(hex_to_bytes(a.psdu_hex), a.sync_reps, "4z2",
                                9, 0.8, a.pri_s, False, True, False)
    echo = uwb.loopback_echo(1997, 4096, (251.5, 1200.0),
                             (complex(0.8), complex(0.35)),
                             0.0, 1, 4194304, 4194304, 0, 0.0, a.pri_s)
    est = uwb.radar_cir_estimator(tmpl, a.sync_reps, "4z2", 9, 16, 100, 10, 0,
                                  64, 8, 0.3, 0.3, True, 64)
    wr = uwb.cir_writer(a.output, "cir", True, 64)
    strobe = blocks.message_strobe(pmt.make_dict(), int(a.pri_s * 1000))

    sock = None
    if not a.no_udp:
        sock = network.socket_pdu("UDP_CLIENT", a.udp_host, str(a.udp_port),
                                  1472)
        print("udp_cir %s:%s mtu=1472 taps_only" % (a.udp_host, a.udp_port),
              flush=True)

    tb = gr.top_block("uwb_radar_cir_udp")
    tb.msg_connect((strobe, "strobe"), (src, "emit"))
    tb.msg_connect((src, "tx"), (echo, "tx"))
    tb.msg_connect((echo, "rx"), (est, "rx"))
    tb.msg_connect((est, "cir"), (wr, "cir"))
    if sock is not None:
        tb.msg_connect((est, "cir"), (sock, "pdus"))

    print("start pri_s=%.4f seconds=%s template=%s" % (
        a.pri_s, a.seconds, tmpl), flush=True)
    tb.start()
    try:
        if a.seconds and a.seconds > 0:
            time.sleep(a.seconds)
        else:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    tb.stop()
    tb.wait()
    try:
        wr.stop()
    except Exception:
        pass
    print("SUMMARY wr_ok=%d wr_fail=%d est_done=%d est_fail=%d est_drop=%d"
          % (wr.frames_written(), wr.frames_failed(), est.pdus_completed(),
             est.pdus_failed(), est.pdus_dropped()), flush=True)
    ok = wr.frames_written() > 0 and est.pdus_failed() == 0
    raise SystemExit(0 if ok else 3)


if __name__ == "__main__":
    main()
