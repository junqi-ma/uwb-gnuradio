#!/usr/bin/env python3
"""Capture a device-timed SC16 burst from the X410 and report the clock offset.

One command, two jobs:

  1. capture N ms of native-rate SC16 to a file (device-timed NUM_SAMPS_AND_DONE,
     not a host sleep), and
  2. print the metadata needed to interpret it.

The receiver's own clock-offset measurement is done separately by
`uwb_dw3000_live`, which prints `spc_ppm` per packet.  This script only records
the capture so that measurement is reproducible.

Usage:
  capture_x410_sc16.py OUT.sc16 [--seconds 0.2] [--channel 3] [--antenna RX1]
                       [--freq 6489.6e6] [--gain 60] [--args ...]
"""
import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, '/usr/local/lib/python3.10/site-packages')
import numpy as np
import uhd

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('out', type=Path)
ap.add_argument('--seconds', type=float, default=0.2)
ap.add_argument('--channel', type=int, default=3)
ap.add_argument('--antenna', default='RX1')
ap.add_argument('--freq', type=float, default=6489.6e6)
ap.add_argument('--gain', type=float, default=60.0)
ap.add_argument('--rate', type=float, default=737.28e6)
ap.add_argument('--lead', type=float, default=0.2,
                help='device-time lead before the burst starts (s)')
ap.add_argument('--args', default='addr=192.168.10.2,recv_frame_size=8000,'
                                 'num_recv_frames=32768')
a = ap.parse_args()

u = uhd.usrp.MultiUSRP(a.args)
ch = a.channel
u.set_rx_rate(a.rate, ch)
u.set_rx_freq(a.freq, ch)
u.set_rx_gain(a.gain, ch)
u.set_rx_antenna(a.antenna, ch)
rate = u.get_rx_rate(ch)
n = int(round(rate * a.seconds))

sa = uhd.usrp.StreamArgs('sc16', 'sc16')
sa.channels = [ch]
stream = u.get_rx_stream(sa)
buf = np.empty((1, n), np.int32)
buf.fill(0)  # fault in the whole receive buffer before streaming

md = uhd.types.RXMetadata()
cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
cmd.num_samps = n
cmd.stream_now = False
cmd.time_spec = u.get_time_now() + uhd.types.TimeSpec(a.lead)
stream.issue_stream_cmd(cmd)

got = 0
errors = []
chunks = []
t0 = time.monotonic()
while got < n:
    k = stream.recv(buf[:, got:min(n, got + 1048576)], md, 1.0)
    if md.error_code != uhd.types.RXMetadataErrorCode.none:
        errors.append({'at': got, 'error': str(md.error_code),
                       'text': md.strerror()})
    if k:
        chunks.append({'at': got, 'n': k,
                       'time': md.time_spec.get_real_secs()})
        got += k
    if time.monotonic() - t0 > 8 or (not k and errors):
        break
stream.issue_stream_cmd(uhd.types.StreamCMD(uhd.types.StreamMode.stop_cont))

a.out.parent.mkdir(parents=True, exist_ok=True)
buf[0, :got].tofile(a.out)
meta = dict(rate=rate, freq=u.get_rx_freq(ch), gain=u.get_rx_gain(ch),
            channel=ch, antenna=u.get_rx_antenna(ch), requested=n, got=got,
            duration_ms=got / rate * 1000.0, errors=errors,
            chunks=len(chunks), file=str(a.out))
Path(str(a.out) + '.json').write_text(json.dumps(meta, indent=2) + '\n')
print(json.dumps(meta, indent=2))
