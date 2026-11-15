#!/usr/bin/env python3
"""Report the X410 <-> transmitter sample-clock offset from a captured SC16 file.

`uwb_dw3000_live` prints `spc_ppm` per packet (the measured chip period relative
to the nominal 2.0 samples/chip).  This script runs the receiver and summarises
that column, so a crystal trim can be verified with one command.

Interpretation:
  mean   - the systematic clock offset.  Trimming the transmitter crystal should
           move this toward 0.
  stdev  - the estimation noise of the 256-peak least-squares fit.  It does NOT
           go away with trimming; it is a floor set by the estimator, not the
           hardware.

Exit code 0 always; this is a measurement tool, not a gate.
"""
import argparse
import re
import statistics
import subprocess
import sys
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('binary', help='path to uwb_dw3000_live')
ap.add_argument('capture', help='SC16 capture to replay')
ap.add_argument('--workers', type=int, default=4)
ap.add_argument('--sync', default='locked', choices=['acquire', 'locked'])
ap.add_argument('--fused', default='off', choices=['on', 'off'])
ap.add_argument('--tone', default='on', choices=['on', 'off'])
ap.add_argument('--period-ms', type=float, default=5.0,
                help='packet period of the transmitter (default 5 ms)')
a = ap.parse_args()

cmd = [a.binary, '--replay', a.capture, '--workers', str(a.workers),
       '--sync', a.sync, '--fused', a.fused, '--tone', a.tone,
       '--period-ms', str(a.period_ms), '--summary', '100000']
r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
if r.returncode != 0:
    sys.exit('%s failed:\n%s\n%s' % (a.binary, r.stdout[-2000:], r.stderr[-2000:]))

rows = [line for line in r.stdout.splitlines() if line.startswith('PACKET')]
ok = [line for line in rows if 'status=OK' in line]
vals = []
cfos = []
for line in ok:
    m = re.search(r'spc_ppm=([-0-9.]+)', line)
    if m:
        vals.append(float(m.group(1)))
    c = re.search(r'cfo_khz=([-0-9.]+)', line)
    if c:
        cfos.append(float(c.group(1)))
if not vals:
    sys.exit('no decodable packet in %s (%d attempts)' % (a.capture, len(rows)))

mean = statistics.fmean(vals)
sd = statistics.pstdev(vals)
fin = [line for line in r.stdout.splitlines() if line.startswith('FINAL')]
fcs = re.search(r'fcs=(\d+/\d+)', fin[0]).group(1) if fin else '?'

LO_HZ = 6489.6e6
print('capture        : %s' % Path(a.capture).name)
print('period         : %.3f ms' % a.period_ms)
print('packets        : %d decoded / %d attempted, FCS %s' % (len(vals), len(rows), fcs))
print()
print('--- sample clock (symbol rate) ---')
print('spc_ppm mean   : %+.4f ppm   <- systematic clock offset' % mean)
print('spc_ppm stdev  :  %.4f ppm   <- estimator noise (not removable by trim)' % sd)
print('spc_ppm range  : %+.4f .. %+.4f ppm' % (min(vals), max(vals)))
print('payload drift  : %.2f samples over 154624 (127 B payload)'
      % (abs(mean) * 1e-6 * 154624.0))
print()
if cfos:
    cmean = statistics.fmean(cfos)
    print('--- carrier ---')
    print('cfo            : %+.2f kHz  (range %+.2f .. %+.2f)'
          % (cmean, min(cfos), max(cfos)))
    print('RX LO (set)    : %.6f MHz' % (LO_HZ / 1e6))
    print('TX carrier     : %.6f MHz  (%+.1f kHz from nominal)'
          % ((LO_HZ + cmean * 1e3) / 1e6, cmean))
    print('cross-check    : %.2f ppm of LO   vs spc_ppm %+.2f ppm'
          % (cmean * 1e3 / LO_HZ * 1e6, mean))
    print()
print('fused-CMF gate : |spc-2| <= 1e-9  =  0.0005 ppm')
if abs(mean) <= 0.0005:
    print('gate           : PASS')
else:
    print('gate           : FAIL (%.0fx over)' % (abs(mean) / 0.0005))
