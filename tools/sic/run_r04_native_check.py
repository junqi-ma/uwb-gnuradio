#!/usr/bin/env python3
"""Minimal native-grid functional check for the R04 rectification.

Runs the 4 cells (2 configs x 2 interference directions) x 8 seeds on the real
65/48 native resampling chain, zero CFO, ISR 20 dB, single path, and prints the
clean/raw/sic pass counts.  Reuses the matrix runner's dump -> scipy -> SC16
conversion so the native source generation is identical to the full matrix.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import run_preamble_recovery_matrix as m  # noqa: E402

binary = Path(sys.argv[1])
out_dir = Path(sys.argv[2])
seeds = int(sys.argv[3]) if len(sys.argv) > 3 else 8
taps = Path('testdata/resampler_65_48/taps_quality_minorder.txt')
out_dir.mkdir(parents=True, exist_ok=True)

cells = []
for config in [(64, 32), (256, 127)]:
    for code in [(9, 10), (10, 9)]:
        cells.append(dict(config=config, code=code, multipath='single',
                          isr=20.0, target_cfo=0.0, interferer_cfo=0.0))

env = dict(os.environ)
env.pop('LD_LIBRARY_PATH', None)

ok_all = True
for cell in cells:
    ok, reason = m.run_cell(binary, cell, 'native', seeds, out_dir, env, taps)
    cid = m.case_id(cell)
    print(('OK   ' if ok else 'FAIL ') + cid + ('' if ok else '  ' + reason))
    ok_all = ok_all and ok

print('cells_ok=%d/%d' % (sum(1 for _ in cells) if ok_all else -1, len(cells)))
sys.exit(0 if ok_all else 1)
