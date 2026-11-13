#!/usr/bin/env python3
"""Generate the E2E SC16 replay fixtures for the 2026-11-14 low-hanging-fruit ABBA.

The pipeline benchmark (`benchmark_sc16_pipeline_steady`) replays a raw
`complex<int16>` stream and slices it into scheduled windows
`[t0 + k*period - pre_guard, + capacity)`.  This script builds one period block
per mode, tiles it `--windows` times, and writes the raw stream.

Scene generation reuses the already-validated `experiment_preamble_recovery`
mixture (code-9 target full packet + code-10 preamble-only interferer, single
path, SNR 20 dB, ISR 20 dB, zero CFO).  The mixture is dumped on the work grid,
resampled 48/65 to the native 737.28 MS/s grid with scipy (the same conversion
`run_preamble_recovery_matrix.py` uses), and placed so that the packet origin
lands at `pre_guard` inside each period block.

Two fixtures per mode:
  clean : the `clean` buffer (target + noise, no interferer)
  jam   : the `raw` buffer   (target + noise + interferer preamble at ISR 20 dB)

The SC16 scale is one common ADC scale per fixture (0.9 * 32767 / peak), so the
clean and jam variants of a mode share the same absolute levels as the real
capture chain would.
"""

import argparse
import json
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from scipy.signal import firwin, resample_poly

# Same generator filter the native matrix runner uses (independent of the
# pipeline's own 65/48 receiver filter).
GENERATOR_TAPS = 8193
GENERATOR_BETA = 10.0

# mode -> (sync_reps, psdu_bytes, period_native, pre_guard, post_guard, consumer)
MODES = {
    'A64':  dict(sync_reps=64,  psdu=32,  period=127572, pre=400, post=400,
                 consumer='sic_sensing'),
    # Mode A is sensing-only: the target still carries a PSDU so the scene is a
    # real packet, but 32 B keeps the 256-SYNC scene inside the 271,626-sample
    # window the baseline used (a 127 B payload alone is ~324k native samples).
    'A256': dict(sync_reps=256, psdu=32,  period=271626, pre=400, post=400,
                 consumer='sic_sensing'),
    'B64':  dict(sync_reps=64,  psdu=32,  period=257636, pre=400, post=400,
                 consumer='sic_comm'),
    'B256': dict(sync_reps=256, psdu=127, period=401689, pre=400, post=400,
                 consumer='sic_comm'),
}


def dump_scene(binary, sync_reps, psdu, seed, out_dir, env):
    """Work-grid clean/raw CF32 pair for one seed (native_phase => q known)."""
    cmd = [str(binary),
           '--sync-reps', str(sync_reps), '--psdu-bytes', str(psdu),
           '--target-code', '9', '--interferer-code', '10',
           '--interferer-sync-reps', '64',
           '--multipath', 'single', '--isr-db', '20',
           '--target-cfo-hz', '0', '--interferer-cfo-hz', '0',
           '--seeds', str(seed + 1), '--grid', 'work', '--stage', 'dump',
           '--dump-dir', str(out_dir)]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        raise SystemExit('dump failed: ' + (r.stderr.strip().splitlines() or ['?'])[-1])
    files = sorted(out_dir.glob('*.cf32'))
    if len(files) != seed + 1:
        raise SystemExit('dump produced %d files, expected %d' % (len(files), seed + 1))
    return files[seed]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', type=Path, required=True,
                    help='experiment_preamble_recovery')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--windows', type=int, default=8,
                    help='period blocks written per fixture (feeder loops anyway)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--modes', default=','.join(MODES))
    a = ap.parse_args()

    a.out.mkdir(parents=True, exist_ok=True)
    env = dict(__import__('os').environ)
    env.pop('LD_LIBRARY_PATH', None)
    generator = firwin(GENERATOR_TAPS, 1 / 65, window=('kaiser', GENERATOR_BETA))

    meta = {}
    with tempfile.TemporaryDirectory(prefix='lf-fixture-') as tmp:
        for name in a.modes.split(','):
            m = MODES[name]
            src = dump_scene(a.binary, m['sync_reps'], m['psdu'], a.seed,
                             Path(tmp), env)
            stacked = np.fromfile(src, '<c8').astype(np.complex128).reshape(2, -1)
            src.unlink()
            q_work = 512 + (a.seed % 16)          # experiment_preamble_recovery kQ
            native = resample_poly(stacked, 48, 65, axis=1, window=generator)
            n_nat = native.shape[1]
            q_nat = int(round(q_work * 48 / 65))

            period = m['period']
            pre = m['pre']
            post = m['post']
            capture = period - pre - post
            if capture <= 0:
                raise SystemExit('%s: bad period/guards' % name)
            if n_nat + (pre - q_nat) > period:
                raise SystemExit('%s: scene %d does not fit period %d'
                                 % (name, n_nat, period))
            if pre - q_nat < 0:
                raise SystemExit('%s: scene origin %d before block start'
                                 % (name, q_nat - pre))

            peak = max(float(np.max(np.abs(native.real))),
                       float(np.max(np.abs(native.imag))))
            if not peak > 0:
                raise SystemExit('%s: empty scene' % name)
            gain = 0.9 * 32767 / peak

            block = np.zeros((2, period), dtype=np.complex128)
            off = pre - q_nat
            block[:, off:off + n_nat] = native
            rounded = np.stack([np.rint(block.real * gain),
                                np.rint(block.imag * gain)], axis=-1)
            if np.max(np.abs(rounded)) > 32767:
                raise SystemExit('%s: SC16 clipping' % name)
            iq = rounded.astype('<i2')

            for idx, variant in ((0, 'clean'), (1, 'jam')):
                path = a.out / ('%s_%s.sc16' % (name, variant))
                tile = np.tile(iq[idx], (a.windows, 1))
                with path.open('wb') as f:
                    f.write(tile.tobytes())
                meta['%s_%s' % (name, variant)] = dict(
                    path=str(path), sync_reps=m['sync_reps'], psdu=m['psdu'],
                    period=period, pre_guard=pre, post_guard=post,
                    capture=capture, consumer=m['consumer'],
                    t0=pre, native_samples=int(tile.size),
                    scene_native=n_nat, scene_offset=off,
                    inverse_scale=1.0 / gain)
                print('%-10s period=%-7d pre=%-4d capture=%-7d scene=%-7d off=%-4d'
                      % (name + '_' + variant, period, pre, capture, n_nat, off))

    (a.out / 'fixtures.json').write_text(json.dumps(meta, indent=2) + '\n')
    print('wrote', a.out / 'fixtures.json')


if __name__ == '__main__':
    sys.exit(main())
