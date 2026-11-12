#!/usr/bin/env python3
"""Paired SC16 737.28M -> production 65/48 FIR -> SIC -> receiver experiment."""
import argparse
import collections
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import statistics
import struct
import subprocess
import tempfile

import numpy as np
from scipy.signal import firwin, resample_poly

PROFILES = {
    'realtime_minorder': 'testdata/resampler_65_48/taps_realtime_minorder.txt',
    'realtime_eq16': 'testdata/resampler_65_48/taps_realtime_eq16.txt',
    'quality_minorder': 'testdata/resampler_65_48/taps_quality_minorder.txt',
    'quality_eq32': 'testdata/resampler_65_48/taps_quality_eq32.txt',
    'realtime_eq_conservative': 'testdata/resampler_65_48_equiripple/taps_realtime_eq_conservative.txt',
    'quality_eq_conservative': 'testdata/resampler_65_48_equiripple/taps_quality_eq_conservative.txt',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--binary', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--seeds', type=int, default=16)
    args = ap.parse_args()
    if args.seeds < 1:
        ap.error('seeds must be positive')
    args.out.mkdir(parents=True, exist_ok=False)
    binary = args.binary.resolve()
    env = dict(os.environ)
    env.pop('LD_LIBRARY_PATH', None)
    # Independent, common native signal synthesis; not the filter under test.
    # scipy compensates this generator FIR delay. Keep the resulting SC16 byte
    # streams identical across all six production interpolation profiles.
    generator_taps = firwin(8193, 1/65, window=('kaiser', 10.0))
    generator_taps.astype('<f8').tofile(args.out/'generator_taps.f64')
    ldd = subprocess.check_output(['ldd', str(binary)], text=True, env=env)
    provenance = dict(commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                      binary=str(binary), binary_sha256=sha(binary), ldd=ldd,
                      seeds=args.seeds, grid='SC16 737.28M -> process_sc16 + flush 65/48 -> CF32 998.4M',
                      generator='scipy resample_poly 48/65, 8193-tap Kaiser beta10, delay compensated',
                      quantization='common scale per clean/raw pair; round-to-nearest; 90% component headroom; zero clipping',
                      time_origin='q + (T-1)/96 output samples; round public acquisition seed only; output unshifted',
                      source_origins='q=512+seed%16 work samples; 16 fractional native-grid phases; public configured origins',
                      comparison='raw/SIC CIR versus same-profile resampled clean; clean CIR versus ideal work reference',
                      sources={str(p): sha(p) for p in [Path(__file__), Path('tools/sic/experiment_preamble_recovery.cc')]},
                      libraries={line.split('=>')[1].strip().split()[0]: sha(line.split('=>')[1].strip().split()[0])
                                 for line in ldd.splitlines() if 'libgnuradio-uwb' in line and '=>' in line},
                      profiles={name: dict(file=p, sha256=sha(p), taps=np.fromfile(p, '<f4').size,
                                           group_delay_outputs=(np.fromfile(p, '<f4').size-1)/96)
                                for name, p in PROFILES.items()})
    manifest = []
    with tempfile.TemporaryDirectory(prefix='uwb-native-recovery-') as scratch_name:
        scratch = Path(scratch_name)
        with open('/tmp/uwb-sic-benchmark.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            with (args.out/'generator.log').open('w') as log:
                subprocess.run([str(binary), str(args.seeds), 'dump', str(scratch)], stdout=log,
                               stderr=subprocess.STDOUT, env=env, check=True)
            for file in sorted(scratch.glob('*.cf32')):
                source = np.fromfile(file, '<c8').astype(np.complex128).reshape(2, -1)
                native = resample_poly(source, 48, 65, axis=1, window=generator_taps)
                peak = max(float(np.max(abs(native.real))), float(np.max(abs(native.imag))))
                gain = .9*32767/peak
                iq = np.empty((*native.shape, 2), dtype='<i2')
                rounded = np.stack([np.rint(native.real*gain), np.rint(native.imag*gain)], axis=-1)
                if np.max(abs(rounded)) > 32767:
                    raise RuntimeError('SC16 clipping')
                iq[:] = rounded
                output = file.with_suffix('.sc16')
                with output.open('wb') as f:
                    f.write(struct.pack('<Qd', native.shape[1], 1/gain))
                    f.write(iq.tobytes())
                manifest.append(dict(case=file.stem, source_sha256=sha(file), native_sha256=sha(output),
                                     native_samples=native.shape[1], inverse_scale=1/gain))
                file.unlink()
            if len(manifest) != 2*7*args.seeds:
                raise RuntimeError('missing source cases')
            print('Native source pairs prepared:', len(manifest), flush=True)
            for profile, taps in PROFILES.items():
                with (args.out/(profile+'.csv')).open('w') as output, (args.out/(profile+'.log')).open('w') as log:
                    subprocess.run([str(binary), str(args.seeds), 'native', str(scratch), taps],
                                   stdout=output, stderr=log, env=env, check=True)
                print('Completed', profile, flush=True)
    (args.out/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    (args.out/'input_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    summary = []
    for profile in PROFILES:
        with (args.out/(profile+'.csv')).open() as f:
            rows = list(csv.DictReader(f))
        if len(rows) != 2*7*args.seeds*4:
            raise RuntimeError('incomplete native matrix')
        groups = collections.defaultdict(list)
        for row in rows:
            groups[row['role'], row['scenario'], row['path']].append(row)
        for (role, scenario, path), group in groups.items():
            valid = [r for r in group if int(r['radar_status']) == 0 and float(r['cir_relative_l2']) >= 0]
            rec = dict(profile=profile, role=role, scenario=scenario, path=path, n=len(group),
                       fcs=sum(int(r['fcs']) for r in group), psdu_exact=sum(int(r['psdu_exact']) for r in group),
                       sfd=sum(int(r['sfd']) for r in group), radar_ok=sum(int(r['radar_status'])==0 for r in group),
                       committed=sum(int(r['sic_committed']) for r in group),
                       reasons=dict(collections.Counter(r['sic_reason'] for r in group)))
            for field in ['cir_peak_gain_db', 'cir_relative_l2', 'cir_peak_shift']:
                values = [float(r[field]) for r in valid]
                rec[field+'_median'] = statistics.median(values) if values else None
                rec[field+'_max_abs'] = max(map(abs, values)) if values else None
            rec['residual_improvement_db_median'] = statistics.median(float(r['residual_improvement_db']) for r in group)
            summary.append(rec)
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    for profile in PROFILES:
        for path in ['clean', 'sic']:
            group = [r for r in summary if r['profile']==profile and r['path']==path]
            print(profile, path, 'PSDU/FCS', sum(min(r['fcs'],r['psdu_exact']) for r in group), '/', sum(r['n'] for r in group),
                  'CIR', sum(r['radar_ok'] for r in group))


if __name__ == '__main__':
    main()
