#!/usr/bin/env python3
"""Compare fixed-window decoder JSONL, excluding the elapsed demod_us field."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('baseline')
    ap.add_argument('current')
    ap.add_argument('--fixtures', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--packets', type=int, default=200)
    args = ap.parse_args()
    if args.packets < 2:
        ap.error('at least two packets required')
    root = Path(__file__).resolve().parents[3]
    args.out.mkdir(parents=True, exist_ok=True)
    seeds = args.out/'starts.txt'
    seeds.write_text(''.join(f'{i*737280}\n' for i in range(args.packets)))
    env = os.environ.copy()
    env.pop('LD_LIBRARY_PATH', None)
    summary = {'binaries': {p: sha256(p) for p in (args.baseline, args.current)},
               'packets_per_fixture': args.packets, 'cases': {}}
    for name in ('f4', 'fm12_200', 'fm14'):
        fixture = args.fixtures/(name+'.sc16')
        rows = []
        for side, binary in (('A', args.baseline), ('B', args.current)):
            cmd = [binary, str(fixture), str(seeds),
                   str(root/'testdata/reference_preamble_code10_998p4.cf32'),
                   str(root/'testdata/resampler_65_48/taps_realtime_eq16.txt'),
                   '256', '4z2', '36864', '450000']
            done = subprocess.run(cmd, env=env, check=True, text=True, capture_output=True)
            (args.out/(name+'_'+side+'.jsonl')).write_text(done.stdout)
            data = [json.loads(line) for line in done.stdout.splitlines()]
            if len(data) != args.packets:
                raise RuntimeError(f'{side}/{name}: expected {args.packets} rows')
            for row in data:
                row.pop('demod_us', None)
            rows.append(data)
        differences = [i for i, (a,b) in enumerate(zip(*rows)) if a != b]
        summary['cases'][name] = {'sha256': sha256(fixture),
            'rows': len(rows[0]), 'differences': differences,
            'fcs': [sum(r.get('fcs', 0) for r in side) for side in rows]}
        print(name, summary['cases'][name], flush=True)
    (args.out/'comparison.json').write_text(json.dumps(summary, indent=2)+'\n')
    if any(c['differences'] for c in summary['cases'].values()):
        raise SystemExit('packet outputs differ')

if __name__ == '__main__':
    main()
