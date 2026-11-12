#!/usr/bin/env python3
"""Serial functional matrix for experiment_preamble_recovery (not a PER benchmark)."""
import argparse
import collections
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', type=int, default=16)
    args = ap.parse_args()
    if args.seeds < 1:
        ap.error('--seeds must be positive')
    args.out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ)
    env.pop('LD_LIBRARY_PATH', None)
    binary = args.binary.resolve()
    command = [str(binary), str(args.seeds)]
    metadata = {
        'command': command,
        'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'git_diff': subprocess.check_output(['git', 'diff', '--stat'], text=True),
        'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        'ldd': subprocess.check_output(['ldd', str(binary)], env=env, text=True),
        'input_grid': '998.4 MS/s work grid; resampler not exercised',
        'profile': '64 SYNC, 4z2, Legacy pulse, 32-byte PSDU including FCS',
        'channel': 'single path; common nominal origin 512; other-code interference',
        'noise': '20 dB desired-packet-average SNR; paired clean/raw/SIC noise',
        'isr': 'power ratio over first 64 SYNC symbols',
        'online_inputs': 'mixed IQ, public templates, configured coarse ROI +/-64; no truth IQ',
        'scope': 'functional recovery; seeds vary PSDU and noise, fixed timing/channel/phase per cell',
    }
    libraries = []
    for line in metadata['ldd'].splitlines():
        if 'libgnuradio-uwb' in line and '=>' in line:
            lib = Path(line.split('=>')[1].strip().split()[0]).resolve()
            libraries.append({'path': str(lib), 'sha256': hashlib.sha256(lib.read_bytes()).hexdigest()})
    metadata['libraries'] = libraries
    metadata['source_sha256'] = {
        p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
        for p in ['tools/sic/experiment_preamble_recovery.cc', __file__]
    }
    (args.out/'provenance.json').write_text(json.dumps(metadata, indent=2)+'\n')
    with open('/tmp/uwb-sic-benchmark.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        with (args.out/'windows.csv').open('w') as output, (args.out/'stderr.log').open('w') as errors:
            subprocess.run(command, stdout=output, stderr=errors, env=env, check=True)
    with (args.out/'windows.csv').open() as f:
        rows = list(csv.DictReader(f))
    groups = collections.defaultdict(list)
    for row in rows:
        groups[row['role'], row['scenario'], row['path']].append(row)
    summary = []
    for (role, scenario, path), group in groups.items():
        valid = [r for r in group if int(r['radar_status']) == 0]
        rec = dict(role=role, scenario=scenario, path=path, n=len(group),
                   sfd=sum(int(r['sfd']) for r in group),
                   fcs=sum(int(r['fcs']) for r in group),
                   psdu_exact=sum(int(r['psdu_exact']) for r in group),
                   radar_ok=len(valid),
                   committed=sum(int(r['sic_committed']) for r in group),
                   reasons=dict(collections.Counter(r['sic_reason'] for r in group)))
        for field in ['cir_peak_shift', 'cir_peak_gain_db', 'cir_relative_l2']:
            values = [float(r[field]) for r in valid]
            rec[field+'_median'] = statistics.median(values) if values else None
            rec[field+'_max_abs'] = max(map(abs, values)) if values else None
        rec['residual_improvement_db_median'] = statistics.median(float(r['residual_improvement_db']) for r in group)
        summary.append(rec)
    (args.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    for r in summary:
        if r['path'] != 'clean':
            print(r['role'], r['scenario'], r['path'],
                  f"FCS={r['fcs']}/{r['n']} PSDU={r['psdu_exact']}/{r['n']} CIR={r['radar_ok']}/{r['n']}")
    if len(rows) != 2*7*args.seeds*3:
        raise RuntimeError('incomplete experiment output')
    if any(r['fcs'] != r['n'] or r['psdu_exact'] != r['n'] or r['radar_ok'] != r['n']
           for r in summary if r['path'] == 'clean'):
        raise RuntimeError('clean reference failed; inspect output before interpreting recovery')


if __name__ == '__main__':
    main()
