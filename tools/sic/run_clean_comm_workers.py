#!/usr/bin/env python3
"""Clean comm receive (no interference, no SIC/sensing) worker-count matrix.

Signal: one code-9 packet, configurable SYNC repetitions and PSDU bytes, with an
injected carrier offset applied at work rate.  Each job is
SC16 resample (737.28 -> 998.4 MS/s) + complete comm demod/FCS.  No SIC and no
sensing job is prepared or executed, so the numbers are the receive chain of an
interference-free packet, not the mixed SIC scenario.
"""
import argparse
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import tempfile

import numpy as np
from scipy.signal import firwin, resample_poly

CORES = [4, 6, 8, 10, 12, 14, 16, 18]
WORK_RATE = 998.4e6


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def percentiles(values):
    a = sorted(values)
    return dict(mean=statistics.fmean(a), p50=statistics.median(a),
                p95=a[int(np.ceil(.95 * len(a))) - 1])


def median_of(dicts, key, field):
    return statistics.median(d[key][field] for d in dicts)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build', type=Path, default=Path('build/sc16-r1r2'))
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--measure', type=int, default=512)
    ap.add_argument('--warmup', type=int, default=128)
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--sync-reps', type=int, default=256)
    ap.add_argument('--psdu-bytes', type=int, default=127)
    ap.add_argument('--cfo-hz', type=float, nargs='+', default=[50000.0, 0.0])
    ap.add_argument('--profile', default='quality_minorder')
    ap.add_argument('--workers', type=int, nargs='+', default=[1, 2, 4])
    args = ap.parse_args()
    if not 2 <= args.psdu_bytes <= 127:
        ap.error('PSDU including FCS must fit [2,127] bytes')
    if min(args.measure, args.warmup, args.reps) < 1:
        ap.error('measure/warmup/reps must be positive')
    if any(w < 1 for w in args.workers):
        ap.error('workers must be >= 1')
    args.out.mkdir(parents=True, exist_ok=False)
    build = args.build.resolve()
    env = dict(os.environ)
    env.pop('LD_LIBRARY_PATH', None)
    binaries = {name: build / 'apps' / name for name in
                ['experiment_preamble_recovery', 'benchmark_chain_workers']}
    metadata = dict(
        commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        cores=CORES, parameters=vars(args) | {'build': str(build), 'out': str(args.out)},
        binaries={name: dict(path=str(p), sha256=sha(p),
                             ldd=subprocess.check_output(['ldd', str(p)], env=env, text=True))
                  for name, p in binaries.items()},
        sources={str(p): sha(p) for p in [Path(__file__), Path('tools/sic/benchmark_chain_workers.cc'),
                                         Path('tools/sic/experiment_preamble_recovery.cc')]},
        topology=subprocess.check_output(['lscpu', '-e=CPU,CORE,SOCKET,ONLINE'], text=True),
        scenario=(f'clean code9 packet only, no interference/noise/SIC/sensing; '
                  f'{args.sync_reps} SYNC, {args.psdu_bytes} PSDU bytes incl FCS; '
                  f'carrier offset injected at work rate: {args.cfo_hz} Hz; '
                  f'16 PSDUs/origins'),
        scope=('SC16 resample 737.28->998.4 MS/s + complete comm decode/FCS per job; '
               'excludes RX, queue and commit'))
    for info in metadata['binaries'].values():
        info['libraries'] = {line.split('=>')[1].strip().split()[0]: sha(line.split('=>')[1].strip().split()[0])
                             for line in info['ldd'].splitlines() if 'libgnuradio-uwb' in line and '=>' in line}
    (args.out / 'provenance.json').write_text(json.dumps(metadata, indent=2) + '\n')

    results = []
    with tempfile.TemporaryDirectory(prefix='uwb-clean-comm-') as temporary:
        directory = Path(temporary)
        with open('/tmp/uwb-sic-benchmark.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            with (args.out / 'generator.log').open('w') as log:
                subprocess.run([str(binaries['experiment_preamble_recovery']), '16', 'dump', temporary,
                                'clean_control', str(args.sync_reps), str(args.psdu_bytes), str(args.sync_reps)],
                               stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
            taps = firwin(8193, 1 / 65, window=('kaiser', 10.0))
            work_pool = []
            source_hashes = []
            for seed in range(16):
                path = directory / f'9_{seed}_clean_control.cf32'
                pair = np.fromfile(path, '<c8').astype(np.complex128).reshape(2, -1)
                clean, raw = pair[0], pair[1]
                if not np.array_equal(clean, raw):
                    raise RuntimeError(f'clean_control halves differ: {path}')
                work_pool.append(raw)
                source_hashes.append(sha(path))
            # One ADC scale for the entire pool, no burst-wise renormalization.
            native_pool = [resample_poly(x, 48, 65, window=taps) for x in work_pool]
            gain = .9 * 32767 / max(max(abs(a.real).max(), abs(a.imag).max()) for a in native_pool)
            count = max(map(len, native_pool)) + 2048
            pool = np.zeros((16, count, 2), dtype='<i2')
            fixture = dict(native_samples_per_window=count, source_pool=16,
                           source_sha256=source_hashes, adc_gain=float(gain), core_scale=1 / 32768,
                           origin='q=512+seed work samples', native_t0=400, pre_guard=400,
                           post_guard=400, period=count, sync_reps=args.sync_reps,
                           psdu_bytes=args.psdu_bytes, cfo_hz=args.cfo_hz,
                           payload_hex=[(directory / f'psdu_9_{i}.bin').read_bytes().hex() for i in range(16)])
            (directory / 'geometry.txt').write_text(f'{args.sync_reps} {args.sync_reps}\n')
            (args.out / 'fixture.json').write_text(json.dumps(fixture, indent=2) + '\n')
            print('Prepared clean native pool:', count, 'samples/window', flush=True)
            fixtures = {}
            for cfo in args.cfo_hz:
                # Inject the carrier offset on the work-rate signal before the
                # 48/65 native synthesis, so the native fixture carries it and
                # the production 65/48 resampler must recover it.
                w = 2 * np.pi * cfo / WORK_RATE
                for seed, x in enumerate(work_pool):
                    phase = np.exp(1j * w * np.arange(len(x)))
                    modulated = x * phase
                    native = resample_poly(modulated, 48, 65, window=taps)
                    rounded = np.stack([np.rint(native.real * gain), np.rint(native.imag * gain)], axis=-1)
                    if abs(rounded).max() > 32767:
                        raise RuntimeError('SC16 clipping')
                    pool[seed, :len(native)] = rounded
                tag = f'cfo{int(round(cfo))}'
                mixture = directory / f'mixture_{tag}.sc16'
                pool.tofile(mixture)
                fixtures[cfo] = (tag, mixture)
                fixture.setdefault('mixture_sha256', {})[tag] = sha(mixture)
                (args.out / 'fixture.json').write_text(json.dumps(fixture, indent=2) + '\n')
            # Interleave CFO blocks across repetitions so a slow thermal drift
            # cannot masquerade as a CFO effect.
            for rep in range(args.reps):
                cfos = list(args.cfo_hz) if rep % 2 == 0 else list(reversed(args.cfo_hz))
                for cfo in cfos:
                    tag, mixture = fixtures[cfo]
                    # The benchmark loads mixture.sc16 by fixed name.
                    (directory / 'mixture.sc16').write_bytes(mixture.read_bytes())
                    order = list(args.workers) if rep % 2 == 0 else list(reversed(args.workers))
                    for workers in order:
                        cores = ','.join(map(str, CORES[:workers]))
                        name = f'clean_comm_{args.profile}_{tag}_n{workers}_r{rep + 1}'
                        dest = args.out / (name + '.csv')
                        cmd = [str(binaries['benchmark_chain_workers']), temporary,
                               'testdata/resampler_65_48/taps_' + args.profile + '.txt',
                               str(count), cores, str(args.warmup), str(args.measure), '16', 'clean_comm']
                        with dest.open('w') as output, (args.out / (name + '.log')).open('w') as log:
                            subprocess.run(['taskset', '-c', '0', *cmd], stdout=output, stderr=log,
                                           env=env, check=True)
                        with dest.open() as f:
                            rows = list(csv.DictReader(f))
                        if len(rows) != args.measure:
                            raise RuntimeError(f'incomplete result: {name}')
                        if not all(int(r['fcs']) and int(r['psdu_exact']) for r in rows):
                            raise RuntimeError(f'comm decode failure: {name}')
                        interval = (max(float(r['end_us']) for r in rows) -
                                    min(float(r['start_us']) for r in rows)) / len(rows)
                        row = dict(cfo_hz=cfo, workers=workers, rep=rep + 1, interval_us=interval,
                                   cfo_est_hz=percentiles([float(r['cfo_est_hz']) for r in rows]),
                                   detected_start=percentiles([float(r['detected_start']) for r in rows]))
                        for field in rows[0]:
                            if field.endswith('_us') and field not in ('start_us', 'end_us'):
                                row[field] = percentiles([float(r[field]) for r in rows])
                        row['worker_jobs'] = {str(w): sum(1 for r in rows if int(r['worker']) == w)
                                              for w in sorted({int(r['worker']) for r in rows})}
                        row['command'] = cmd
                        results.append(row)
                        (args.out / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
                        print(name, 'interval', round(interval, 2), 'us/window', flush=True)
            aggregate = []
            for cfo in args.cfo_hz:
                for workers in args.workers:
                    group = [r for r in results if r['cfo_hz'] == cfo and r['workers'] == workers]
                    entry = dict(cfo_hz=cfo, workers=workers, reps=len(group),
                                 interval_us=statistics.median(r['interval_us'] for r in group),
                                 cfo_est_hz=median_of(group, 'cfo_est_hz', 'mean'),
                                 detected_start=median_of(group, 'detected_start', 'mean'))
                    for field, value in group[0].items():
                        if field.endswith('_us') and field not in ('start_us', 'end_us') and isinstance(value, dict):
                            entry[field] = {k: median_of(group, field, k) for k in ('mean', 'p50', 'p95')}
                    aggregate.append(entry)
            (args.out / 'aggregate.json').write_text(json.dumps(aggregate, indent=2) + '\n')


if __name__ == '__main__':
    main()
