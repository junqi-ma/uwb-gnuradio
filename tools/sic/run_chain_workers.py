#!/usr/bin/env python3
"""Serial worker-count matrix: production pipeline and full DSP incl. comm RX."""
import argparse
import collections
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import statistics
import subprocess
import tempfile

import numpy as np
from scipy.signal import firwin, resample_poly

PROFILES = ['quality_minorder', 'quality_eq32', 'realtime_minorder', 'realtime_eq16']
CORES = [4, 6, 8, 10, 12, 14, 16, 18]


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def percentiles(values):
    a = sorted(values)
    return dict(mean=statistics.fmean(a), p50=statistics.median(a), p95=a[int(np.ceil(.95*len(a)))-1])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build', type=Path, default=Path('gr-uwb/build-sc16-r1r2'))
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--measure', type=int, default=512)
    ap.add_argument('--warmup', type=int, default=128)
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--sync-reps', type=int, choices=[64,256], default=64)
    ap.add_argument('--sic-sync-reps', type=int, choices=[64,256], default=None)
    ap.add_argument('--psdu-bytes', type=int, default=32,
                    help='total PSDU bytes including the 2-byte FCS, maximum 127')
    ap.add_argument('--single-worker-detail', action='store_true',
                    help='independent sensing / profiled sensing / comm DSP jobs; one worker only')
    args = ap.parse_args()
    if args.sic_sync_reps is None:
        args.sic_sync_reps=args.sync_reps
    if not 2<=args.psdu_bytes<=127:
        ap.error('PSDU including FCS must fit [2,127] bytes')
    if min(args.measure, args.warmup, args.reps) < 1:
        ap.error('measure/warmup/reps must be positive')
    args.out.mkdir(parents=True, exist_ok=False)
    build = args.build.resolve()
    env = dict(os.environ)
    env.pop('LD_LIBRARY_PATH', None)
    binaries = {name: build/'apps'/name for name in [
        'experiment_preamble_recovery', 'benchmark_sc16_pipeline_steady', 'benchmark_chain_workers']}
    metadata = dict(commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                    cores=CORES, rx_core=2, coordinator_core=0,
                    parameters=vars(args) | {'build': str(build), 'out': str(args.out)},
                    binaries={name: dict(path=str(p), sha256=sha(p),
                                         ldd=subprocess.check_output(['ldd', str(p)], env=env, text=True))
                              for name, p in binaries.items()},
                    sources={str(p): sha(p) for p in [Path(__file__), Path('tools/sic/benchmark_chain_workers.cc'),
                                                     Path('tools/sic/experiment_preamble_recovery.cc')]},
                    topology=subprocess.check_output(['lscpu', '-e=CPU,CORE,SOCKET,ONLINE'], text=True),
                    scenario=f'complete code9 target {args.sync_reps} SYNC/{args.psdu_bytes} PSDU bytes incl FCS + code10 {args.sic_sync_reps} SYNC/SFD-only interference; ISR20, SNR20, zero CFO; 16 PSDUs/noise/origins',
                    cir_repetitions=args.sync_reps-10, sic_fit_repetitions=8,
                    full_dsp_scope='SC16 resampler + preamble SIC + sensing CIR + complete comm decode/FCS, direct pull jobs; excludes RX/queue/commit',
                    pipeline_scope='production RX+assembly+queue+SC16 resampler+preamble SIC+sensing CIR+ordered commit; excludes complete comm decode')
    for info in metadata['binaries'].values():
        info['libraries'] = {line.split('=>')[1].strip().split()[0]: sha(line.split('=>')[1].strip().split()[0])
                             for line in info['ldd'].splitlines() if 'libgnuradio-uwb' in line and '=>' in line}
    (args.out/'provenance.json').write_text(json.dumps(metadata, indent=2)+'\n')
    results = []
    pipeline_commands = {}
    with tempfile.TemporaryDirectory(prefix='uwb-chain-workers-') as temporary:
        directory = Path(temporary)
        with open('/tmp/uwb-sic-benchmark.lock', 'a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            with (args.out/'generator.log').open('w') as log:
                subprocess.run([str(binaries['experiment_preamble_recovery']), '16', 'dump', temporary, 'sync_isr20',
                                str(args.sync_reps),str(args.psdu_bytes),str(args.sic_sync_reps)],
                               stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
            taps = firwin(8193, 1/65, window=('kaiser', 10.0))
            sources = []
            source_hashes = []
            for seed in range(16):
                path = directory/f'9_{seed}_sync_isr20.cf32'
                source = np.fromfile(path, '<c8').astype(np.complex128).reshape(2, -1)[1]
                native = resample_poly(source, 48, 65, window=taps)
                sources.append(native)
                source_hashes.append(sha(path))
            # One ADC scale for the entire pool, no burst-wise renormalization.
            gain = .9*32767/max(max(abs(a.real).max(), abs(a.imag).max()) for a in sources)
            count = max(map(len, sources))+2048
            pool = np.zeros((16, count, 2), dtype='<i2')
            for seed, a in enumerate(sources):
                rounded = np.stack([np.rint(a.real*gain), np.rint(a.imag*gain)], axis=-1)
                if abs(rounded).max()>32767:
                    raise RuntimeError('SC16 clipping')
                pool[seed, :len(a)] = rounded
            pool.tofile(directory/'mixture.sc16')
            (directory/'geometry.txt').write_text(f'{args.sync_reps} {args.sic_sync_reps}\n')
            (args.out/'fixture.json').write_text(json.dumps(dict(native_samples_per_window=count,
                source_pool=16, source_sha256=source_hashes, mixture_sha256=sha(directory/'mixture.sc16'),
                adc_gain=float(gain), core_scale=1/32768, origin='q=512+seed work samples',
                native_t0=400, pre_guard=400, post_guard=400, period=count,
                sync_reps=args.sync_reps,sic_sync_reps=args.sic_sync_reps,psdu_bytes=args.psdu_bytes,
                payload_hex=[(directory/f'psdu_9_{i}.bin').read_bytes().hex() for i in range(16)]), indent=2)+'\n')
            print('Prepared valid mixed native pool:', count, 'samples/window', flush=True)
            # Outer lock covers every child: pipeline child must not acquire it
            # a second time. Window feeder throttling is benchmark-only.
            for rep in range(args.reps):
                order = [1] if args.single_worker_detail else ([1, 2, 4, 8] if rep%2==0 else [8, 4, 2, 1])
                for profile in PROFILES:
                    for workers in order:
                        cores = ','.join(map(str, CORES[:workers]))
                        kinds=['pipeline','full_dsp']
                        if args.single_worker_detail:
                            kinds=['sensing','sensing_detail','comm'] if rep%2==0 else ['comm','sensing_detail','sensing']
                        for kind in kinds:
                            name = f'{kind}_{profile}_n{workers}_r{rep+1}'
                            if kind=='pipeline':
                                dest = args.out/(name+'.json')
                                cmd = [str(binaries['benchmark_sc16_pipeline_steady']), '--file', str(directory/'mixture.sc16'),
                                    '--native-rate', '737280000', '--work-rate', '998400000',
                                    '--capture', str(count-800), '--pre-guard', '400', '--post-guard', '400',
                                    '--period', str(count), '--t0', '400', '--consumer', 'both',
                                    '--sic-policy', 'preamble', '--sic-backend', 'ipatov', '--sic-comm-code', '10',
                                    '--target-sync-reps',str(args.sync_reps),'--sic-sync-reps',str(args.sic_sync_reps),
                                    '--sic-sync-template', str(directory/'sync_10.bin'),
                                    '--resampler-profile', profile, '--dsp-workers', str(workers), '--cpu-rx', '2', '--cpu-dsp', cores,
                                    '--pool-size', '16', '--flow-in-flight', str(8*workers),
                                    '--warmup-windows', str(args.warmup), '--measure-windows', str(args.measure), '--json', str(dest)]
                                pipeline_commands[profile,workers]=list(cmd)
                                with (args.out/(name+'.log')).open('w') as log:
                                    subprocess.run(['taskset','-c','0',*cmd], stdout=log, stderr=subprocess.STDOUT, env=env, check=True)
                                data = json.loads(dest.read_text())
                                ws = [w for w in data['windows'] if w['state']=='Completed'][args.warmup:]
                                ledger=data['ledger']
                                if len(ws)!=args.measure or ledger['dropped_admission'] or not all(w['sic_committed'] and w['radar_sfd_found'] for w in ws):
                                    raise RuntimeError('invalid pipeline functional or accounting result: '+name)
                                row = dict(kind=kind, profile=profile, workers=workers, rep=rep+1,
                                           interval_us=data['meta']['steady_window_us'], startup_ms=data['meta']['startup_ready_ms'])
                                for field in ['resample_us','sic_us','sensing_us','e2e_us','queue_us']:
                                    row[field]=percentiles([w['timing'][field] for w in ws])
                                # Sensing provenance/timers are top-level window
                                # fields (not under "timing").  radar_frontend_us
                                # is timing+cfo+sfd; coverage planning and the
                                # final-CFO derotation are children of the SFD
                                # parent and are never added again.
                                for field in ['radar_frontend_us','radar_cir_us','radar_other_us']:
                                    row[field]=percentiles([w[field] for w in ws])
                                row['radar_cfo_hz']=percentiles([w['radar_cfo_hz'] for w in ws])
                                row['radar_valid_reps']=sorted(set(w['radar_valid_reps'] for w in ws))
                                row['radar_cir_origin']=sorted(set(w['radar_cir_origin'] for w in ws))
                                row['radar_detected_origin']=sorted(set(w['radar_detected_origin'] for w in ws))
                                row['queue_plus_service_us']=percentiles([w['timing']['queue_us']+w['timing']['e2e_us'] for w in ws])
                            else:
                                dest=args.out/(name+'.csv')
                                cmd=[str(binaries['benchmark_chain_workers']),temporary,'testdata/resampler_65_48/taps_'+profile+'.txt',
                                     str(count),cores,'32',str(args.measure),'16']
                                if args.single_worker_detail:
                                    cmd.append(kind)
                                with dest.open('w') as output, (args.out/(name+'.log')).open('w') as log:
                                    subprocess.run(['taskset','-c','0',*cmd],stdout=output,stderr=log,env=env,check=True)
                                with dest.open() as f:
                                    ws=list(csv.DictReader(f))
                                required=['committed']
                                if kind!='comm': required.append('radar_ok')
                                if kind in ['comm','full_dsp']: required+=['fcs','psdu_exact']
                                if len(ws)!=args.measure or not all(all(int(w[k]) for k in required) for w in ws):
                                    raise RuntimeError('invalid complete DSP result: '+name)
                                interval=(max(float(w['end_us']) for w in ws)-min(float(w['start_us']) for w in ws))/len(ws)
                                row=dict(kind=kind,profile=profile,workers=workers,rep=rep+1,interval_us=interval)
                                for field in [key for key in ws[0] if key.endswith('_us') and key not in ['start_us','end_us']]:
                                    row[field]=percentiles([float(w[field]) for w in ws])
                                row['fallbacks']={key:sum(int(w[key]) for w in ws) for key in ['soft_fallbacks','payload_fallbacks','phr_fallbacks']}
                                row['worker_jobs']=dict(collections.Counter(w['worker'] for w in ws))
                            row['command']=cmd
                            results.append(row)
                            (args.out/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
                            print(name,'interval',round(row['interval_us'],2),'us/window',flush=True)
            # Separate instrumented runs; never mix diagnostic counters into
            # the uninstrumented latency/throughput matrix.
            for workers in ([] if args.single_worker_detail else [1,4,8]):
                name=f'diagnostic_quality_minorder_n{workers}'
                cmd=list(pipeline_commands['quality_minorder',workers])
                cmd[cmd.index('--json')+1]=str(args.out/(name+'.json'))
                cmd+=['--diag-rx','--diag-commit']
                with (args.out/(name+'.log')).open('w') as log:
                    subprocess.run(['taskset','-c','0',*cmd],stdout=log,stderr=subprocess.STDOUT,env=env,check=True)
                print('Completed',name,flush=True)


if __name__=='__main__':
    main()
