#!/usr/bin/env python3
"""串行比较原生接收机各 worker 配置；先构建 live_dw3000_packets.py 对应程序。"""
import argparse
from collections import Counter
import fcntl
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BINARY = ROOT / 'build/cfo-tail-fix/apps/openuwb_rx'


def fields(line):
    return dict(part.split('=', 1) for part in line.split()[1:] if '=' in part)


def stats(values):
    values = sorted(values)
    if not values:
        return None
    def percentile(q):
        at = (len(values)-1)*q
        lo = int(at)
        return values[lo] + (values[min(lo+1,len(values)-1)]-values[lo])*(at-lo)
    return dict(n=len(values), mean=statistics.mean(values), p50=percentile(.5),
                p95=percentile(.95), p99=percentile(.99), maximum=max(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='新建结果目录，不覆盖历史 run')
    parser.add_argument('--seconds', type=float, default=3)
    parser.add_argument('--replay', type=Path, help='不指定时使用 X410；指定时做 paced 回放')
    parser.add_argument('--workers', type=int, nargs='+', default=[1,2,4,4,2,1],
                        help='按指定顺序运行，例如 --workers 8 8；默认 1 2 4 4 2 1')
    args = parser.parse_args()
    if any(not 1 <= w <= 32 for w in args.workers):
        parser.error('workers must be within 1..32')
    if args.seconds <= 0:
        parser.error('seconds must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.pop('LD_LIBRARY_PATH', None)
    runs = []
    with open('/tmp/uwb-sic-benchmark.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for index, workers in enumerate(args.workers, 1):
            command = [str(BINARY), '--workers', str(workers), '--seconds', str(args.seconds)]
            if args.replay:
                command += ['--replay', str(args.replay.resolve())]
            path = args.output / f'run_{index}_workers_{workers}.log'
            print(f'run {index}/{len(args.workers)} workers={workers}', flush=True)
            with path.open('w') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                        env=env, timeout=args.seconds+90)
            lines = path.read_text().splitlines()
            # UHD overflow markers go to stderr without a newline (OPACKET...).
            def extract(marker):
                return [fields(line[line.index(marker):]) for line in lines if marker in line]
            packets = extract('PACKET ')
            totals = extract('COST_TOTAL ')
            final = extract('FINAL ')
            if result.returncode or not final or not totals:
                raise RuntimeError(f'run failed, inspect {path}')
            if len(packets) != int(final[-1]['completed']):
                raise RuntimeError(f'incomplete packet log, inspect {path}')
            runs.append(dict(workers=workers, command=command, log=path.name,
                             packets=packets, cost=totals[-1], final=final[-1]))
            print(final[-1], flush=True)
    summary = dict(base_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT,text=True).strip(),
                   binary_sha256=hashlib.sha256(BINARY.read_bytes()).hexdigest(),
                   mode='paced_replay' if args.replay else 'uhd', seconds=args.seconds,
                   order=args.workers, excluded_warmup_ids=20, runs=runs, workers={})
    keys = ['queue_us','resample_us','demod_us','retry_us','compute_us','cpu_us','latency_ms']
    for w in sorted(set(args.workers)):
        group = [r for r in runs if r['workers']==w]
        packets = [p for r in group for p in r['packets']]
        measured = [p for p in packets if int(p['id'])>20 and 'compute_us' in p]
        good = [p for p in measured if p['status']=='OK' and p.get('crc')=='1']
        cost = {k:stats([float(p[k]) for p in measured]) for k in keys}
        normal = [p for p in good if p.get('full_retry')=='0']
        duration = sum(float(r['cost']['frontend_samples'])/737.28e6 for r in group)
        front = {k:sum(float(r['cost'][k]) for r in group)/duration/1000
                 for k in ['frontend_us','estimate_us','cancel_us']}
        summary['workers'][w] = dict(candidates=len(packets), statuses=dict(Counter(p['status'] for p in packets)),
                measured=len(measured), measured_success=len(good), full_retries=sum(int(p.get('full_retry',0)) for p in packets),
                costs=cost, normal_success_compute_us=stats([float(p['compute_us']) for p in normal]),
                frontend_ms_per_signal_second=front,
                worker_cpu_ms_per_signal_second=sum(float(p['cpu_us']) for p in measured)/duration/1000)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for w, data in summary['workers'].items():
        print(w, json.dumps(data,ensure_ascii=False), flush=True)


if __name__=='__main__':
    main()
