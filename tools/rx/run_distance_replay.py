#!/usr/bin/env python3
"""Run the 7-distance 1 ms offline replay and summarize one binary's result.

This is the comparison harness for the receiver optimization work
(docs/performance/多Subagent并行开发方案_距离接收优化_2026-11-16.md).  It
never writes into the capture directories: logs go to --out and the summary is
printed as a table plus JSON.

Per distance it reports, from the PACKET lines of the live console:
  * PRR over the nominal 1 ms slot grid (phase aligned on the first CRC-OK
    packet, so a late first lock is not counted as success),
  * the failure classification (NOWIN / TIM / CFO / SFD / PAY / other),
  * demod_us mean (all windows and OK-only), p95, compute_us mean,
  * the stage means used by the optimization plan
    (st_timing / st_sfd / sfd_cforef / st_cir / st_payload / resample_us).
"""
import argparse
import collections
import json
import math
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _common.paths import find_app  # noqa: E402

DISTANCES = ['d0p8m', 'd1p6m', 'd2p5m', 'd3p5m', 'd3p8m', 'd4p2m', 'd5p2m']
DEFAULT_ROOT = Path('/media/oi/Data/uwb/clean_distance/2026-11-16')
DEFAULT_CAPTURE = 'rx_2s_r01.sc16'
DEFAULT_ARGS = ['--mode', 'continuous', '--sync', 'locked', '--cir', 'full',
                '--workers', '4', '--period-ms', '1.0', '--taps', 'eq16',
                '--print', 'packets', '--summary', '1000']

FAIL_KEYS = ['NO_WINDOW', 'TIMING_FAILED', 'CFO_FAILED', 'SFD_FAILED',
             'CIR_FAILED', 'PHR_FAILED', 'PAYLOAD_FAILED', 'FCS_FAILED']
STAGE_KEYS = ['st_timing', 'st_sfd', 'sfd_cforef', 'st_cfo', 'st_cir',
              'st_payload', 'st_nssfd', 'st_phr', 'resample_us', 'demod_us',
              'compute_us', 'st_fused', 'st_fallback']


def fields(line):
    return dict(re.findall(r'(\w+)=([^ ]+)', line))


def mean(xs):
    return sum(xs) / len(xs) if xs else float('nan')


def pct(xs, q):
    if not xs:
        return float('nan')
    s = sorted(xs)
    k = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return s[k]


def summarize_log(log_path, distance_m):
    lines = Path(log_path).read_text(errors='replace').splitlines()
    rows = [fields(l) for l in lines if l.startswith('PACKET ')]
    good = [r for r in rows if r['status'] == 'OK' and r.get('crc') == '1']
    start = next((fields(l) for l in lines if l.startswith('START ')), {})
    final = next((fields(l) for l in lines if l.startswith('FINAL ')), {})
    lock = next((fields(l) for l in lines if l.startswith('LOCK_STATS ')), {})
    cost = next((fields(l) for l in lines if l.startswith('COST_TOTAL ')), {})

    # Nominal 1 ms slot grid; phase from the first CRC-OK packet so that a late
    # first lock shows up as NO_WINDOW slots instead of a shifted grid.
    expected = 2000
    if final.get('raw_samples'):
        expected = int(round(int(final['raw_samples']) / 737280000.0 * 1000))
    phase = (float(good[0]['time_ms']) - math.floor(float(good[0]['time_ms']))
             if good else 0.0)
    slots = collections.defaultdict(list)
    for r in rows:
        k = int(round(float(r['time_ms']) - phase))
        if 0 <= k < expected:
            slots[k].append(r['status'])
    counts = collections.Counter()
    for k in range(expected):
        sts = slots.get(k, [])
        if not sts:
            counts['NO_WINDOW'] += 1
            continue
        if 'OK' in sts:
            counts['OK'] += 1
        else:
            counts[sts[0]] += 1

    def nums(key, rs):
        out = []
        for r in rs:
            v = r.get(key)
            if v is None:
                continue
            try:
                out.append(float(v))
            except ValueError:
                pass
        return out

    ok_demod = nums('demod_us', good)
    all_demod = nums('demod_us', rows)
    all_compute = nums('compute_us', rows)
    stages = {k: mean(nums(k, good)) for k in STAGE_KEYS}
    return dict(
        distance_m=distance_m,
        expected_slots=expected,
        ok_slots=counts['OK'],
        prr_pct=100.0 * counts['OK'] / expected,
        failures={k: counts.get(k, 0) for k in FAIL_KEYS},
        windows=len(rows),
        duplicate_ok_windows=len(good) - counts['OK'],
        first_ok_ms=(float(good[0]['time_ms']) if good else None),
        demod_mean_all=mean(all_demod),
        demod_mean_ok=mean(ok_demod),
        demod_p95_ok=pct(ok_demod, 0.95),
        demod_p95_all=pct(all_demod, 0.95),
        demod_max_ok=(max(ok_demod) if ok_demod else float('nan')),
        compute_mean=mean(all_compute),
        compute_p95=pct(all_compute, 0.95),
        stages=stages,
        grid_reject=int(mean(nums('grid_reject', good)) * len(good)) if good else 0,
        fused_ok=sum(1 for r in good if r.get('st_fused') not in (None, '0')),
        lock=dict(lock_entries=lock.get('lock_entries'),
                  holdover_entries=lock.get('holdover_entries'),
                  reacquire_entries=lock.get('reacquire_entries'),
                  discontinuities=lock.get('discontinuities'),
                  period_ppm=lock.get('period_ppm'),
                  expected_slots=lock.get('expected_slots')),
        frontend_us=cost.get('frontend_us'),
        cancel_us=cost.get('cancel_us'),
        raw_drop_samples=final.get('raw_drop_samples'),
        source_errors=final.get('source_errors'),
        gaps=final.get('gaps'),
        start_args=start,
    )


def run_one(binary, capture, out_dir, extra_args, timeout_s):
    out_dir.mkdir(parents=True, exist_ok=True)
    log = out_dir / 'replay.log'
    cmd = [str(binary), '--replay', str(capture)] + list(extra_args)
    t0 = time.time()
    with log.open('w') as f:
        proc = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT,
                              timeout=timeout_s)
    return log, proc.returncode, time.time() - t0


def fmt_table(results):
    head = ('dist  PRR%    OK/exp   NOWIN TIM  CFO  SFD  PAY   '
            'demod_all demod_ok  p95_ok  compute   timing   sfd  cforef    cir  payload')
    lines = [head]
    for r in results:
        f = r['failures']
        st = r['stages']
        lines.append(
            f"{r['distance_m']:>4.1f} {r['prr_pct']:7.2f} "
            f"{r['ok_slots']:4d}/{r['expected_slots']:<4d} "
            f"{f['NO_WINDOW']:5d} {f['TIMING_FAILED']:4d} {f['CFO_FAILED']:4d} "
            f"{f['SFD_FAILED']:4d} {f['PAYLOAD_FAILED'] + f['FCS_FAILED']:4d}  "
            f"{r['demod_mean_all']:9.1f} {r['demod_mean_ok']:8.1f} "
            f"{r['demod_p95_ok']:7.1f} {r['compute_mean']:8.1f} "
            f"{st['st_timing']:8.1f} {st['st_sfd']:5.1f} {st['sfd_cforef']:7.1f} "
            f"{st['st_cir']:6.1f} {st['st_payload']:8.1f}")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--binary', type=Path, default=None,
                    help='receiver binary (default: discover openuwb_rx)')
    ap.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--distances', nargs='*', default=DISTANCES)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--label', default='')
    ap.add_argument('args', nargs=argparse.REMAINDER,
                    help='extra live args after -- (defaults to the plan set)')
    args = ap.parse_args()
    if args.binary is None:
        args.binary = find_app('openuwb_rx')
    extra = args.args[1:] if args.args[:1] == ['--'] else args.args
    if not extra:
        extra = list(DEFAULT_ARGS)
    args.out.mkdir(parents=True, exist_ok=True)
    results = []
    for d in args.distances:
        cap = args.root / f'{d}_period1ms' / DEFAULT_CAPTURE
        if not cap.exists():
            print(f'missing capture {cap}', file=sys.stderr)
            continue
        dist = float(d[1:-1].replace('p', '.'))
        log, rc, wall = run_one(args.binary, cap, args.out / d, extra, args.timeout)
        r = summarize_log(log, dist)
        r['exit_code'] = rc
        r['wall_s'] = round(wall, 2)
        r['log'] = str(log)
        results.append(r)
        print(f"{d}: rc={rc} PRR={r['prr_pct']:.2f}% "
              f"demod_ok={r['demod_mean_ok']:.1f}us compute={r['compute_mean']:.1f}us "
              f"({wall:.1f}s)", flush=True)
    if not results:
        return 1
    payload = dict(label=args.label, binary=str(args.binary),
                   args=extra, results=results)
    (args.out / 'summary.json').write_text(json.dumps(payload, indent=2) + '\n')
    table = fmt_table(results)
    (args.out / 'summary.txt').write_text(table + '\n')
    print()
    print(table)
    return 0


if __name__ == '__main__':
    sys.exit(main())
