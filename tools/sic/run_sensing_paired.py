#!/usr/bin/env python3
"""A/B/B/A paired sensing benchmark driver (R04 rectification).

Runs the two pre-built binaries alternately so drift affects both arms equally,
holds the exclusive benchmark lock for the whole run, and reports per-arm
mean/P50/P95 plus the paired ratio.

  arm A = control (921c38f, the three defects present, fast flags off)
  arm B = candidate (rectified, fast flags on)
"""
import argparse
import fcntl
import statistics
import subprocess
import sys
from pathlib import Path


def run(binary, label, iters, warmup, csv_path):
    cmd = [str(binary), '--label', label, '--iters', str(iters),
           '--warmup', str(warmup), '--csv', str(csv_path)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr)
        raise SystemExit('%s failed' % label)
    vals = []
    for line in Path(csv_path).read_text().splitlines():
        if line.startswith('#') or line.startswith('iter'):
            continue
        parts = line.split(',')
        vals.append(float(parts[1]))
    return vals, r.stderr.strip()


def stats(v):
    s = sorted(v)
    n = len(s)
    return dict(n=n, mean=statistics.fmean(s), p50=s[n // 2],
                p95=s[min(n - 1, int(round(0.95 * (n - 1))))],
                min=s[0], max=s[-1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--before', type=Path, required=True)
    ap.add_argument('--after', type=Path, required=True)
    ap.add_argument('--groups', type=int, default=3)
    ap.add_argument('--slot', type=int, default=32)
    ap.add_argument('--warmup', type=int, default=32)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--lock', default='/tmp/uwb-sic-benchmark.lock')
    a = ap.parse_args()

    a.out.mkdir(parents=True, exist_ok=True)
    lock = open(a.lock, 'w')
    fcntl.flock(lock, fcntl.LOCK_EX)

    before, after = [], []
    for g in range(a.groups):
        for order in (('before', 1), ('after', 1), ('after', 2), ('before', 2)):
            arm = order[0]
            binary = a.before if arm == 'before' else a.after
            tag = '%s_g%d_%d' % (arm, g + 1, order[1])
            vals, note = run(binary, tag, a.slot, a.warmup,
                             a.out / (tag + '.csv'))
            (before if arm == 'before' else after).extend(vals)
            print('%-18s n=%3d mean=%8.2f us  %s' %
                  (tag, len(vals), statistics.fmean(vals), note))

    sb, sa = stats(before), stats(after)
    print()
    print('arm        n    mean      p50      p95      min      max')
    for name, s in (('before', sb), ('after', sa)):
        print('%-8s %4d %8.2f %8.2f %8.2f %8.2f %8.2f' %
              (name, s['n'], s['mean'], s['p50'], s['p95'], s['min'], s['max']))
    print()
    print('ratio after/before: mean %.4f  p50 %.4f  p95 %.4f' %
          (sa['mean'] / sb['mean'], sa['p50'] / sb['p50'],
           sa['p95'] / sb['p95']))
    print('speedup (before/after mean): %.2fx' % (sb['mean'] / sa['mean']))

    import json
    (a.out / 'summary.json').write_text(json.dumps(
        {'before': sb, 'after': sa,
         'ratio_mean': sa['mean'] / sb['mean'],
         'ratio_p50': sa['p50'] / sb['p50'],
         'ratio_p95': sa['p95'] / sb['p95'],
         'speedup_mean': sb['mean'] / sa['mean']}, indent=2) + '\n')
    fcntl.flock(lock, fcntl.LOCK_UN)


if __name__ == '__main__':
    main()
