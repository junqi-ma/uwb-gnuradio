#!/usr/bin/env python3
"""Serial 3-session runner for the shared SYNC-CIR / shared preamble benchmark.

Builds nothing: it takes an already-built `benchmark_shared_cir` binary, runs
three independent sessions (three separate process invocations) strictly
serially, and folds the per-session CSVs into one compact markdown table.

Locking: `benchmark_shared_cir` takes the exclusive
`/tmp/uwb-sic-benchmark.lock` flock itself for the whole of its own run, so the
runner must NOT hold it (that would deadlock the child).  Serial execution plus
the per-session exclusive lock gives the same guarantee: no two benchmark
processes ever overlap.

Usage:
    python3 tools/sic/run_shared_cir_bench.py \
        --binary gr-uwb/build-sensing-shared-cir/bench/benchmark_shared_cir \
        --out docs/performance/data_sensing_shared_cir_20261112
"""
import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

METRIC_ORDER = [
    'total',
    'sfd',
    'refine',
    'cir_scalar_kernel',
    'frontend_timing',
    'frontend_cfo',
    'frontend_sfd',
    'coverage',
    'cir_us_reported',
    'cir_shared_kernel',
    'stage_cir',
]


def read_csv(path):
    """Parse the benchmark CSV, dropping the '#' provenance header."""
    rows, header = [], []
    with path.open() as f:
        lines = [ln for ln in f if not ln.startswith('#')]
    for ln in lines:
        if ln.startswith('record_type'):
            header = ln.rstrip('\n').split(',')
            break
    if not header:
        raise RuntimeError(f'no CSV header in {path}')
    for ln in lines:
        if ln.startswith('record_type') or not ln.strip():
            continue
        rows.append(dict(zip(header, ln.rstrip('\n').split(','))))
    return rows


def provenance(path):
    return [ln.rstrip('\n')[1:].strip() for ln in path.open()
            if ln.startswith('#')]


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def fmt(v, nd=2):
    return '' if v is None else f'{v:.{nd}f}'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--sessions', type=int, default=3)
    ap.add_argument('--groups', type=int, default=5)
    ap.add_argument('--iters', type=int, default=52,
                    help='iterations per ABBA slot (5 groups x 2 slots x N >= 512)')
    ap.add_argument('--warmup', type=int, default=40)
    ap.add_argument('--testdata', type=Path, default=Path('testdata'))
    ap.add_argument('--lock', default='/tmp/uwb-sic-benchmark.lock')
    ap.add_argument('--skip-comm', action='store_true')
    ap.add_argument('--reuse', action='store_true',
                    help='fold existing session CSVs instead of re-running them')
    args = ap.parse_args()

    binary = args.binary.resolve()
    if not binary.is_file():
        sys.exit(f'missing benchmark binary: {binary}')
    if args.warmup < 32:
        sys.exit('warmup must be >= 32')
    per_arm = args.groups * 2 * args.iters
    if per_arm < 512:
        sys.exit(f'groups*2*iters = {per_arm} < 512 observations per arm')

    args.out.mkdir(parents=True, exist_ok=True)

    sessions = []
    for s in range(1, args.sessions + 1):
        csv_path = args.out / f'session{s}.csv'
        log_path = args.out / f'session{s}.log'
        cmd = [str(binary), '--csv', str(csv_path), '--session', str(s),
               '--sessions', str(args.sessions), '--groups', str(args.groups),
               '--iters', str(args.iters), '--warmup', str(args.warmup),
               '--testdata', str(args.testdata), '--lock', args.lock]
        if args.skip_comm:
            cmd.append('--skip-comm')
        if args.reuse:
            if not csv_path.is_file():
                sys.exit(f'--reuse: missing {csv_path}')
            print(f'[{s}/{args.sessions}] reuse {csv_path}', flush=True)
        else:
            print(f'[{s}/{args.sessions}] {" ".join(cmd)}', flush=True)
            with log_path.open('w') as log:
                proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            if proc.returncode != 0:
                tail = log_path.read_text().splitlines()[-20:]
                sys.exit(f'session {s} failed (rc={proc.returncode}):\n' +
                         '\n'.join(tail))
        rows = read_csv(csv_path)
        if not rows:
            sys.exit(f'session {s} produced no CSV rows')
        sessions.append(dict(index=s, rows=rows, csv=csv_path, log=log_path,
                             provenance=provenance(csv_path)))
        print(f'[{s}/{args.sessions}] ok, {len(rows)} summary rows', flush=True)

    # ---- fold the sessions --------------------------------------------------
    combined = {}
    for sess in sessions:
        for r in sess['rows']:
            key = (r['config'], r['arm'], r['metric'])
            entry = combined.setdefault(key, dict(
                n=[], mean=[], p50=[], p95=[], min=[], max=[]))
            for field in ('n', 'mean_us', 'p50_us', 'p95_us', 'min_us', 'max_us'):
                v = fnum(r[field])
                if v is not None:
                    entry[{'n': 'n', 'mean_us': 'mean', 'p50_us': 'p50',
                           'p95_us': 'p95', 'min_us': 'min',
                           'max_us': 'max'}[field]].append(v)

    def agg(key, field):
        vals = combined.get(key, {}).get(field) or []
        return sum(vals) / len(vals) if vals else None

    def spread(key, field):
        """min..max of the per-session values of `field`."""
        vals = combined.get(key, {}).get(field) or []
        return (min(vals), max(vals)) if vals else (None, None)

    configs = []
    for (config, _arm, _metric) in combined:
        if config not in configs:
            configs.append(config)

    # ---- markdown -----------------------------------------------------------
    out = []
    out.append('# Combined summary (3 independent sessions)\n')
    out.append('All values in microseconds. `ratio` = candidate mean / control '
               'mean for the same metric.\n')
    out.append('`mean`/`P50`/`P95` are the average of the per-session values; '
               'the bracketed range is the min..max of those per-session '
               'values. `min`/`max` are the extreme single observations seen '
               'in any session.\n')

    for config in configs:
        out.append(f'\n## {config}\n')
        out.append('| arm | metric | n | mean | P50 | P95 | min | max | ratio |')
        out.append('|---|---|---|---|---|---|---|---|---|')
        arms = []
        for (c, arm, _m) in combined:
            if c == config and arm not in arms:
                arms.append(arm)
        for arm in arms:
            metrics = [m for (c, a, m) in combined if c == config and a == arm]
            metrics.sort(key=lambda m: METRIC_ORDER.index(m)
                         if m in METRIC_ORDER else 99)
            for metric in metrics:
                key = (config, arm, metric)
                ratio = agg(key, 'mean')
                ctrl = agg((config, 'control', metric), 'mean')
                if arm != 'control' and ctrl:
                    ratio = ratio / ctrl
                else:
                    ratio = None
                lo, hi = spread(key, 'mean')
                out.append(
                    f'| {arm} | {metric} | {int(agg(key, "n") or 0)} | '
                    f'{fmt(agg(key, "mean"))}'
                    f'{"" if lo is None else f" [{fmt(lo)}..{fmt(hi)}]"} | '
                    f'{fmt(agg(key, "p50"))} | {fmt(agg(key, "p95"))} | '
                    f'{fmt(spread(key, "min")[0])} | '
                    f'{fmt(spread(key, "max")[1])} | '
                    f'{"" if ratio is None else f"{ratio:.4f}"} |')

    # ---- key ratios ---------------------------------------------------------
    out.append('\n## Key ratios (candidate / control)\n')
    out.append('| config | metric | control mean | candidate mean | ratio | '
               'speedup |')
    out.append('|---|---|---|---|---|---|')
    # The kernel-only arms have different metric names on purpose (the control
    # runs the legacy scalar kernel, the candidate the shared kernel), so they
    # are paired explicitly.
    pairs = [('total', 'total'), ('stage_cir', 'stage_cir'),
             ('cir_scalar_kernel', 'cir_shared_kernel')]
    for config in configs:
        for ctrl_metric, cand_metric in pairs:
            ctrl = agg((config, 'control', ctrl_metric), 'mean')
            cand = agg((config, 'candidate', cand_metric), 'mean')
            if ctrl is None or cand is None:
                ctrl = agg((config, 'comm_legacy_profile', ctrl_metric), 'mean')
                cand = agg((config, 'comm_fast_profile', cand_metric), 'mean')
            if ctrl is None or cand is None:
                continue
            ratio = cand / ctrl
            label = ctrl_metric if ctrl_metric == cand_metric else \
                f'{ctrl_metric} -> {cand_metric}'
            out.append(f'| {config} | {label} | {fmt(ctrl)} | {fmt(cand)} | '
                       f'{ratio:.4f} | {1/ratio:.3f}x |')

    summary_md = args.out / 'summary.md'
    summary_md.write_text('\n'.join(out) + '\n')

    # ---- machine readable ---------------------------------------------------
    payload = dict(
        binary=str(binary),
        sessions=args.sessions,
        groups=args.groups,
        iters_per_slot=args.iters,
        warmup=args.warmup,
        observations_per_arm_per_session=per_arm,
        provenance={f'session{s["index"]}': s['provenance'] for s in sessions},
        combined={f'{c}|{a}|{m}': dict(
            n=agg((c, a, m), 'n'),
            mean=agg((c, a, m), 'mean'),
            p50=agg((c, a, m), 'p50'),
            p95=agg((c, a, m), 'p95'),
            min=agg((c, a, m), 'min'),
            max=agg((c, a, m), 'max'),
            mean_session_range=spread((c, a, m), 'mean'),
        ) for (c, a, m) in combined},
    )
    (args.out / 'combined.json').write_text(json.dumps(payload, indent=2) + '\n')
    print(f'wrote {summary_md} and {args.out / "combined.json"}', flush=True)


if __name__ == '__main__':
    main()
