#!/usr/bin/env python3
"""R06 functional matrix runner for experiment_preamble_recovery.

Enumerates the plan §4.2 matrix (2 configs x 2 grids x 2 code directions x
2 multipath x 3 ISR x 3 target CFO x 3 interferer CFO = 432 cell-grid pairs,
432 * 16 = 6912 seed cases), runs it serially under
/tmp/uwb-sic-benchmark.lock, and aggregates per cell and per axis.

Nothing is skipped silently: a cell whose CSV is missing, short, or whose
process failed is reported as a failure with its reason in summary.json,
summary.md and cells.csv.

The native grid needs the real 65/48 chain, so each native cell is a three
step pipeline:
  1. binary --stage dump          (work-grid CF32 clean+raw pair per seed)
  2. this script                  (independent scipy 48/65 generator -> SC16)
  3. binary --grid native         (production 65/48 core -> demod/SIC/CIR)
"""

import argparse
import collections
import csv
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import struct
import subprocess
import sys
import tempfile

AXES = {
    'config': [(64, 32), (256, 127)],
    'code': [(9, 10), (10, 9)],
    'multipath': ['single', 'fractional'],
    'isr': [0.0, 10.0, 20.0],
    'target_cfo': [0.0, 50000.0, -50000.0],
    'interferer_cfo': [0.0, 50000.0, -50000.0],
}
GRIDS = ['work', 'native']

SMOKE = {
    'config': [(64, 32), (256, 127)],
    'code': [(9, 10), (10, 9)],
    'multipath': ['single', 'fractional'],
    'isr': [0.0, 20.0],
    'target_cfo': [0.0],
    'interferer_cfo': [0.0],
}

# Independent native generator; deliberately NOT the 65/48 filter under test.
GENERATOR_TAPS = 8193
GENERATOR_BETA = 10.0


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def percentile(values, q):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def case_id(cell):
    reps, psdu = cell['config']
    tgt, itf = cell['code']
    return ('sync%d_psdu%d_tgt%d_int%d_%s_isr%d_ct%d_ci%d'
            % (reps, psdu, tgt, itf, cell['multipath'], round(cell['isr']),
               round(cell['target_cfo']), round(cell['interferer_cfo'])))


def enumerate_cells(axes):
    cells = []
    for config in axes['config']:
        for code in axes['code']:
            for mp in axes['multipath']:
                for isr in axes['isr']:
                    for tcfo in axes['target_cfo']:
                        for icfo in axes['interferer_cfo']:
                            cells.append(dict(config=config, code=code, multipath=mp,
                                              isr=isr, target_cfo=tcfo,
                                              interferer_cfo=icfo))
    return cells


def cell_args(cell, seeds, grid, out, extra=()):
    reps, psdu = cell['config']
    tgt, itf = cell['code']
    return ['--sync-reps', str(reps), '--psdu-bytes', str(psdu),
            '--target-code', str(tgt), '--interferer-code', str(itf),
            '--multipath', cell['multipath'],
            '--isr-db', repr(cell['isr']),
            '--target-cfo-hz', repr(cell['target_cfo']),
            '--interferer-cfo-hz', repr(cell['interferer_cfo']),
            '--seeds', str(seeds), '--grid', grid, '--out', str(out)] + list(extra)


def expected_rows(cell, seeds, grid):
    return seeds * (4 if grid == 'native' else 3)


def run_native_sources(binary, cell, seeds, scratch, env, extra=()):
    """dump -> independent scipy 48/65 -> SC16, returns (ok, reason)."""
    import numpy as np
    from scipy.signal import firwin, resample_poly

    dump = subprocess.run([str(binary), '--stage', 'dump', '--dump-dir', str(scratch)]
                          + cell_args(cell, seeds, 'work', '-', extra),
                          capture_output=True, text=True, env=env)
    if dump.returncode != 0:
        return False, 'dump: ' + (dump.stderr.strip().splitlines() or ['?'])[-1]

    generator = firwin(GENERATOR_TAPS, 1 / 65, window=('kaiser', GENERATOR_BETA))
    sources = sorted(scratch.glob('*.cf32'))
    if len(sources) != seeds:
        return False, 'dump produced %d source pairs, expected %d' % (len(sources), seeds)
    for path in sources:
        stacked = np.fromfile(path, '<c8').astype(np.complex128).reshape(2, -1)
        native = resample_poly(stacked, 48, 65, axis=1, window=generator)
        peak = max(float(np.max(abs(native.real))), float(np.max(abs(native.imag))))
        if not peak > 0:
            return False, 'empty native source ' + path.name
        # One common ADC scale for the clean/raw pair; no clipping.
        gain = .9 * 32767 / peak
        rounded = np.stack([np.rint(native.real * gain), np.rint(native.imag * gain)],
                           axis=-1)
        if np.max(abs(rounded)) > 32767:
            return False, 'SC16 clipping in ' + path.name
        iq = rounded.astype('<i2')
        for block, name in ((0, 'clean'), (1, 'raw')):
            target = path.with_name(path.stem + '_' + name + '.sc16')
            with target.open('wb') as handle:
                handle.write(struct.pack('<Qd', native.shape[1], 1 / gain))
                handle.write(iq[block].tobytes())
        path.unlink()
    return True, ''


def run_cell(binary, cell, grid, seeds, out_dir, env, taps, extra=()):
    out = out_dir / (case_id(cell) + '.csv')
    if grid == 'work':
        args = [str(binary)] + cell_args(cell, seeds, 'work', out, extra)
        result = subprocess.run(args, capture_output=True, text=True, env=env)
        if result.returncode != 0:
            return False, 'run: ' + (result.stderr.strip().splitlines() or ['?'])[-1]
        return True, ''
    with tempfile.TemporaryDirectory(prefix='uwb-r06-native-') as name:
        scratch = Path(name)
        ok, reason = run_native_sources(binary, cell, seeds, scratch, env, extra)
        if not ok:
            return False, reason
        args = [str(binary)] + cell_args(cell, seeds, 'native', out, extra)
        args += ['--sc16-dir', str(scratch), '--taps', str(taps)]
        result = subprocess.run(args, capture_output=True, text=True, env=env)
        if result.returncode != 0:
            return False, 'run: ' + (result.stderr.strip().splitlines() or ['?'])[-1]
    return True, ''


def numeric(row, field):
    try:
        value = float(row[field])
    except (KeyError, ValueError):
        return None
    return value if math.isfinite(value) else None


def aggregate(rows):
    """Aggregate one group of rows (one path) into the reported metrics."""
    n = len(rows)
    if not n:
        return None
    valid = [r for r in rows if int(r['radar_status']) == 0]
    l2 = [v for v in (numeric(r, 'relative_l2') for r in valid) if v is not None]
    shift = [v for v in (numeric(r, 'peak_shift') for r in valid) if v is not None]
    gain = [v for v in (numeric(r, 'peak_gain_db') for r in valid) if v is not None]
    pslr = [v for v in (numeric(r, 'pslr_db') for r in valid) if v is not None]
    islr = [v for v in (numeric(r, 'islr_db') for r in valid) if v is not None]
    width = [v for v in (numeric(r, 'mainlobe_width_taps') for r in valid) if v is not None]
    resid = [v for v in (numeric(r, 'residual_improvement_db') for r in rows) if v is not None]
    cfo_err = []
    for r in rows:
        measured = numeric(r, 'measured_cfo_hz')
        applied = numeric(r, 'target_cfo_hz')
        if measured is not None and applied is not None:
            cfo_err.append(abs(measured - applied))
    weak = [int(r['weak_preserved']) for r in rows if int(r['weak_preserved']) >= 0]
    return dict(
        n=n,
        sfd=sum(int(r['sfd_ok']) for r in rows),
        phr=sum(int(r['phr_ok']) for r in rows),
        fcs=sum(int(r['fcs_pass']) for r in rows),
        psdu=sum(int(r['psdu_exact']) for r in rows),
        radar_ok=len(valid),
        sic_committed=sum(int(r['sic_committed']) for r in rows),
        relative_l2_mean=statistics.fmean(l2) if l2 else None,
        relative_l2_p50=percentile(l2, .5),
        relative_l2_p95=percentile(l2, .95),
        peak_shift_mean=statistics.fmean(shift) if shift else None,
        peak_shift_max_abs=max(map(abs, shift)) if shift else None,
        peak_gain_db_mean=statistics.fmean(gain) if gain else None,
        pslr_db_mean=statistics.fmean(pslr) if pslr else None,
        pslr_db_p50=percentile(pslr, .5),
        islr_db_mean=statistics.fmean(islr) if islr else None,
        islr_db_p50=percentile(islr, .5),
        mainlobe_width_mean=statistics.fmean(width) if width else None,
        residual_improvement_db_mean=statistics.fmean(resid) if resid else None,
        residual_improvement_db_p50=percentile(resid, .5),
        measured_cfo_abs_err_mean=statistics.fmean(cfo_err) if cfo_err else None,
        measured_cfo_abs_err_p95=percentile(cfo_err, .95),
        weak_preserved_rate=(sum(weak) / len(weak)) if weak else None,
        reasons=dict(collections.Counter(r['sic_reason'] for r in rows)),
        failures=dict(collections.Counter(
            r['failure_reason'] for r in rows if r['failure_reason'])),
    )


def axis_label(field, cell):
    if field == 'config':
        return '%d/%d' % cell['config']
    if field == 'code':
        return '%d<-%d' % cell['code']
    if field == 'multipath':
        return cell['multipath']
    if field == 'isr':
        return '%g dB' % cell['isr']
    if field == 'target_cfo':
        return '%g Hz' % cell['target_cfo']
    return '%g Hz' % cell['interferer_cfo']


def fmt(value, digits=3):
    if value is None:
        return '-'
    if isinstance(value, float):
        return ('%.' + str(digits) + 'f') % value
    return str(value)


def markdown_axis_table(title, field, cells, rows_by_cell, grids, paths):
    lines = ['### %s' % title, '']
    lines.append('| %s | grid | path | n | SFD | PSDU | FCS | radar | SIC | L2 mean | '
                 'L2 P50 | L2 P95 | shift mean | shift max | gain dB | PSLR dB | '
                 'ISLR dB | width | residual dB | CFO err Hz |' % field)
    lines.append('|' + '---|' * 20)
    labels = []
    for cell in cells:
        label = axis_label(field, cell)
        if label not in labels:
            labels.append(label)
    for label in labels:
        for grid in grids:
            for path in paths:
                group = []
                for cell in cells:
                    if axis_label(field, cell) != label:
                        continue
                    group.extend(rows_by_cell.get((grid, case_id(cell)), {}).get(path, []))
                agg = aggregate(group)
                if agg is None:
                    lines.append('| %s | %s | %s | 0 | - | - | - | - | - | - | - | - | '
                                 '- | - | - | - | - | - | - | - |' % (label, grid, path))
                    continue
                lines.append('| %s | %s | %s | %d | %d | %d | %d | %d | %d | %s | %s | '
                             '%s | %s | %s | %s | %s | %s | %s | %s | %s |'
                             % (label, grid, path, agg['n'], agg['sfd'], agg['psdu'],
                                agg['fcs'], agg['radar_ok'], agg['sic_committed'],
                                fmt(agg['relative_l2_mean'], 4),
                                fmt(agg['relative_l2_p50'], 4),
                                fmt(agg['relative_l2_p95'], 4),
                                fmt(agg['peak_shift_mean'], 2),
                                fmt(agg['peak_shift_max_abs'], 2),
                                fmt(agg['peak_gain_db_mean'], 2),
                                fmt(agg['pslr_db_mean'], 2),
                                fmt(agg['islr_db_mean'], 2),
                                fmt(agg['mainlobe_width_mean'], 1),
                                fmt(agg['residual_improvement_db_mean'], 2),
                                fmt(agg['measured_cfo_abs_err_mean'], 1)))
    lines.append('')
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--seeds', type=int, default=16)
    ap.add_argument('--grid', choices=['work', 'native', 'both'], default=None,
                    help='default: both for the full matrix, work for --smoke')
    ap.add_argument('--taps', type=Path,
                    default=Path('testdata/resampler_65_48/taps_quality_minorder.txt'))
    ap.add_argument('--smoke', action='store_true',
                    help='representative subset (2 config x 2 code x 2 ISR x 1 CFO '
                         'x 2 multipath x 2 seeds, work grid unless --grid is given)')
    ap.add_argument('--resume', action='store_true',
                    help='skip cells whose CSV already has the expected row count')
    ap.add_argument('--dual-target', action='store_true',
                    help='add the -20 dB weak echo (separate special case, not the '
                         '432-cell matrix)')
    ap.add_argument('--limit', type=int, default=0,
                    help='debug: run at most N cell-grid pairs')
    args = ap.parse_args()

    if args.seeds < 1:
        ap.error('--seeds must be positive')
    binary = args.binary.resolve()
    if not binary.exists():
        ap.error('binary not found: %s' % binary)
    if args.grid is None:
        grids = ['work'] if args.smoke else list(GRIDS)
    else:
        grids = list(GRIDS) if args.grid == 'both' else [args.grid]
    axes = dict(SMOKE) if args.smoke else dict(AXES)
    cells = enumerate_cells(axes)
    pairs_all = [(grid, cell) for grid in grids for cell in cells]
    pairs = pairs_all[:args.limit] if args.limit else pairs_all

    env = dict(os.environ)
    env.pop('LD_LIBRARY_PATH', None)
    if args.out.exists() and any(args.out.iterdir()) and not args.resume:
        ap.error('output directory is not empty: %s (historical runs are never '
                 'overwritten; use --resume to continue this one)' % args.out)
    args.out.mkdir(parents=True, exist_ok=True)
    for grid in grids:
        (args.out / grid).mkdir(exist_ok=True)

    taps = args.taps.resolve()
    if 'native' in grids and not taps.exists():
        ap.error('native grid needs --taps: %s' % taps)

    ldd = subprocess.check_output(['ldd', str(binary)], text=True, env=env)
    provenance = dict(
        command=' '.join(sys.argv),
        commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'],
                                       text=True).strip(),
        git_status=subprocess.check_output(['git', 'status', '--short'],
                                           text=True),
        binary=str(binary),
        binary_sha256=sha256(binary),
        ldd=ldd,
        libraries={line.split('=>')[1].strip().split()[0]:
                   sha256(line.split('=>')[1].strip().split()[0])
                   for line in ldd.splitlines()
                   if 'libgnuradio-uwb' in line and '=>' in line},
        source_sha256={p: sha256(p) for p in
                       ['tools/sic/experiment_preamble_recovery.cc', __file__]},
        taps=(dict(path=str(taps), sha256=sha256(taps),
                   taps=len(taps.read_bytes()) // 4) if 'native' in grids else None),
        axes={k: [list(v) if isinstance(v, tuple) else v for v in vals]
              for k, vals in axes.items()},
        grids=grids,
        seeds=args.seeds,
        cells=len(cells),
        cell_grid_pairs=len(pairs_all),
        seed_cases=len(pairs_all) * args.seeds,
        pairs_run=len(pairs),
        limit=args.limit,
        dual_target=args.dual_target,
        generator=('independent scipy resample_poly 48/65, %d-tap Kaiser beta %g, '
                   'group delay compensated by scipy' % (GENERATOR_TAPS, GENERATOR_BETA)),
        quantization='one common scale per clean/raw pair, round-to-nearest, 90% '
                     'component headroom, zero clipping',
        online_inputs='mixed IQ, public target SYNC template, public interferer '
                      'preamble template, configured coarse ROI +/-64, configured '
                      'nominal target start',
        truth_only='packet roles, applied target/interferer CFO, ISR, noise sigma, '
                   'channel delays and gains, PSDU bytes, clean reference IQ/CIR, '
                   'expected peak tap, main-lobe region, weak-target tap',
        noise='20 dB desired-packet-average SNR, identical noise in clean/raw/SIC',
        isr='power ratio over the first 64 SYNC symbols (independent of the '
            'configured preamble length)',
        multipath='fractional = second target path at +7.3 samples, gain '
                  '0.6*e^{j0.7}, 65-tap Kaiser-windowed sinc fractional delay',
        metrics='fixed CIR axis (tap i <-> lag i - 16 relative to the public '
                'TX-time prediction); fixed main-lobe region = +/-6 taps around '
                'each truth path, applied unchanged to ideal/clean/raw/sic',
    )
    (args.out / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')

    failures = []
    with open('/tmp/uwb-sic-benchmark.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        for index, (grid, cell) in enumerate(pairs, 1):
            cid = case_id(cell)
            target_csv = args.out / grid / (cid + '.csv')
            need = expected_rows(cell, args.seeds, grid)
            if args.resume and target_csv.exists():
                with target_csv.open() as handle:
                    if len(list(csv.DictReader(handle))) == need:
                        print('[%d/%d] %s %s (resume: complete)' % (index, len(pairs),
                                                                    grid, cid), flush=True)
                        continue
            extra = ['--dual-target'] if args.dual_target else []
            ok, reason = run_cell(binary, cell, grid, args.seeds, args.out / grid, env,
                                  taps, extra)
            rows = 0
            if ok and target_csv.exists():
                with target_csv.open() as handle:
                    rows = len(list(csv.DictReader(handle)))
            if not ok or rows != need:
                reason = reason or ('row count %d != expected %d' % (rows, need))
                failures.append(dict(grid=grid, case_id=cid, reason=reason))
                print('[%d/%d] %s %s FAILED: %s' % (index, len(pairs), grid, cid, reason),
                      flush=True)
            else:
                print('[%d/%d] %s %s ok (%d rows)' % (index, len(pairs), grid, cid, rows),
                      flush=True)

    # ---- load ------------------------------------------------------------
    rows_by_cell = {}
    for grid, cell in pairs:
        path = args.out / grid / (case_id(cell) + '.csv')
        if not path.exists():
            continue
        with path.open() as handle:
            grouped = collections.defaultdict(list)
            for row in csv.DictReader(handle):
                grouped[row['path']].append(row)
            rows_by_cell[(grid, case_id(cell))] = grouped

    # ---- per cell --------------------------------------------------------
    cell_records = []
    for grid, cell in pairs:
        cid = case_id(cell)
        grouped = rows_by_cell.get((grid, cid), {})
        record = dict(grid=grid, case_id=cid, config='%d/%d' % cell['config'],
                      code='%d<-%d' % cell['code'], multipath=cell['multipath'],
                      isr_db=cell['isr'], target_cfo_hz=cell['target_cfo'],
                      interferer_cfo_hz=cell['interferer_cfo'],
                      expected_rows=expected_rows(cell, args.seeds, grid),
                      paths={})
        for path in ('ideal', 'clean', 'raw', 'sic'):
            agg = aggregate(grouped.get(path, []))
            if agg is not None:
                record['paths'][path] = agg
        record['complete'] = (sum(len(v) for v in grouped.values()) ==
                              record['expected_rows'])
        cell_records.append(record)

    # ---- per axis --------------------------------------------------------
    axis_records = {}
    for field in AXES:
        axis_records[field] = {}
        for cell in cells:
            label = axis_label(field, cell)
            bucket = axis_records[field].setdefault(label, {})
            for grid in grids:
                grouped = rows_by_cell.get((grid, case_id(cell)), {})
                for path, rows in grouped.items():
                    bucket.setdefault((grid, path), []).extend(rows)
    axis_summary = {}
    for field, labels in axis_records.items():
        axis_summary[field] = {}
        for label, bucket in labels.items():
            axis_summary[field][label] = {
                '%s/%s' % (grid, path): aggregate(rows)
                for (grid, path), rows in sorted(bucket.items())}

    summary = dict(provenance=provenance, failures=failures,
                   cells=cell_records, axes=axis_summary)
    (args.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')

    with (args.out / 'cells.csv').open('w', newline='') as handle:
        fields = ['grid', 'case_id', 'config', 'code', 'multipath', 'isr_db',
                  'target_cfo_hz', 'interferer_cfo_hz', 'path', 'n', 'complete',
                  'sfd', 'phr', 'fcs', 'psdu', 'radar_ok', 'sic_committed',
                  'relative_l2_mean', 'relative_l2_p50', 'relative_l2_p95',
                  'peak_shift_mean', 'peak_shift_max_abs', 'peak_gain_db_mean',
                  'pslr_db_mean', 'islr_db_mean', 'mainlobe_width_mean',
                  'residual_improvement_db_mean', 'measured_cfo_abs_err_mean',
                  'measured_cfo_abs_err_p95', 'weak_preserved_rate']
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in cell_records:
            for path, agg in record['paths'].items():
                writer.writerow(dict(
                    grid=record['grid'], case_id=record['case_id'],
                    config=record['config'], code=record['code'],
                    multipath=record['multipath'], isr_db=record['isr_db'],
                    target_cfo_hz=record['target_cfo_hz'],
                    interferer_cfo_hz=record['interferer_cfo_hz'], path=path,
                    complete=record['complete'],
                    **{k: agg.get(k) for k in fields if k in agg}))

    # ---- markdown --------------------------------------------------------
    lines = ['# R06 functional matrix summary', '']
    lines.append('Command: `%s`' % provenance['command'])
    lines.append('')
    lines.append('| item | value |')
    lines.append('|---|---|')
    lines.append('| commit | `%s` |' % provenance['commit'])
    lines.append('| binary sha256 | `%s` |' % provenance['binary_sha256'])
    lines.append('| grids | %s |' % ', '.join(grids))
    lines.append('| cells (per grid) | %d |' % len(cells))
    lines.append('| cell-grid pairs | %d |' % len(pairs_all))
    lines.append('| seed cases | %d |' % (len(pairs_all) * args.seeds))
    if args.limit:
        lines.append('| pairs run (--limit) | %d |' % len(pairs))
    lines.append('| seeds per cell | %d |' % args.seeds)
    lines.append('| failed cells | %d |' % len(failures))
    lines.append('| dual target | %s |' % ('yes' if args.dual_target else 'no'))
    lines.append('')
    if failures:
        lines.append('## Failed cells (never dropped)')
        lines.append('')
        lines.append('| grid | cell | reason |')
        lines.append('|---|---|---|')
        for failure in failures:
            lines.append('| %s | %s | %s |' % (failure['grid'], failure['case_id'],
                                               failure['reason'].replace('|', '/')))
        lines.append('')
    incomplete = [r for r in cell_records if not r['complete']]
    if incomplete:
        lines.append('## Incomplete cells')
        lines.append('')
        lines.append('| grid | cell | rows | expected |')
        lines.append('|---|---|---|---|')
        for record in incomplete:
            got = sum(len(v) for v in record['paths'].values())
            lines.append('| %s | %s | %d | %d |'
                         % (record['grid'], record['case_id'], got,
                            record['expected_rows']))
        lines.append('')
    lines.append('## Per-axis aggregates')
    lines.append('')
    lines.append('`L2` is the complex relative L2 of the CIR against the paired clean '
                 'reference; `shift` is the measured minus expected peak tap; `CFO err` '
                 'is |measured - applied| target CFO. `SIC` counts committed rows.')
    lines.append('')
    for field in AXES:
        lines.extend(markdown_axis_table(field, field, cells, rows_by_cell, grids,
                                         ('clean', 'raw', 'sic')))
    lines.append('## Worst cells by SIC residual L2')
    lines.append('')
    worst = []
    for record in cell_records:
        sic = record['paths'].get('sic')
        if sic and sic['relative_l2_mean'] is not None:
            worst.append((sic['relative_l2_mean'], record))
    worst.sort(key=lambda item: -item[0])
    lines.append('| grid | cell | L2 mean | L2 P95 | FCS | SIC | residual dB |')
    lines.append('|---|---|---|---|---|---|---|')
    for value, record in worst[:25]:
        sic = record['paths']['sic']
        lines.append('| %s | %s | %.4f | %s | %d/%d | %d/%d | %s |'
                     % (record['grid'], record['case_id'], value,
                        fmt(sic['relative_l2_p95'], 4), sic['fcs'], sic['n'],
                        sic['sic_committed'], sic['n'],
                        fmt(sic['residual_improvement_db_mean'], 2)))
    lines.append('')
    (args.out / 'summary.md').write_text('\n'.join(lines) + '\n')

    print('\n%d/%d cell-grid pairs complete, %d failed (matrix is %d pairs / %d '
          'seed cases)' % (len(pairs) - len(failures), len(pairs), len(failures),
                           len(pairs_all), len(pairs_all) * args.seeds))
    print('wrote', args.out / 'summary.md', args.out / 'summary.json',
          args.out / 'cells.csv')
    return 1 if failures or incomplete else 0


if __name__ == '__main__':
    sys.exit(main())
