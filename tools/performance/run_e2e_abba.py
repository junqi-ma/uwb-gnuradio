#!/usr/bin/env python3
"""ABBA paired E2E benchmark for the 2026-11-14 low-hanging-fruit optimisation.

Runs the SAME fixture and the SAME CLI against two pre-built
`benchmark_sc16_pipeline_steady` binaries (control = bd49267, treatment = the
merged tracks) in A B B A order, repeating `--groups` times, and reports the
paired per-phase means plus the steady-state throughput.

Every run passes `--benchmark`, so the binary itself holds
`/tmp/uwb-sic-benchmark.lock` for the whole run.  Nothing is averaged across a
run whose ledger invariants failed or which produced fewer measured windows
than requested: those are counted and reported separately.

Usage:
  run_e2e_abba.py --control <bin> --treatment <bin> --fixtures <dir> \
      --out <dir> [--groups 3] [--warmup 8] [--measure 24] \
      [--configs A64_clean,A256_jam] [--workers 1]
"""

import argparse
import json
import os
import statistics
import subprocess
import sys
from pathlib import Path

MODES = {
    'A64':  dict(sync_reps=64,  period=127572, capture=126772, consumer='sic_sensing'),
    'A256': dict(sync_reps=256, period=271626, capture=270826, consumer='sic_sensing'),
    'B64':  dict(sync_reps=64,  period=257636, capture=256836, consumer='sic_comm'),
    'B256': dict(sync_reps=256, period=401689, capture=400889, consumer='sic_comm'),
}

PHASES = ('resample_us_mean', 'sic_us_mean', 'sensing_us_mean', 'comm_us_mean',
          'e2e_us_mean')

# Deterministic per-window fields that must be identical between the two arms.
# The three tracks are supposed to change COST, not decisions.
EXACT_FIELDS = ('state', 'native_start_abs', 'native_sample_count',
                'sic_committed', 'radar_sfd_found', 'radar_detected_start',
                'radar_cir_origin', 'radar_detected_origin', 'radar_valid_reps',
                'comm_ok', 'comm_phr_ok', 'comm_phr_length', 'comm_fcs_ok',
                'comm_payload_bytes', 'comm_psdu_fnv1a', 'comm_sync_origin',
                'comm_sfd_start')
# Float fields: compared with a tolerance, worst delta reported.
NEAR_FIELDS = ('radar_peak_metric', 'radar_cfo_hz', 'comm_cfo_hz',
               'actual_suppression_db')


def functional_diff(control_windows, treatment_windows):
    """Compare the deterministic per-window fields of the two arms."""
    n = min(len(control_windows), len(treatment_windows))
    mismatches = []
    for i in range(n):
        c, t = control_windows[i], treatment_windows[i]
        for key in EXACT_FIELDS:
            if c.get(key) != t.get(key):
                mismatches.append('win%d.%s: %r != %r'
                                  % (i, key, c.get(key), t.get(key)))
    near = {}
    for key in NEAR_FIELDS:
        worst, at = 0.0, -1
        for i in range(n):
            d = abs(float(control_windows[i].get(key, 0.0)) -
                    float(treatment_windows[i].get(key, 0.0)))
            if d > worst:
                worst, at = d, i
        near[key] = dict(max_abs_delta=worst, window=at)
    return dict(n=n, mismatches=mismatches, near=near)


def configs(names):
    out = []
    for name in names:
        mode, variant = name.split('_')
        m = MODES[mode]
        out.append(dict(name=name, variant=variant, **m))
    return out


def build_cmd(binary, fixtures, cfg, workers, warmup, measure, label, cpus=''):
    cmd = []
    if cpus:
        cmd += ['taskset', '-c', cpus]
    cmd += [
        str(binary),
        '--file', str(Path(fixtures) / (cfg['name'] + '.sc16')),
        '--native-rate', '737280000', '--work-rate', '998400000',
        '--capture', str(cfg['capture']),
        '--pre-guard', '400', '--post-guard', '400',
        '--period', str(cfg['period']), '--t0', '400',
        '--consumer', cfg['consumer'],
        '--resampler-profile', 'realtime_eq16',
        '--radar-code', '9', '--radar-sync-reps', str(cfg['sync_reps']),
        '--comm-code', '9', '--comm-sync-reps', str(cfg['sync_reps']),
        '--sic-code', '10', '--sic-sync-reps', '64',
        '--sic-policy', 'preamble', '--sic-full-window', '0', '--sic-stride', '1',
        '--dsp-workers', str(workers),
        '--warmup-windows', str(warmup), '--measure-windows', str(measure),
        '--flow-in-flight', str(4 * workers),
        '--label', label, '--benchmark',
    ]
    return cmd


def run_one(binary, fixtures, cfg, workers, warmup, measure, label, out_dir, cpus=''):
    json_path = out_dir / (cfg['name'] + '_' + label + '.json')
    cmd = build_cmd(binary, fixtures, cfg, workers, warmup, measure, label, cpus)
    cmd += ['--json', str(json_path)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return None, 'exit %d: %s' % (r.returncode,
                                      (r.stderr.strip().splitlines() or ['?'])[-1])
    try:
        doc = json.loads(json_path.read_text())
        meta = doc['meta']
        meta['_windows'] = doc['windows']
    except Exception as exc:  # noqa: BLE001
        return None, 'json: %s' % exc
    if not meta.get('ready_ok') or not meta.get('ledger_invariants_ok'):
        return None, 'ledger/ready failed'
    if meta.get('measure_windows_actual', 0) != measure:
        return None, 'only %s measured windows' % meta.get('measure_windows_actual')
    return meta, ''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--control', type=Path, required=True)
    ap.add_argument('--treatment', type=Path, required=True)
    ap.add_argument('--fixtures', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--groups', type=int, default=3)
    ap.add_argument('--warmup', type=int, default=8)
    ap.add_argument('--measure', type=int, default=24)
    ap.add_argument('--workers', type=int, default=1)
    ap.add_argument('--cpus', default='',
                    help="taskset -c value for the whole benchmark process")
    ap.add_argument('--configs', default=','.join(
        '%s_%s' % (m, v) for m in MODES for v in ('clean', 'jam')))
    a = ap.parse_args()

    a.out.mkdir(parents=True, exist_ok=True)
    cfgs = configs(a.configs.split(','))
    results = {}
    failures = []

    for cfg in cfgs:
        arms = {'control': [], 'treatment': []}
        groups = []
        for g in range(a.groups):
            slot_metas = {'control': [], 'treatment': []}
            for arm, slot in (('control', 1), ('treatment', 1),
                              ('treatment', 2), ('control', 2)):
                binary = a.control if arm == 'control' else a.treatment
                label = '%s_g%d_%d' % (arm, g + 1, slot)
                meta, err = run_one(binary, a.fixtures, cfg, a.workers,
                                    a.warmup, a.measure, label, a.out, a.cpus)
                if meta is None:
                    failures.append('%s %s: %s' % (cfg['name'], label, err))
                    print('FAIL %-12s %-22s %s' % (cfg['name'], label, err))
                    continue
                arms[arm].append(meta)
                slot_metas[arm].append(meta)
                print('%-12s %-22s steady=%9.2f us  sic=%8.2f  sens=%8.2f  comm=%8.2f'
                      % (cfg['name'], label, meta['steady_window_us'],
                         meta['sic_us_mean'], meta['sensing_us_mean'],
                         meta['comm_us_mean']))
            groups.append(slot_metas)
        results[cfg['name']] = dict(arms=arms, groups=groups)

    summary = {}
    print()
    print('%-12s %-9s %4s %10s %10s %8s   %s'
          % ('config', 'arm', 'n', 'steady', 'e2e', 'ratio', 'functional'))
    for name, res in results.items():
        arms = res['arms']
        c, t = arms['control'], arms['treatment']
        if not c or not t:
            continue
        row = {}
        for arm, metas in (('control', c), ('treatment', t)):
            d = dict(n=len(metas))
            for key in ('steady_window_us',) + PHASES:
                vals = [m[key] for m in metas]
                d[key] = statistics.fmean(vals)
                d[key + '_p50'] = statistics.median(vals)
            d['sic_committed'] = sum(m['sic_committed'] for m in metas)
            d['radar_sfd_found'] = sum(m['radar_sfd_found'] for m in metas)
            d['comm_ok'] = sum(m['comm_ok'] for m in metas)
            d['comm_fcs_ok'] = sum(m['comm_fcs_ok'] for m in metas)
            d['comm_attempted'] = sum(m['comm_attempted'] for m in metas)
            d['radar_executed'] = sum(m['radar_executed'] for m in metas)
            # Informational: the SIC bypass reason.  Track B deliberately
            # changes clean windows from `below_detect_threshold` to
            # `no_outside_autocorr_preamble`, so this is reported, not asserted.
            reasons = {}
            for m in metas:
                for w in m['_windows']:
                    r = w.get('sic_status') or ''
                    reasons[r] = reasons.get(r, 0) + 1
            d['sic_status_histogram'] = reasons
            row[arm] = d
        diff = functional_diff(c[0]['_windows'], t[0]['_windows'])
        row['functional_diff'] = diff
        # Paired within-group ratio: mean(treatment slots) / mean(control slots)
        # for that group.  The median across groups is the headline number; the
        # spread is reported so a noisy session is visible instead of hidden.
        paired = {}
        for key in ('steady_window_us',) + PHASES:
            rs = []
            for g in res['groups']:
                if len(g['control']) == 2 and len(g['treatment']) == 2:
                    cm = statistics.fmean(m[key] for m in g['control'])
                    tm = statistics.fmean(m[key] for m in g['treatment'])
                    if cm:
                        rs.append(tm / cm)
            if rs:
                paired[key] = dict(median=statistics.median(rs),
                                   min=min(rs), max=max(rs), n=len(rs))
        row['paired_ratio'] = paired
        row['ratio'] = {k: (row['treatment'][k] / row['control'][k]
                            if row['control'][k] else float('nan'))
                        for k in ('steady_window_us',) + PHASES}
        summary[name] = row
        print('%-12s %-9s %4d %10.2f %10.2f %8.4f   sic=%d/%d sfd=%d/%d '
              'comm=%d/%d fcs=%d/%d'
              % (name, 'control', row['control']['n'],
                 row['control']['steady_window_us'], row['control']['e2e_us_mean'],
                 1.0, row['control']['sic_committed'], row['control']['n'],
                 row['control']['radar_sfd_found'], row['control']['radar_executed'],
                 row['control']['comm_ok'], row['control']['comm_attempted'],
                 row['control']['comm_fcs_ok'], row['control']['comm_attempted']))
        if diff['mismatches']:
            print('    !! %d functional mismatches, first: %s'
                  % (len(diff['mismatches']), diff['mismatches'][0]))
        pr = row['paired_ratio'].get('steady_window_us')
        if pr:
            print('    paired steady ratio: median %.4f  [%.4f .. %.4f]  n=%d'
                  % (pr['median'], pr['min'], pr['max'], pr['n']))
        pr = row['paired_ratio'].get('e2e_us_mean')
        if pr:
            print('    paired e2e    ratio: median %.4f  [%.4f .. %.4f]  n=%d'
                  % (pr['median'], pr['min'], pr['max'], pr['n']))
        print('%-12s %-9s %4d %10.2f %10.2f %8.4f   sic=%d/%d sfd=%d/%d '
              'comm=%d/%d fcs=%d/%d'
              % (name, 'treatment', row['treatment']['n'],
                 row['treatment']['steady_window_us'], row['treatment']['e2e_us_mean'],
                 row['ratio']['steady_window_us'],
                 row['treatment']['sic_committed'], row['treatment']['n'],
                 row['treatment']['radar_sfd_found'], row['treatment']['radar_executed'],
                 row['treatment']['comm_ok'], row['treatment']['comm_attempted'],
                 row['treatment']['comm_fcs_ok'], row['treatment']['comm_attempted']))

    (a.out / 'summary.json').write_text(json.dumps(
        dict(workers=a.workers, warmup=a.warmup, measure=a.measure, cpus=a.cpus,
             groups=a.groups, summary=summary, failures=failures), indent=2) + '\n')
    print()
    print('failures:', len(failures))
    for f in failures:
        print('  ', f)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
