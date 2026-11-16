#!/usr/bin/env python3
"""Summarize existing 1 ms distance replay logs without reading raw IQ."""
import argparse
import collections
import csv
import json
import math
from pathlib import Path
import re


ORDER = ['INCOMPLETE_WINDOW', 'TIMING_FAILED', 'CFO_FAILED', 'SFD_FAILED',
         'CIR_FAILED', 'PHR_FAILED', 'PAYLOAD_FAILED', 'FCS_FAILED', 'OK']


def fields(line):
    return dict(re.findall(r'(\w+)=([^ ]+)', line))


def summarize(folder):
    stem = folder / 'rx_2s_r01'
    meta = json.loads(Path(str(stem) + '.sc16.json').read_text())
    prior = json.loads(Path(str(stem) + '.summary.json').read_text())
    lines = Path(str(stem) + '.demod.log').read_text().splitlines()
    rows = [fields(line) for line in lines if line.startswith('PACKET ')]
    assert all(r['status'] in ORDER for r in rows)
    good = [r for r in rows if r['status'] == 'OK' and r.get('crc') == '1']
    assert all(r.get('crc') == '1' for r in rows if r['status'] == 'OK')
    # Phase only: preserve leading failed slots when first success is late.
    # With no success use the first candidate; this is not an independent TX grid.
    reference = float((good or rows)[0]['time_ms'])
    phase = reference - math.floor(reference)
    expected = round(meta['got'] / meta['rate'] * 1000)
    slots = collections.defaultdict(list)
    outside = []
    for r in rows:
        k = round(float(r['time_ms']) - phase)
        (slots[k] if 0 <= k < expected else outside).append(r)
    counts = collections.Counter()
    failed_slots = []
    for k in range(expected):
        attempts = slots[k]
        status = max((r['status'] for r in attempts), key=ORDER.index,
                     default='NO_WINDOW')
        counts[status] += 1
        if status != 'OK':
            failed_slots.append(dict(slot=k, time_ms=k+phase, status=status,
                                     attempts=len(attempts)))
    assert sum(counts.values()) == expected
    assert counts['OK'] == prior['correct_unique_packets']
    cfo = [r for r in rows if r['status'] == 'CFO_FAILED']
    sfd = [r for r in rows if r['status'] == 'SFD_FAILED']
    detail = dict(
        cfo_sync_lt4=sum(int(r.get('sync', 0)) < 4 for r in cfo),
        cfo_before_sfd=sum(int(r.get('sfd_win', 0)) == 0 for r in cfo),
        sfd_bootstrap_only=sum(int(r.get('sfd_win', 0)) > 0 and
                               int(r.get('sfd_cforef', 0)) == 0 and
                               int(r.get('sfd_final', 0)) == 0 for r in sfd),
        timing_bounded_fail=sum(r['status'] == 'TIMING_FAILED' and
                                r.get('bounded_fail') == '1' for r in rows))
    final = fields(next(l for l in lines if l.startswith('FINAL ')))
    lock = fields(next(l for l in lines if l.startswith('LOCK_STATS ')))
    return dict(distance_m=float(folder.name.split('_')[0][1:-1].replace('p', '.')),
                source_log=str(Path(str(stem)+'.demod.log')),
                expected_slots=expected, phase_ms=phase,
                phase_source='first_crc_ok' if good else 'first_failed_candidate',
                slot_counts=dict(counts), window_counts=dict(collections.Counter(r['status'] for r in rows)),
                windows=len(rows), duplicate_good_windows=len(good)-counts['OK'],
                out_of_capture_grid_windows=len(outside), detail=detail,
                final=final, lock=lock, failed_slots=failed_slots,
                max_candidate_phase_residual_ms=max(abs(float(r['time_ms'])-phase-
                    round(float(r['time_ms'])-phase)) for r in rows))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    results = sorted([summarize(p) for p in args.root.glob('d*m_period1ms')],
                     key=lambda r: r['distance_m'])
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out/'metrics.json').write_text(json.dumps(results, indent=2)+'\n')
    columns = ['distance_m', 'expected_slots', 'OK', 'NO_WINDOW'] + ORDER[:-1]
    with (args.out/'slot_counts.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for r in results:
            writer.writerow(dict(distance_m=r['distance_m'], expected_slots=r['expected_slots'],
                                 **{k: r['slot_counts'].get(k, 0) for k in columns[2:]}))
    for r in results:
        print(r['distance_m'], r['slot_counts'], 'detail', r['detail'],
              'outside', r['out_of_capture_grid_windows'])


if __name__ == '__main__':
    main()
