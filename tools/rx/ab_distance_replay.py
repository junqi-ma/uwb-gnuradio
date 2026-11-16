#!/usr/bin/env python3
"""Aggregate paired distance-replay runs produced by run_distance_replay.py.

Usage: ab_distance_replay.py OUT [--labels a,b] [--runs 3]

Reads /tmp/opencode/fin-<label>-<i>/summary.json, checks every run is valid
(raw_drop_samples == 0), and prints per-distance paired means plus the
per-stage delta.  Timing is only meaningful for runs taken on an idle machine;
the script reports the sample count so a reader can judge.
"""
import argparse
import json
import statistics as st
from pathlib import Path

STAGES = ['st_timing', 'st_sfd', 'sfd_cforef', 'st_cfo', 'st_cir',
          'st_payload', 'resample_us', 'demod_us', 'compute_us']
FAILS = ['NO_WINDOW', 'TIMING_FAILED', 'CFO_FAILED', 'SFD_FAILED',
         'PAYLOAD_FAILED', 'FCS_FAILED']


def load(root, label, runs):
    out = []
    for i in range(1, runs + 1):
        p = Path(root) / f'{label}-{i}' / 'summary.json'
        if not p.exists():
            continue
        d = json.loads(p.read_text())
        valid = all(str(r.get('raw_drop_samples', '0')) in ('0', 'None')
                    for r in d['results'])
        out.append((i, d, valid))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('--labels', default='base,final')
    ap.add_argument('--runs', type=int, default=3)
    a = ap.parse_args()
    la, lb = a.labels.split(',')
    A = load(a.root, la, a.runs)
    B = load(a.root, lb, a.runs)
    for name, runs in ((la, A), (lb, B)):
        bad = [i for i, _, v in runs if not v]
        print(f'{name}: {len(runs)} runs, invalid (drops): {bad or "none"}')
    A = [(i, d) for i, d, v in A if v]
    B = [(i, d) for i, d, v in B if v]
    dists = [r['distance_m'] for r in A[0][1]['results']]

    def per(runs):
        m = {}
        for d in dists:
            rows = [next(r for r in dd['results'] if r['distance_m'] == d)
                    for _, dd in runs]
            m[d] = dict(
                ok=st.mean(r['ok_slots'] for r in rows),
                prr=st.mean(r['prr_pct'] for r in rows),
                fails={k: st.mean(r['failures'][k] for r in rows) for k in FAILS},
                **{s: st.mean(r['stages'][s] for r in rows) for s in STAGES})
        return m

    ma, mb = per(A), per(B)
    hdr = (f"{'dist':>5} {'PRR b':>7} {'PRR n':>7} {'OK b':>6} {'OK n':>6} "
           f"{'NOWIN':>11} {'TIM':>9} {'SFD':>9} {'PAY':>9} "
           f"{'demod b':>8} {'demod n':>8} {'d%':>7} "
           f"{'comp b':>8} {'comp n':>8} {'d%':>7}")
    print(hdr)
    for d in dists:
        x, y = ma[d], mb[d]
        def pair(k):
            return f"{x['fails'][k]:.1f}/{y['fails'][k]:.1f}"
        print(f"{d:>5.1f} {x['prr']:7.2f} {y['prr']:7.2f} "
              f"{x['ok']:6.1f} {y['ok']:6.1f} {pair('NO_WINDOW'):>11} "
              f"{pair('TIMING_FAILED'):>9} {pair('SFD_FAILED'):>9} "
              f"{pair('PAYLOAD_FAILED'):>9} "
              f"{x['demod_us']:8.1f} {y['demod_us']:8.1f} "
              f"{100*(y['demod_us']/x['demod_us']-1):7.1f} "
              f"{x['compute_us']:8.1f} {y['compute_us']:8.1f} "
              f"{100*(y['compute_us']/x['compute_us']-1):7.1f}")
    print()
    print(f"{'stage':>12} {'base':>9} {'new':>9} {'delta':>9} {'delta%':>8}")
    for s in STAGES:
        xb = st.mean(ma[d][s] for d in dists)
        yb = st.mean(mb[d][s] for d in dists)
        print(f"{s:>12} {xb:9.1f} {yb:9.1f} {yb-xb:+9.1f} "
              f"{100*(yb/xb-1):+8.1f}")
    tot_a = sum(ma[d]['ok'] for d in dists)
    tot_b = sum(mb[d]['ok'] for d in dists)
    print(f"\nCRC-OK total: {tot_a:.0f} -> {tot_b:.0f} "
          f"({tot_b-tot_a:+.0f} of {2000*len(dists)})")


if __name__ == '__main__':
    main()
