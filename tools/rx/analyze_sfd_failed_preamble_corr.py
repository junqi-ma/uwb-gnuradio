#!/usr/bin/env python3
"""Export 20 SFD_FAILED preprocessed windows and analyze single-preamble correlation."""
import argparse
import json
import math
import re
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

WINDOW_SAMPLES = 374217  # pre 36864 + capture 329161 + post 8192
EQ16_GROUP_DELAY_WORK = 519.0 / 48.0
FS_WORK = 998.4e6
SYM_LEN = 1016
N_SYNC = 256
N_SFD = 8
# IEEE 802.15.4z 8-symbol SFD: [+1, +1, +1, -1, +1, +1, -1, +1]
SFD_8SYM = np.array([1, 1, 1, -1, 1, 1, -1, 1], dtype=np.float32)


def parse_fields(line: str) -> dict:
    return dict(re.findall(r'(\w+)=([^ ]+)', line))


def pick_sfd_failed(log_path: Path, limit: int = 20) -> list[dict]:
    rows = []
    for line in log_path.read_text().splitlines():
        if not line.startswith('PACKET '):
            continue
        f = parse_fields(line)
        if f.get('status') == 'SFD_FAILED':
            start_nat = int(f['start'])
            win_nat = int(f['win']) if 'win' in f else max(0, start_nat - 36864)
            rows.append({
                'id': int(f['id']),
                'slot': int(f['slot']) if 'slot' in f else int(round(float(f['time_ms']))),
                'time_ms': float(f['time_ms']),
                'start_native': start_nat,
                'win_native': win_nat,
                'sync': int(f['sync']),
                'cfo_khz': float(f['cfo_khz']),
                'tmetric': float(f['tmetric']),
                'sfd_win': int(f.get('sfd_win', 0)),
                'path': f.get('path', 'unknown'),
            })
            if len(rows) >= limit:
                break
    return rows


def export_windows(rows: list[dict], source_sc16: Path, out_dir: Path,
                   export_bin: Path, res_root: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    for idx, r in enumerate(rows):
        prefix = out_dir / f'sfd_fail_{idx + 1:02d}_slot{r["slot"]}_id{r["id"]}'
        cf32_path = Path(str(prefix) + '_tone_cancelled_resampled.cf32')
        if not cf32_path.exists():
            subprocess.run(
                [
                    str(export_bin),
                    str(source_sc16),
                    str(r['win_native']),
                    str(WINDOW_SAMPLES),
                    str(prefix),
                    str(res_root),
                ],
                capture_output=True,
                text=True,
                check=True,
            )
            sc16_tmp = Path(str(prefix) + '_tone_cancelled.sc16')
            if sc16_tmp.exists():
                sc16_tmp.unlink()
        r['cf32_path'] = str(cf32_path)
    (out_dir / 'manifest.json').write_text(json.dumps(rows, indent=2) + '\n')


def analyze_window(cf32_path: Path, row: dict, tmpl: np.ndarray) -> dict:
    x = np.fromfile(cf32_path, dtype='<c8')
    rel_native = row['start_native'] - row['win_native']
    exp_start = int(round(rel_native * 65.0 / 48.0 + EQ16_GROUP_DELAY_WORK))

    # Fine-align start_work within +-8 samples using strongest of first 8 SYNCs
    best_offset = 0
    best_score = -1.0
    for off in range(-8, 9):
        score = 0.0
        for m in range(8):
            p = exp_start + off + m * SYM_LEN
            if 0 <= p and p + SYM_LEN <= len(x):
                seg = x[p:p + SYM_LEN]
                e = float(np.vdot(seg, seg).real)
                if e > 0:
                    c = np.vdot(tmpl, seg)
                    score += float((c.real * c.real + c.imag * c.imag) / e)
        if score > best_score:
            best_score = score
            best_offset = off
    start_work = exp_start + best_offset

    n_total = N_SYNC + N_SFD + 4  # 268 symbols
    corr_raw = np.zeros(n_total, dtype=np.complex64)
    seg_rms = np.zeros(n_total, dtype=np.float32)
    metric = np.zeros(n_total, dtype=np.float32)

    for k in range(n_total):
        p0 = start_work + k * SYM_LEN
        # Search +-3 samples around expected symbol start for max |corr|
        best_m = -1.0
        best_c = 0j
        best_e = 1.0
        for d in range(-3, 4):
            p = p0 + d
            if p < 0 or p + SYM_LEN > len(x):
                continue
            seg = x[p:p + SYM_LEN]
            e = float(np.vdot(seg, seg).real)
            if e <= 1e-12:
                continue
            c = np.vdot(tmpl, seg)
            m = float((c.real * c.real + c.imag * c.imag) / e)
            if m > best_m:
                best_m = m
                best_c = c
                best_e = e
        if best_m >= 0:
            corr_raw[k] = best_c / math.sqrt(best_e)
            seg_rms[k] = math.sqrt(best_e / SYM_LEN)
            metric[k] = best_m

    # Estimate CFO from ALL strong SYNC symbols (metric >= 0.15)
    sync_m = metric[:N_SYNC]
    good_k = np.where(sync_m >= 0.15)[0]
    if len(good_k) >= 4:
        phases = np.unwrap(np.angle(corr_raw[good_k]))
        # Linear fit: phase(k) = slope * k + intercept
        t_k = good_k.astype(np.float64) * (SYM_LEN / FS_WORK)
        slope, intercept = np.polyfit(t_k, phases, 1)
        est_cfo_hz = slope / (2.0 * math.pi)
    else:
        est_cfo_hz = row['cfo_khz'] * 1e3
        intercept = float(np.angle(corr_raw[0]))

    # Derotate correlation values by est_cfo_hz and align reference phase to +1
    t_all = np.arange(n_total, dtype=np.float64) * (SYM_LEN / FS_WORK)
    derot = corr_raw * np.exp(-1j * 2.0 * math.pi * est_cfo_hz * t_all)
    # Reference unit phasor from median SYNC real/imag after CFO derotation
    if len(good_k) >= 1:
        ref = np.median(derot[good_k].real) + 1j * np.median(derot[good_k].imag)
    else:
        ref = np.mean(derot[:N_SYNC])
    ref_unit = ref / (abs(ref) + 1e-12)
    signed_corr = (derot * np.conj(ref_unit)).real  # in [-1, 1]

    # Also compute SFD 8-symbol coherent projection against 8-symbol SFD
    sfd_slice = signed_corr[N_SYNC:N_SYNC + N_SFD]
    sfd_proj = float(np.dot(sfd_slice, -SFD_8SYM) / np.linalg.norm(SFD_8SYM))

    # Compute the actual algorithm's 8-symbol SFD matched-filter metric curve
    # over lag in [-64, +64] around start_work + N_SYNC * SYM_LEN
    sfd_tmpl = np.kron(SFD_8SYM, tmpl).astype(np.complex64)
    sfd_tmpl /= np.linalg.norm(sfd_tmpl)
    sfd_len = len(sfd_tmpl)
    p_sfd0 = start_work + N_SYNC * SYM_LEN
    lags = np.arange(-64, 65, dtype=np.int64)
    t_sfd = np.arange(sfd_len, dtype=np.float64) / FS_WORK

    def algo_curve(cfo_hz: float) -> np.ndarray:
        rot = np.exp(-1j * 2.0 * math.pi * cfo_hz * t_sfd).astype(np.complex64)
        curve = np.zeros(len(lags), dtype=np.float32)
        for idx_l, d in enumerate(lags):
            p = p_sfd0 + int(d)
            if p < 0 or p + sfd_len > len(x):
                continue
            seg = x[p:p + sfd_len] * rot
            e = float(np.vdot(seg, seg).real)
            if e > 1e-12:
                c = np.vdot(sfd_tmpl, seg)
                curve[idx_l] = float((c.real * c.real + c.imag * c.imag) / e)
        return curve

    algo_log = algo_curve(row['cfo_khz'] * 1e3)
    algo_est = algo_curve(est_cfo_hz)

    return {
        'start_work': start_work,
        'offset_from_exp': best_offset,
        'est_cfo_khz': est_cfo_hz / 1e3,
        'sync_med_metric': float(np.median(sync_m)),
        'sync_tail_med': float(np.median(sync_m[216:N_SYNC])),
        'sync_gt_025': int(np.sum(sync_m >= 0.25)),
        'sync_gt_015': int(np.sum(sync_m >= 0.15)),
        'sfd_signed': [float(v) for v in sfd_slice],
        'sfd_proj': sfd_proj,
        'metric': metric,
        'signed_corr': signed_corr,
        'algo_lags': lags,
        'algo_log': algo_log,
        'algo_est': algo_est,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', type=Path, default=Path('/tmp/uwb_exp_sfd1_sync1/d3p8m_period1ms.log'))
    ap.add_argument('--source', type=Path, default=Path('/media/oi/Data/uwb/clean_distance/2026-11-16/d3p8m_period1ms/rx_2s_r01.sc16'))
    ap.add_argument('--out-dir', type=Path, default=Path('/home/oi/Desktop/uwb-gnuradio/docs/performance/sfd_failed_20_20261116'))
    ap.add_argument('--export-bin', type=Path, default=Path('/tmp/uwb_window_diag/export_preprocessed'))
    ap.add_argument('--repo-root', type=Path, default=Path('/home/oi/Desktop/uwb-gnuradio/.worktrees/rx-sfd-sync-opt'))
    ap.add_argument('--limit', type=int, default=20)
    args = ap.parse_args()

    rows = pick_sfd_failed(args.log, args.limit)
    print(f"Selected {len(rows)} SFD_FAILED windows from {args.log}")
    export_windows(rows, args.source, args.out_dir, args.export_bin, args.repo_root / 'testdata')

    tmpl = np.fromfile(args.repo_root / 'testdata/reference_preamble_code10_998p4.cf32', dtype='<c8')
    assert len(tmpl) == SYM_LEN, len(tmpl)
    tmpl = tmpl / np.linalg.norm(tmpl)

    results = []
    print(f"\n{'#':>2} {'slot':>4} {'log_sync':>8} {'gt025':>5} {'log_cfo':>8} {'est_cfo':>8} {'sync_med':>8} {'tail_med':>8} {'sfd_proj':>8} | SFD[256..263] signed norm corr (expect 0 + 0 - + 0 0 -)")
    print('-' * 135)
    for i, r in enumerate(rows):
        res = analyze_window(Path(r['cf32_path']), r, tmpl)
        results.append((r, res))
        sfd_str = ' '.join(f'{v:+.2f}' for v in res['sfd_signed'])
        print(f"{i+1:>2} {r['slot']:>4} {r['sync']:>8} {res['sync_gt_025']:>5} {r['cfo_khz']:>+8.2f} {res['est_cfo_khz']:>+8.2f} {res['sync_med_metric']:>8.3f} {res['sync_tail_med']:>8.3f} {res['sfd_proj']:>+8.3f} | [{sfd_str}]")

    # Plot 20 cases: 5x4 grid of SFD region (248..267) signed correlation
    fig, axes = plt.subplots(5, 4, figsize=(18, 14), sharex=True, sharey=True)
    sym_x = np.arange(248, 268)
    for i, (r, res) in enumerate(results):
        ax = axes[i // 4, i % 4]
        y = res['signed_corr'][248:268]
        colors = ['steelblue' if k < 256 else ('crimson' if k < 264 else 'gray') for k in sym_x]
        ax.bar(sym_x, y, color=colors, width=0.8)
        ax.axhline(0, color='black', linewidth=0.6)
        ax.axvline(255.5, color='orange', linestyle='--', linewidth=0.8)
        ax.set_ylim(-0.85, 0.85)
        ax.set_title(f"#{i+1} slot={r['slot']} sync={r['sync']} cfo={r['cfo_khz']:+.1f}->{res['est_cfo_khz']:+.1f}k", fontsize=9)
        if i // 4 == 4:
            ax.set_xlabel('Symbol index (256..263 = SFD)')
        if i % 4 == 0:
            ax.set_ylabel('Signed norm corr')
    fig.suptitle('20 SFD_FAILED Windows: Single-Preamble Signed Correlation (blue=SYNC tail, red=SFD[256..263])', fontsize=13)
    fig.tight_layout()
    plot_path = args.out_dir / 'sfd_failed_20_corr.png'
    fig.savefig(plot_path, dpi=140)
    plt.close(fig)
    print(f"\nSaved plot to {plot_path}")

    # Second plot: actual algorithm 8-symbol SFD matched-filter metric over +-64 lags
    fig2, axes2 = plt.subplots(5, 4, figsize=(18, 14), sharex=True, sharey=True)
    for i, (r, res) in enumerate(results):
        ax = axes2[i // 4, i % 4]
        lags = res['algo_lags']
        ax.plot(lags, res['algo_est'], color='steelblue', linewidth=1.2, label='full-train CFO')
        if abs(r['cfo_khz'] - res['est_cfo_khz']) > 0.5:
            ax.plot(lags, res['algo_log'], color='darkorange', linewidth=1.0, linestyle='--', label='logged CFO')
        ax.axhline(0.30, color='crimson', linestyle='--', linewidth=1.0, label='thr=0.30')
        pk_est = float(np.max(res['algo_est']))
        pk_log = float(np.max(res['algo_log']))
        ax.set_ylim(0.0, 0.36)
        ax.set_title(f"#{i+1} slot={r['slot']} pk={pk_log:.3f} (est={pk_est:.3f})", fontsize=9)
        if i == 0:
            ax.legend(fontsize=7, loc='upper right')
        if i // 4 == 4:
            ax.set_xlabel('Lag from expected SFD start (samples)')
        if i % 4 == 0:
            ax.set_ylabel('8-sym SFD metric')
    fig2.suptitle('20 SFD_FAILED Windows: Algorithm 8-Symbol SFD Metric (|dot|^2 / pwr) vs 0.30 Threshold', fontsize=13)
    fig2.tight_layout()
    plot_path2 = args.out_dir / 'sfd_failed_20_algo_metric.png'
    fig2.savefig(plot_path2, dpi=140)
    plt.close(fig2)
    print(f"Saved algorithm metric plot to {plot_path2}")

    # Save CSV summary
    import csv
    csv_path = args.out_dir / 'summary.csv'
    with csv_path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['idx', 'slot', 'id', 'log_sync', 'gt025', 'log_cfo_khz', 'est_cfo_khz',
                    'sync_med_metric', 'tail_med_metric', 'sfd_sym_256', 'sfd_sym_257',
                    'sfd_sym_258', 'sfd_sym_259', 'sfd_sym_260', 'sfd_sym_261',
                    'sfd_sym_262', 'sfd_sym_263', 'cf32_file'])
        for i, (r, res) in enumerate(results):
            w.writerow([
                i + 1, r['slot'], r['id'], r['sync'], res['sync_gt_025'],
                f"{r['cfo_khz']:.3f}", f"{res['est_cfo_khz']:.3f}",
                f"{res['sync_med_metric']:.4f}", f"{res['sync_tail_med']:.4f}",
                *[f"{v:+.4f}" for v in res['sfd_signed']],
                Path(r['cf32_path']).name,
            ])
    print(f"Saved CSV to {csv_path}")


if __name__ == '__main__':
    main()
