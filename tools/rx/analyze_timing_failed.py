#!/usr/bin/env python3
"""Export 12 typical TIMING_FAILED windows and plot waveform, per-symbol SYNC metric, and 256-symbol CIR."""
import argparse
import csv
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
SYM_LEN = 1016
N_SYNC = 256


def parse_fields(line: str) -> dict:
    return dict(re.findall(r'(\w+)=([^ ]+)', line))


def pick_timing_failed(log_path: Path, limit: int = 12) -> list[dict]:
    all_tf = []
    for line in log_path.read_text().splitlines():
        if not line.startswith('PACKET '):
            continue
        f = parse_fields(line)
        if f.get('status') == 'TIMING_FAILED' and f.get('path') == 'target':
            pred_nat = int(f['pred'])
            win_nat = int(f['win'])
            all_tf.append({
                'id': int(f['id']),
                'slot': int(f.get('slot', 0)),
                'time_ms': float(f['time_ms']),
                'pred_native': pred_nat,
                'win_native': win_nat,
                'bounded_fail': int(f.get('bounded_fail', 0)),
                'roi': int(f.get('roi', 256)),
            })
    if len(all_tf) <= limit:
        return all_tf
    indices = [i * len(all_tf) // limit for i in range(limit)]
    return [all_tf[i] for i in indices]


def export_windows(rows: list[dict], source_sc16: Path, out_dir: Path,
                   export_bin: Path, res_root: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    for idx, r in enumerate(rows):
        prefix = out_dir / f'timing_fail_{idx + 1:02d}_slot{r["slot"]}_id{r["id"]}'
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


def analyze_window(cf32_path: Path, row: dict, tmpl_conj: np.ndarray) -> dict:
    x = np.fromfile(cf32_path, dtype='<c8')
    rel_native = row['pred_native'] - row['win_native']
    exp_start = int(round(rel_native * 65.0 / 48.0 + EQ16_GROUP_DELAY_WORK))

    # 1. Compute 256-symbol incoherent normalized CIR across lag in [-256, +256]
    #    Also track per-symbol correlation at each lag using FFT / vectorized slice.
    lags = np.arange(-256, 257)
    cir_256 = np.zeros(len(lags), dtype=np.float64)
    cir_per_sym = np.zeros((N_SYNC, len(lags)), dtype=np.float32)
    # Use vectorized correlation per symbol over [-256, +256]
    # Each symbol k starts at exp_start + k * SYM_LEN
    valid_syms = 0
    for k in range(N_SYNC):
        base = exp_start + k * SYM_LEN
        lo = base - 256
        hi = base + 256 + SYM_LEN
        if lo < 0 or hi > len(x):
            continue
        seg_win = x[lo:hi]  # length 512 + SYM_LEN
        # Sliding dot product with tmpl_conj via FFT or direct correlate
        dots = np.correlate(seg_win, np.conj(tmpl_conj), mode='valid')  # wait: np.correlate(a, v) = sum a * conj(v)
        # Let's be explicit: tmpl is unit-norm, so pass tmpl directly to correlate(seg_win, tmpl)
        p_sq = (seg_win.real * seg_win.real + seg_win.imag * seg_win.imag).astype(np.float64)
        cs = np.concatenate(([0.0], np.cumsum(p_sq)))
        e_win = cs[SYM_LEN:] - cs[:-SYM_LEN]
        e_win = np.maximum(e_win, 1e-12)
        met_k = (dots.real * dots.real + dots.imag * dots.imag) / e_win
        cir_per_sym[k, :] = met_k.astype(np.float32)
        cir_256 += met_k
        valid_syms += 1

    if valid_syms > 0:
        cir_256 /= valid_syms

    best_lag_idx = int(np.argmax(cir_256))
    best_lag = int(lags[best_lag_idx])
    cir_peak = float(cir_256[best_lag_idx])
    bg_mask = np.abs(lags - best_lag) > 12
    cir_bg = float(np.mean(cir_256[bg_mask])) if np.any(bg_mask) else 1e-9
    cir_par = cir_peak / max(cir_bg, 1e-9)

    # 2. Per-symbol metric at best_lag (and local +-2 samples around best_lag)
    sym_metrics = np.zeros(N_SYNC, dtype=np.float32)
    sym_rms = np.zeros(N_SYNC, dtype=np.float32)
    for k in range(N_SYNC):
        base = exp_start + best_lag + k * SYM_LEN
        best_m = 0.0
        for d in range(-2, 3):
            p = base + d
            if 0 <= p and p + SYM_LEN <= len(x):
                seg = x[p:p + SYM_LEN]
                e = float(np.vdot(seg, seg).real)
                if e > 0:
                    c = np.vdot(np.conj(tmpl_conj), seg)
                    m = float((c.real * c.real + c.imag * c.imag) / e)
                    if m > best_m:
                        best_m = m
        sym_metrics[k] = best_m
        if 0 <= base and base + SYM_LEN <= len(x):
            seg = x[base:base + SYM_LEN]
            sym_rms[k] = float(np.sqrt(np.mean(seg.real ** 2 + seg.imag ** 2)))

    # 2b. Continuous 1D matched filter output across the entire 256-symbol preamble
    cont_pre = 508
    cont_lo = max(0, exp_start - cont_pre)
    cont_hi = min(len(x), exp_start + N_SYNC * SYM_LEN + cont_pre + SYM_LEN)
    seg_full = x[cont_lo:cont_hi]
    dots_full = np.correlate(seg_full, np.conj(tmpl_conj), mode='valid')
    p_full = (seg_full.real * seg_full.real + seg_full.imag * seg_full.imag).astype(np.float64)
    cs_full = np.concatenate(([0.0], np.cumsum(p_full)))
    e_full = np.maximum(cs_full[SYM_LEN:] - cs_full[:-SYM_LEN], 1e-12)
    cont_metric = ((dots_full.real * dots_full.real + dots_full.imag * dots_full.imag) / e_full).astype(np.float32)
    cont_x_sym = (np.arange(len(cont_metric), dtype=np.float32) + (cont_lo - exp_start)) / float(SYM_LEN)

    # 3. Waveform envelope (block RMS every 254 samples = 4 points/symbol) over [-15, +310] symbols
    step = 254
    sym_lo, sym_hi = -15, 310
    n_blocks = (sym_hi - sym_lo) * (SYM_LEN // step)
    wf_x_sym = np.zeros(n_blocks, dtype=np.float32)
    wf_rms = np.zeros(n_blocks, dtype=np.float32)
    wf_max = np.zeros(n_blocks, dtype=np.float32)
    start_samp = exp_start + sym_lo * SYM_LEN
    for b in range(n_blocks):
        s0 = start_samp + b * step
        s1 = s0 + step
        wf_x_sym[b] = sym_lo + (b + 0.5) * step / SYM_LEN
        if 0 <= s0 and s1 <= len(x):
            seg = x[s0:s1]
            mag = np.abs(seg)
            wf_rms[b] = float(np.sqrt(np.mean(mag * mag)))
            wf_max[b] = float(np.max(mag))

    first_5_max = float(np.max(sym_metrics[:5]))
    gt025_all = int(np.sum(sym_metrics >= 0.25))
    gt025_first5 = int(np.sum(sym_metrics[:5] >= 0.25))
    med_all = float(np.median(sym_metrics))
    med_front = float(np.median(sym_metrics[:16]))
    med_tail = float(np.median(sym_metrics[32:]))

    return {
        'exp_start': exp_start,
        'best_lag': best_lag,
        'cir_peak': cir_peak,
        'cir_par': cir_par,
        'lags': lags,
        'cir_256': cir_256,
        'cont_x_sym': cont_x_sym,
        'cont_metric': cont_metric,
        'sym_metrics': sym_metrics,
        'sym_rms': sym_rms,
        'wf_x_sym': wf_x_sym,
        'wf_rms': wf_rms,
        'wf_max': wf_max,
        'first_5_max': first_5_max,
        'gt025_all': gt025_all,
        'gt025_first5': gt025_first5,
        'med_all': med_all,
        'med_front': med_front,
        'med_tail': med_tail,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log', type=Path, default=Path('/tmp/uwb_exp_cfo_tail_008/d3p8m_period1ms.log'))
    ap.add_argument('--source', type=Path, default=Path('/media/oi/Data/uwb/clean_distance/2026-11-16/d3p8m_period1ms/rx_2s_r01.sc16'))
    ap.add_argument('--out-dir', type=Path, default=Path('/home/oi/Desktop/uwb-gnuradio/docs/performance/timing_failed_12_20261116'))
    ap.add_argument('--export-bin', type=Path, default=Path('/tmp/uwb_window_diag/export_preprocessed'))
    ap.add_argument('--repo-root', type=Path, default=Path('/home/oi/Desktop/uwb-gnuradio/.worktrees/rx-sfd-sync-opt'))
    ap.add_argument('--limit', type=int, default=12)
    args = ap.parse_args()

    rows = pick_timing_failed(args.log, args.limit)
    print(f"Selected {len(rows)} TIMING_FAILED windows from {args.log}")
    export_windows(rows, args.source, args.out_dir, args.export_bin, args.repo_root / 'testdata')

    tmpl = np.fromfile(args.repo_root / 'testdata/reference_preamble_code10_998p4.cf32', dtype='<c8')
    assert len(tmpl) == SYM_LEN, len(tmpl)
    tmpl = tmpl / np.linalg.norm(tmpl)
    tmpl_conj = np.conj(tmpl)

    results = []
    print(f"\n{'#':>2} {'slot':>4} {'t_ms':>7} {'lag':>5} {'cir256':>7} {'par':>6} {'f5_max':>6} {'gt025':>5} {'med_0_15':>8} {'med_32+':>8}")
    print('-' * 78)
    for i, r in enumerate(rows):
        res = analyze_window(Path(r['cf32_path']), r, tmpl_conj)
        results.append((r, res))
        print(f"{i+1:>2} {r['slot']:>4} {r['time_ms']:>7.1f} {res['best_lag']:>+5d} "
              f"{res['cir_peak']:>7.4f} {res['cir_par']:>6.1f} {res['first_5_max']:>6.3f} "
              f"{res['gt025_all']:>5d} {res['med_front']:>8.3f} {res['med_tail']:>8.3f}")

    n_cases = len(results)
    ncols = 4
    nrows = math.ceil(n_cases / ncols)
    fig_h = 3.2 * nrows

    # Plot 1: Tone-cancelled & resampled waveform envelope
    fig1, axes1 = plt.subplots(nrows, ncols, figsize=(18, fig_h), sharex=True, squeeze=False)
    for i, (r, res) in enumerate(results):
        ax = axes1[i // ncols, i % ncols]
        ax.plot(res['wf_x_sym'], res['wf_rms'], color='steelblue', linewidth=0.8, label='RMS')
        ax.plot(res['wf_x_sym'], res['wf_max'], color='lightsteelblue', linewidth=0.5, alpha=0.6, label='Peak')
        ax.axvspan(0, 5, color='orange', alpha=0.25, label='SYNC[0..4] search')
        ax.axvline(256, color='crimson', linestyle='--', linewidth=0.8, label='SFD@256')
        ax.axvline(264, color='green', linestyle=':', linewidth=0.8, label='PHR@264')
        ax.set_title(f"#{i+1} slot={r['slot']} t={r['time_ms']:.1f}ms lag={res['best_lag']:+d}", fontsize=9)
        if i == 0:
            ax.legend(fontsize=7, loc='upper right')
        if i // ncols == nrows - 1:
            ax.set_xlabel('Symbol index rel. pred')
        if i % ncols == 0:
            ax.set_ylabel('Amplitude |x|')
    fig1.suptitle(f'{n_cases} TIMING_FAILED Windows: Tone-Cancelled & Resampled Waveform Envelope (998.4 MS/s)', fontsize=13)
    fig1.tight_layout()
    p1 = args.out_dir / 'timing_failed_waveform.png'
    fig1.savefig(p1, dpi=140)
    plt.close(fig1)

    # Plot 2: Per-symbol SYNC[0..255] matched filter metric
    fig2, axes2 = plt.subplots(nrows, ncols, figsize=(18, fig_h), sharex=True, sharey=True, squeeze=False)
    sym_idx = np.arange(N_SYNC)
    for i, (r, res) in enumerate(results):
        ax = axes2[i // ncols, i % ncols]
        ax.plot(sym_idx, res['sym_metrics'], color='steelblue', linewidth=0.8)
        ax.axvspan(0, 5, color='orange', alpha=0.25)
        ax.axhline(0.25, color='crimson', linestyle='--', linewidth=0.9, label='thr=0.25')
        ax.set_ylim(0.0, 0.85)
        ax.set_title(f"#{i+1} slot={r['slot']} f5max={res['first_5_max']:.2f} gt025={res['gt025_all']}/256", fontsize=9)
        if i == 0:
            ax.legend(fontsize=7, loc='upper right')
        if i // ncols == nrows - 1:
            ax.set_xlabel('SYNC symbol index (0..255)')
        if i % ncols == 0:
            ax.set_ylabel('Norm corr |dot|^2/E')
    fig2.suptitle(f'{n_cases} TIMING_FAILED Windows: Per-Symbol SYNC[0..255] Matched Filter Metric (orange = m=0..4 ROI)', fontsize=13)
    fig2.tight_layout()
    p2 = args.out_dir / 'timing_failed_sync_metrics.png'
    fig2.savefig(p2, dpi=140)
    plt.close(fig2)

    # Plot 3: 256-symbol accumulated CIR across lag in [-256, +256]
    fig3, axes3 = plt.subplots(nrows, ncols, figsize=(18, fig_h), sharex=True, sharey=True, squeeze=False)
    for i, (r, res) in enumerate(results):
        ax = axes3[i // ncols, i % ncols]
        ax.plot(res['lags'], res['cir_256'], color='navy', linewidth=0.9)
        ax.axvline(res['best_lag'], color='crimson', linestyle='--', linewidth=0.8)
        ax.set_ylim(0.0, 0.55)
        ax.set_title(f"#{i+1} slot={r['slot']} lag={res['best_lag']:+d} pk={res['cir_peak']:.3f} PAR={res['cir_par']:.1f}", fontsize=9)
        if i // ncols == nrows - 1:
            ax.set_xlabel('Lag from pred_start (work samples)')
        if i % ncols == 0:
            ax.set_ylabel('256-sym mean |dot|^2/E')
    fig3.suptitle(f'{n_cases} TIMING_FAILED Windows: 256-Symbol Accumulated CIR Matched Filter (lag +-256)', fontsize=13)
    fig3.tight_layout()
    p3 = args.out_dir / 'timing_failed_cir256.png'
    fig3.savefig(p3, dpi=140)
    plt.close(fig3)

    # Plot 4: Continuous 1D matched filter output across all 256 preamble symbols (one long line)
    fig4, axes4 = plt.subplots(nrows, ncols, figsize=(18, fig_h), sharex=True, sharey=True, squeeze=False)
    for i, (r, res) in enumerate(results):
        ax = axes4[i // ncols, i % ncols]
        ax.plot(res['cont_x_sym'], res['cont_metric'], color='steelblue', linewidth=0.35)
        ax.axvspan(0, 5, color='orange', alpha=0.25, label='m=0..4 ROI')
        ax.axhline(0.25, color='crimson', linestyle='--', linewidth=0.8, label='thr=0.25')
        ax.set_xlim(-0.5, N_SYNC + 0.5)
        ax.set_ylim(0.0, 0.55)
        ax.set_title(f"#{i+1} slot={r['slot']} f5max={res['first_5_max']:.2f} gt025={res['gt025_all']}/256", fontsize=9)
        if i == 0:
            ax.legend(fontsize=7, loc='upper right')
        if i // ncols == nrows - 1:
            ax.set_xlabel('Preamble time (symbol index 0..256)')
        if i % ncols == 0:
            ax.set_ylabel('Norm corr |dot|^2/E')
    fig4.suptitle(f'{n_cases} TIMING_FAILED Windows: Continuous 1D Matched Filter Output Across All 256 Preamble Symbols', fontsize=13)
    fig4.tight_layout()
    p4 = args.out_dir / 'timing_failed_cir_longline.png'
    fig4.savefig(p4, dpi=150)
    plt.close(fig4)

    # Save CSV
    csv_path = args.out_dir / 'summary.csv'
    with csv_path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['idx', 'slot', 'id', 'time_ms', 'best_lag', 'cir_peak', 'cir_par',
                    'first_5_max', 'gt025_all', 'med_front_0_15', 'med_tail_32_255'])
        for i, (r, res) in enumerate(results):
            w.writerow([
                i + 1, r['slot'], r['id'], f"{r['time_ms']:.3f}", res['best_lag'],
                f"{res['cir_peak']:.4f}", f"{res['cir_par']:.2f}",
                f"{res['first_5_max']:.4f}", res['gt025_all'],
                f"{res['med_front']:.4f}", f"{res['med_tail']:.4f}",
            ])
    print(f"\nSaved plots and CSV to {args.out_dir}")


if __name__ == '__main__':
    main()
