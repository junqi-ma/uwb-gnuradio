#!/usr/bin/env python3
"""Plot exact-replay preamble diagnostics and compare adjacent CRC-good controls."""
import argparse
import csv
import json
from pathlib import Path
import warnings
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import spectrogram, welch

FS = 998400000
BANDS = [(-183, -173), (-118, -108), (-105, -86), (-59.38, -58.38), (-44, -27)]


def load_csv(path):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore')
        return np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True))


def powers(y, grid):
    n = np.arange(1016)
    starts = grid['position'].astype(int)-1015
    block = y[starts[:, None]+n]
    return (abs(block)**2).mean(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--examples', type=Path, required=True)
    args = ap.parse_args()
    out = args.examples/'analysis'
    examples = {a['id']: a for a in json.loads((args.examples/'manifest.json').read_text())}
    replays = json.loads((out/'replay_results.json').read_text())
    summaries = []
    index = ['<!doctype html><meta charset="utf-8"><title>失败窗口诊断</title>',
             '<style>body{font:16px sans-serif;max-width:1400px;margin:auto}img{width:100%}</style>',
             '<h1>失败窗口诊断及相邻成功包对照</h1>',
             '<p>这里的信号经过与接收器相同的单音消除和 eq16 重采样。原始波形在上一级文件夹。</p>',
             '<p>逐符号诊断探测包括原跟踪器首次中断后的符号；不以这些探测结果直接宣告解码成功。</p>']
    for r in replays:
        name = r['id']
        base, control = r['runs']['baseline'], r['runs']['good_control']
        y = np.fromfile(base['prefix']+'.cf32', dtype=np.complex64)
        good_y = np.fromfile(control['prefix']+'.cf32', dtype=np.complex64)
        grid = load_csv(base['prefix']+'.grid.csv')
        good_grid = load_csv(control['prefix']+'.grid.csv')
        local = load_csv(base['prefix']+'.local.csv')
        sfd = load_csv(base['prefix']+'.sfd.csv')
        nsfd = load_csv(base['prefix']+'.nsfd.csv')
        total, good_total = powers(y, grid), powers(good_y, good_grid)
        coherent = grid['corr_re']**2+grid['corr_im']**2
        good_coherent = good_grid['corr_re']**2+good_grid['corr_im']**2
        summary = dict(id=name, kind=r['kind'], baseline=base['status_name'],
                       first_five_metrics=grid['metric'][:5].tolist(),
                       symbols_below_025=int(np.sum(grid['metric'] < .25)),
                       initial_sync=base['initial_sync'], initial_period=base['initial_period'],
                       passing_modes=[k for k, v in r['runs'].items() if v['fcs'] and k != 'good_control'],
                       regions={})
        for region, sl in [('first5', slice(0, 5)), ('preamble', slice(None)), ('tail40', slice(-40, None))]:
            summary['regions'][region] = dict(
                total_power_vs_control_db=float(10*np.log10(np.median(total[sl])/np.median(good_total[sl]))),
                matched_projection_vs_control_db=float(10*np.log10(np.median(coherent[sl])/np.median(good_coherent[sl]))))
        summary['sfd_calls'] = [dict(call=int(call), max_metric=float(max(sfd['metric'][sfd['call'] == call])),
                                     expected=int(sfd['expected'][sfd['call'] == call][0]))
                                for call in np.unique(sfd['call'])] if len(sfd) else []
        summary['nsfd_calls'] = [dict(metric=float(a['metric']), position=int(a['position'])) for a in nsfd]
        # Compare band power in the first five SYNCs, where timing admission occurs.
        start, good_start = int(grid['position'][0])-1015, int(good_grid['position'][0])-1015
        f, p = welch(y[start:start+5*1016], fs=FS, nperseg=4096, noverlap=2048,
                     return_onesided=False, detrend=False)
        _, gp = welch(good_y[good_start:good_start+5*1016], fs=FS, nperseg=4096, noverlap=2048,
                      return_onesided=False, detrend=False)
        summary['first5_band_power_vs_control_db'] = {
            f'{lo}:{hi}': float(10*np.log10(p[(f/1e6 >= lo) & (f/1e6 < hi)].sum()/
                                         gp[(f/1e6 >= lo) & (f/1e6 < hi)].sum())) for lo, hi in BANDS}
        sfd_a, sfd_b = start+256*1016, good_start+256*1016
        sa, sb = y[sfd_a:sfd_a+8128], good_y[sfd_b:sfd_b+8128]
        f, sp = welch(sa, fs=FS, nperseg=4096, noverlap=2048,
                      return_onesided=False, detrend=False)
        _, sgp = welch(sb, fs=FS, nperseg=4096, noverlap=2048,
                       return_onesided=False, detrend=False)
        summary['sfd_total_power_vs_control_db'] = float(10*np.log10(np.mean(abs(sa)**2)/np.mean(abs(sb)**2)))
        summary['sfd_band_power_vs_control_db'] = {
            f'{lo}:{hi}': float(10*np.log10(sp[(f/1e6 >= lo) & (f/1e6 < hi)].sum()/
                                         sgp[(f/1e6 >= lo) & (f/1e6 < hi)].sum())) for lo, hi in BANDS}
        fig, axes = plt.subplots(4, 1, figsize=(14, 12), constrained_layout=True)
        # Nominal filter delay 519/48 work samples; keep all relative axes consistent.
        ref = round((base['seed']-base['window_start'])*65/48)+519/48
        gref = round((control['seed']-control['window_start'])*65/48)+519/48
        for sig, origin, label in [(y, ref, 'Selected window'), (good_y, gref, 'Adjacent CRC-good control')]:
            n = len(sig)//998*998
            rms = np.sqrt((abs(sig[:n])**2).reshape(-1, 998).mean(1))*32768
            t = (np.arange(len(rms))*998+498.5-origin)/FS*1e6
            axes[0].plot(t, rms, label=label, linewidth=.85)
        axes[0].set(xlim=(-40, 450), ylabel='Work IQ RMS x32768', xlabel='Time from expected packet start (us)')
        axes[0].legend(fontsize=9)
        axes[1].plot(grid['symbol'], grid['metric'], '.-', markersize=2, linewidth=.7, label='Selected window: probe all SYNCs')
        axes[1].plot(good_grid['symbol'], good_grid['metric'], linewidth=.7, alpha=.7, label='Adjacent CRC-good control')
        axes[1].axhline(.25, color='red', linestyle='--', label='SYNC threshold 0.25')
        axes[1].axhline(.20, color='orange', linestyle=':', label='Tail threshold 0.20')
        axes[1].set(xlabel='Known SYNC symbol index', ylabel='Normalized correlation', xlim=(0, 255))
        axes[1].legend(fontsize=8, ncol=2)
        f, t, ps = spectrogram(y, fs=FS, nperseg=4096, noverlap=3072,
                               return_onesided=False, detrend=False, mode='psd')
        f, ps = np.fft.fftshift(f), np.fft.fftshift(ps, axes=0)
        mask = abs(f) <= 250e6
        image = axes[2].pcolormesh((t-ref/FS)*1e6, f[mask]/1e6+6489.6,
                                   10*np.log10(np.maximum(ps[mask], 1e-30))+60,
                                   shading='auto', vmin=-85, vmax=-40, cmap='magma', rasterized=True)
        axes[2].set(xlim=(-40, 450), xlabel='Time from expected packet start (us)', ylabel='RF frequency (MHz)')
        fig.colorbar(image, ax=axes[2], label='Work PSD (dBFS/MHz)')
        axes[3].plot(grid['symbol'], 10*np.log10(total/good_total), label='Total symbol power / control')
        axes[3].plot(grid['symbol'], 10*np.log10(coherent/good_coherent), label='Template projection power / control')
        axes[3].axhline(0, color='gray', linewidth=.5)
        axes[3].set(xlabel='Known SYNC symbol index', ylabel='Power ratio (dB)', xlim=(0, 255))
        axes[3].legend(fontsize=9)
        for axis in [axes[0], axes[1], axes[3]]:
            axis.grid(alpha=.2)
        fig.suptitle(f'{name} | replay: {base["status_name"]} | initial SYNC={base["initial_sync"]}\n'
                     'Exact tone-epoch cancellation + eq16 resampling; control is a nearby CRC-good packet', fontsize=12)
        fig.savefig(out/(name+'_diagnostic.png'), dpi=145)
        plt.close(fig)
        index.append(f'<h2>{name}</h2><p>默认补截/回放：{base["status_name"]}；'
                     f'通过 CRC 的诊断对照：{", ".join(summary["passing_modes"]) or "无"}</p>'
                     f'<img loading="lazy" src="{name}_diagnostic.png">')
        summaries.append(summary)
        print(name, summary['first5_band_power_vs_control_db'], flush=True)
    (out/'window_metrics.json').write_text(json.dumps(summaries, indent=2)+'\n')
    (out/'index.html').write_text('\n'.join(index))
    original = args.examples/'index.html'
    s = original.read_text()
    link = '<p><a href="analysis/index.html">查看逐窗口诊断及相邻成功包对照</a></p>'
    if link not in s:
        original.write_text(s.replace('<h1>', link+'<h1>', 1))


if __name__ == '__main__':
    main()
