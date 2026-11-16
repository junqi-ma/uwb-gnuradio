#!/usr/bin/env python3
"""Plot raw and tone-cancelled native SC16 side by side with shared scales."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--examples', type=Path, required=True)
    args = ap.parse_args()
    processed = args.examples/'preprocessed'
    out = processed/'comparison'
    out.mkdir(exist_ok=True)
    rows = json.loads((processed/'manifest.json').read_text())
    index = ['<!doctype html><meta charset="utf-8"><title>单音消减前后对照</title>',
             '<style>body{font:16px sans-serif;max-width:1600px;margin:auto}img{width:100%}</style>',
             '<h1>20 个窗口的单音消减前后对照</h1>',
             '<p>左：原始；右：消减固定单音后。两侧均为 737.28 MS/s 原生数据，每行共享纵轴，时间对齐。</p>',
             '<p>依次为 I、Q、幅度和约 1 us RMS。橙色标注预期包范围；I/Q/幅度以分箱最小/最大值显示。</p>']
    for row in rows:
        name = row['id']
        raw = np.fromfile(args.examples/(name+'.sc16'), dtype='<i2').reshape(-1, 2)
        cancelled = np.fromfile(processed/row['native_file'], dtype='<i2').reshape(-1, 2)
        assert raw.shape == cancelled.shape == (737280, 2)
        time = (np.arange(len(raw))-row['expected_start_native_offset'])/737.28
        fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex=True, sharey='row', constrained_layout=True)
        for col, (data, color) in enumerate([(raw, 'steelblue'), (cancelled, 'seagreen')]):
            data = data.astype(np.float32)
            magnitude = np.hypot(data[:, 0], data[:, 1])
            step = 74
            n = len(data)//step*step
            t = time[:n].reshape(-1, step).mean(1)
            for rr, values in enumerate([data[:, 0], data[:, 1], magnitude]):
                blocks = values[:n].reshape(-1, step)
                axes[rr, col].fill_between(t, blocks.min(1), blocks.max(1), color=color, alpha=.8)
            step = 737
            n = len(data)//step*step
            rms = np.sqrt((magnitude[:n]**2).reshape(-1, step).mean(1))
            axes[3, col].plot(time[:n].reshape(-1, step).mean(1), rms, color=color, linewidth=.8)
        axes[0, 0].set_title('BEFORE: raw SC16', fontsize=13)
        axes[0, 1].set_title('AFTER: fixed-tone cancellation', fontsize=13)
        for rr, label in enumerate(['I (LSB)', 'Q (LSB)', '|IQ| (LSB)', 'RMS (LSB), ~1 us']):
            axes[rr, 0].set_ylabel(label)
            if rr < 2:
                limit = max(abs(v) for ax in axes[rr] for v in ax.get_ylim())
                axes[rr, 0].set_ylim(-limit, limit)
            else:
                limit = max(ax.get_ylim()[1] for ax in axes[rr])
                axes[rr, 0].set_ylim(0, limit)
        for ax in axes.flat:
            ax.axvspan(0, 329161/737.28, color='orange', alpha=.12)
            ax.axvline(0, color='darkorange', linestyle='--', linewidth=.6)
            ax.axvline(329161/737.28, color='darkorange', linestyle='--', linewidth=.6)
            ax.set_xlim(-100, 900)
            ax.grid(alpha=.2)
        for ax in axes[-1]:
            ax.set_xlabel('Time from expected packet start (us)')
        fig.suptitle(f'{name} | {row["distance_m"]:g} m | Before / after -232.96 MHz tone cancellation\n'
                     'Same native samples, same time axis, same vertical scale in each row', fontsize=12)
        filename = name+'_before_after.png'
        fig.savefig(out/filename, dpi=145)
        plt.close(fig)
        index.append(f'<h2>{name} · {row["distance_m"]} m</h2><img loading="lazy" src="{filename}">')
        print(name, flush=True)
    (out/'index.html').write_text('\n'.join(index))
    link = '<p><a href="comparison/index.html">查看单音消减前后左右并排对照（统一坐标）</a></p>'
    idx = processed/'index.html'
    text = idx.read_text()
    if link not in text:
        idx.write_text(text.replace('<h1>', link+'<h1>', 1))


if __name__ == '__main__':
    main()
