#!/usr/bin/env python3
"""Extract twenty raw SC16 examples with locally bracketed packet timing."""
import argparse
import collections
import csv
import html
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

FS = 737280000
PERIOD = 737280
PACKET = 329161
KINDS = ['NO_WINDOW', 'TIMING_FAILED', 'CFO_FAILED', 'SFD_FAILED']
ORDER = ['INCOMPLETE_WINDOW', 'TIMING_FAILED', 'CFO_FAILED', 'SFD_FAILED',
         'CIR_FAILED', 'PHR_FAILED', 'PAYLOAD_FAILED', 'FCS_FAILED', 'OK']


def candidates(root):
    found = collections.defaultdict(list)
    for folder in sorted(root.glob('d*m_period1ms')):
        source = folder/'rx_2s_r01.sc16'
        lines = source.with_suffix('.demod.log').read_text().splitlines()
        rows = []
        for line in lines:
            if line.startswith('PACKET '):
                row = dict(re.findall(r'(\w+)=([^ ]+)', line))
                row['log_line'] = line
                rows.append(row)
        good = [r for r in rows if r['status'] == 'OK' and r.get('crc') == '1']
        if not good:
            continue
        phase = float(good[0]['time_ms']) % 1
        by = collections.defaultdict(list)
        for row in rows:
            by[round(float(row['time_ms'])-phase)].append(row)
        good_by = {k: next(r for r in rs if r['status'] == 'OK')
                   for k, rs in by.items() if any(r['status'] == 'OK' for r in rs)}
        for k in range(5, 1995):
            rs = by[k]
            kind = max((r['status'] for r in rs), key=ORDER.index, default='NO_WINDOW')
            if kind not in KINDS:
                continue
            before = sorted(j for j in good_by if k-10 <= j < k)
            after = sorted(j for j in good_by if k < j <= k+10)
            if len(before) < 3 or len(after) < 3:
                continue
            anchors = before[-3:] + after[:3]
            x = np.array(anchors)-k
            y = np.array([int(good_by[j]['start']) for j in anchors])
            slope, intercept = np.polyfit(x, y, 1)
            residual = float(max(abs(slope*x+intercept-y)))
            if residual > 2 or abs(slope-PERIOD) > 10:
                continue
            target = [r for r in rs if r['status'] == kind and r.get('path') == 'target'
                      and abs(int(r['pred'])-intercept) <= 32]
            if kind != 'NO_WINDOW' and not target:
                continue
            row = min(target, key=lambda r: abs(int(r['pred'])-intercept)) if target else None
            found[kind].append(dict(kind=kind, source=str(source),
                distance_m=float(folder.name.split('_')[0][1:-1].replace('p', '.')),
                nominal_slot=k, predicted_start_sample=round(float(intercept)),
                fitted_period_samples=float(slope), anchor_max_residual_samples=residual,
                nearest_good_before_ms=k-before[-1], nearest_good_after_ms=after[0]-k,
                anchors=[dict(slot=j, start_sample=int(good_by[j]['start']),
                              log_line=good_by[j]['log_line']) for j in anchors],
                selected_attempt=row, all_attempts=rs,
                actual_prediction_error_samples=(int(row['pred'])-float(intercept)) if row else None))
    selected = []
    for kind in KINDS:
        pool = sorted(found[kind], key=lambda r: (
            r['nearest_good_before_ms']+r['nearest_good_after_ms'],
            r['anchor_max_residual_samples']))
        chosen = []
        # Prefer different recordings and separate interference episodes.
        while pool and len(chosen) < 5:
            pool.sort(key=lambda r: (
                sum(c['source'] == r['source'] for c in chosen),
                any(c['source'] == r['source'] and abs(c['nominal_slot']-r['nominal_slot']) < 20 for c in chosen),
                r['nearest_good_before_ms']+r['nearest_good_after_ms'],
                r['anchor_max_residual_samples']))
            chosen.append(pool.pop(0))
        assert len(chosen) == 5, (kind, len(chosen))
        selected.extend(chosen)
    return selected


def plot_example(raw, info, path):
    x = raw.astype(np.float32)
    t = (np.arange(len(raw))-info['start_offset_samples'])/FS*1e6
    amp = np.hypot(x[:, 0], x[:, 1])
    fig, ax = plt.subplots(5, 1, figsize=(14, 12), constrained_layout=True)
    # Min/max envelopes preserve excursions; no aliased point decimation.
    step = 74  # ~100 ns
    n = len(raw)//step*step
    tb = t[:n].reshape(-1, step).mean(1)
    for axis, values, label in [(ax[0], x[:, 0], 'Raw I (LSB)'),
                                (ax[1], x[:, 1], 'Raw Q (LSB)'),
                                (ax[2], amp, 'Raw |IQ| (LSB)')]:
        blocks = values[:n].reshape(-1, step)
        axis.fill_between(tb, blocks.min(1), blocks.max(1), color='steelblue', alpha=.7)
        axis.set_ylabel(label)
    step = 737  # ~1 us; documented actual width
    n = len(raw)//step*step
    rms = np.sqrt((amp[:n]**2).reshape(-1, step).mean(1))
    ax[3].plot(t[:n].reshape(-1, step).mean(1), rms, linewidth=.8)
    ax[3].set_ylabel('Raw RMS (LSB)\n0.9996 us bins')
    for axis in ax[:4]:
        axis.axvspan(0, PACKET/FS*1e6, color='orange', alpha=.12)
        axis.axvline(0, color='darkorange', linestyle='--', linewidth=.8)
        axis.axvline(PACKET/FS*1e6, color='darkorange', linestyle='--', linewidth=.8)
        axis.set(xlim=(-100, 900), xlabel='Time relative to interpolated packet start (us)')
        axis.grid(alpha=.2)
    # All native samples, 50 ns at 20 us inside predicted preamble.
    lo = info['start_offset_samples']+round(20e-6*FS)
    hi = lo+37
    tz = np.arange(hi-lo)/FS*1e9
    ax[4].plot(tz, x[lo:hi, 0], '.-', label='I', linewidth=.8)
    ax[4].plot(tz, x[lo:hi, 1], '.-', label='Q', linewidth=.8)
    ax[4].set(xlabel='Native samples: ns after predicted start + 20 us', ylabel='Raw IQ (LSB)')
    ax[4].legend()
    ax[4].grid(alpha=.2)
    row = info['selected_attempt'] or {}
    title = (f"{info['id']} | {info['kind']} | {info['distance_m']:g} m | "
             f"absolute {info['predicted_start_sample']/FS*1000:.6f} ms\n"
             f"6 CRC-good timing anchors; max residual {info['anchor_max_residual_samples']:.3f} samples; "
             f"SYNC={row.get('sync', 'N/A')}, fit={row.get('fit', 'N/A')}; "
             'RAW: no tone cancellation / filtering')
    fig.suptitle(title, fontsize=11)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    selected = candidates(args.root)
    args.out.mkdir(parents=True, exist_ok=True)
    numbers = collections.Counter()
    for info in selected:
        numbers[info['kind']] += 1
        name = f"{info['kind'].lower()}_{numbers[info['kind']]:02d}"
        info['id'] = name
        info['sample_rate_hz'] = FS
        info['start_offset_samples'] = 73728
        info['file_start_sample'] = info['predicted_start_sample']-73728
        info['complex_samples'] = PERIOD
        info['format'] = 'little-endian interleaved int16 I,Q; original unmodified SC16'
        info['raw_file'] = name+'.sc16'
        info['plot_file'] = name+'.png'
        meta = json.loads(Path(info['source']+'.json').read_text())
        assert meta['rate'] == FS and not meta['errors'] and meta['got'] == meta['requested']
        a = np.memmap(info['source'], dtype='<i2', mode='r').reshape(-1, 2)
        start = info['file_start_sample']
        raw = np.array(a[start:start+PERIOD])
        assert raw.shape == (PERIOD, 2)
        raw.tofile(args.out/info['raw_file'])
        # Verify the exported snippet exactly matches the original samples.
        assert np.array_equal(np.fromfile(args.out/info['raw_file'], dtype='<i2').reshape(-1, 2), raw)
        info['raw_rms_lsb'] = float(np.sqrt(np.mean(raw.astype(np.float64)**2)*2))
        info['peak_abs_component_lsb'] = int(np.max(abs(raw.astype(np.int32))))
        info['clipped_components'] = int(np.count_nonzero((raw == -32768) | (raw == 32767)))
        (args.out/(name+'.json')).write_text(json.dumps(info, indent=2)+'\n')
        plot_example(raw, info, args.out/info['plot_file'])
        print(name, info['distance_m'], info['nominal_slot'], flush=True)
    (args.out/'manifest.json').write_text(json.dumps(selected, indent=2)+'\n')
    keys = ['id', 'kind', 'distance_m', 'nominal_slot', 'predicted_start_sample',
            'anchor_max_residual_samples', 'actual_prediction_error_samples',
            'nearest_good_before_ms', 'nearest_good_after_ms', 'raw_rms_lsb',
            'peak_abs_component_lsb', 'clipped_components', 'source']
    with (args.out/'summary.csv').open('w') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(selected)
    content = ['<!doctype html><meta charset="utf-8"><title>失败窗口原始波形</title>',
               '<style>body{font:16px sans-serif;max-width:1400px;margin:auto}img{width:100%}</style>',
               '<h1>20 个失败时隙：原始 SC16 波形</h1>',
               '<p>每类 5 个。橙色范围是由相邻成功包插值得到的预期包区间。未滤波、未消单音。</p>',
               '<p>时间位置由前后各 3 个 CRC 成功包核对，不代表独立确认该时隙存在发射包。</p>']
    for r in selected:
        content.append(f'<h2>{html.escape(r["id"])} · {r["distance_m"]} m · slot {r["nominal_slot"]}</h2>'
                       f'<p><a href="{r["raw_file"]}">原始 SC16</a> · '
                       f'<a href="{r["id"]}.json">位置及日志证据</a></p>'
                       f'<img loading="lazy" src="{r["plot_file"]}">')
    (args.out/'index.html').write_text('\n'.join(content))
    (args.out/'README.md').write_text('''# 失败窗口的原始波形

打开 index.html 浏览全部 20 张图；每类 5 个：无窗口、同步失败、CFO 失败、SFD 失败。
每个例子包含 PNG、未经修改的 1 ms SC16 切片和 JSON 位置/日志证据。
原始文件均保留，提取过程不改写源文件。

窗口为插值包起点前 100 us 至后 900 us，采样率 737.28 MS/s，
每个 SC16 含 737280 个复样点（2949120 字节），小端 int16 I,Q 交错。
橙色区间从预期起点到 +446.452 us（329161 原生样点）。
每个点的位置由前后各 3 个 CRC 成功包起点做局部直线拟合，锚点均在 ±10 ms，
拟合最大残差要求不超过 2 原生样点；这不是严格的插值误差上界。
有解调日志的例子还要求 path=target，实际预测起点偏差不超过 32 原生样点。
这排除了明显的窗口错位；不能独立证明失败时隙确实发包或推断物理失败根因。
无窗口例子是按此时间模型补截的原始数据，不存在对应解调尝试。
部分无窗口例子可能来自同一次失锁事件，不能当作独立重复实验。

图前 3 行分别为原始 I、Q、幅度，使用 74 样点（约 100 ns）分箱 min/max
显示包络，避免直接抽点导致混叠；第 4 行为 737 样点（约 1 us）RMS；
最后一行为预期起点 +20 us 处连续 37 个原生样点。未做单音消除、滤波或归一化。
原始波形可能被固定单音主导，不能仅凭包络判断同步/SFD 失败根因。
summary.csv 和 manifest.json 提供索引，各例 JSON 保留锚点与实际失败日志。
''')


if __name__ == '__main__':
    main()
