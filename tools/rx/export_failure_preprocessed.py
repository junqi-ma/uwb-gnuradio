#!/usr/bin/env python3
"""Export the same 20 one-ms examples after tone cancellation and resampling."""
import argparse
import json
from pathlib import Path
import subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def envelope(axis, x, time, label):
    step = 100
    n = len(x)//step*step
    a = x[:n].reshape(-1, step)
    axis.fill_between(time[:n].reshape(-1, step).mean(1), a.min(1), a.max(1),
                      color='steelblue', alpha=.8)
    axis.set_ylabel(label)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--examples', type=Path, required=True)
    ap.add_argument('--binary', type=Path, default=Path('/tmp/uwb_window_diag/export_preprocessed'))
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[2]
    out = args.examples/'preprocessed'
    out.mkdir(exist_ok=True)
    rows = json.loads((args.examples/'manifest.json').read_text())
    index = ['<!doctype html><meta charset="utf-8"><title>单音消减和重采样后的波形</title>',
             '<style>body{font:16px sans-serif;max-width:1400px;margin:auto}img{width:100%}</style>',
             '<h1>20 个窗口：消单音及重采样结果</h1>',
             '<p>对应上一级原始 1 ms 切片，预期起点前 100 us 至后 900 us。橙色为预期包范围。</p>',
             '<p>SC16：消单音后，737.28 MS/s，int16 I/Q；CF32：再经 eq16 重采样，998.4 MS/s，complex64。</p>']
    results = []
    for r in rows:
        prefix = out/r['id']
        run = subprocess.run([str(args.binary), r['source'], str(r['file_start_sample']),
                              str(r['complex_samples']), str(prefix), str(root/'testdata')],
                             capture_output=True, text=True, check=True)
        counts = json.loads(run.stdout)
        native_path = out/(r['id']+'_tone_cancelled.sc16')
        work_path = out/(r['id']+'_tone_cancelled_resampled.cf32')
        raw = np.fromfile(args.examples/r['raw_file'], dtype='<i2').reshape(-1, 2)
        native = np.fromfile(native_path, dtype='<i2').reshape(-1, 2)
        work = np.fromfile(work_path, dtype='<c8')
        assert len(native) == len(raw) == counts['native_samples'] == 737280
        assert len(work) == counts['work_samples'] and np.isfinite(work).all()
        # Each selected window falls within one model epoch: verify subtraction
        # has exactly the production model's 288-sample periodicity.
        correction = raw.astype(np.int32)-native.astype(np.int32)
        assert np.array_equal(correction[288:], correction[:-288])
        n = np.arange(288)
        phasor = np.exp(2j*np.pi*(-91/288)*n)
        coefficients = []
        for a in [raw, native]:
            folded = (a[:, 0].astype(float)+1j*a[:, 1]).reshape(-1, 288).sum(0)
            coefficients.append(abs(np.vdot(phasor, folded))/len(a))
        suppression = float(20*np.log10(coefficients[0]/max(coefficients[1], 1e-30)))
        meta = dict(id=r['id'], kind=r['kind'], distance_m=r['distance_m'],
                    source=r['source'], source_start_sample=r['file_start_sample'],
                    native_rate_hz=737280000, work_rate_hz=998400000,
                    native_samples=len(native), work_samples=len(work),
                    expected_start_native_offset=73728,
                    expected_start_work_offset=99840+519/48,
                    resampler_group_delay_work_samples=519/48,
                    native_format='little-endian int16 I,Q interleaved, ADC LSB units',
                    work_format='little-endian complex64, real/imag float32; input scale 1/32768',
                    processing='production FixedToneCancelCore; -232.96 MHz; 20 ms / 0.5 ms / crossfade 256; eq16 65/48',
                    raw_tone_projection_lsb=coefficients[0], remaining_tone_projection_lsb=coefficients[1],
                    tone_projection_suppression_db=suppression,
                    native_file=native_path.name, work_file=work_path.name,
                    note='1 ms inspection window; resampler resets at this window start. DSP replay diagnostic windows have a different pre-guard.',
                    **{'filter_taps': counts['filter_taps']})
        (out/(r['id']+'.json')).write_text(json.dumps(meta, indent=2)+'\n')
        tn = (np.arange(len(native))-73728)/737.28
        tw = (np.arange(len(work))-99840-519/48)/998.4
        fig, ax = plt.subplots(5, 1, figsize=(14, 12), constrained_layout=True)
        for a, tt, label in [(raw[:, 0].astype(float)+1j*raw[:, 1], tn, 'Raw'),
                             (native[:, 0].astype(float)+1j*native[:, 1], tn, 'Tone cancelled'),
                             (work*32768, tw, 'Tone cancelled + resampled')]:
            step = 737 if label != 'Tone cancelled + resampled' else 998
            size = len(a)//step*step
            rms = np.sqrt((abs(a[:size])**2).reshape(-1, step).mean(1))
            ax[0].plot(tt[:size].reshape(-1, step).mean(1), rms, label=label, linewidth=.8)
        ax[0].set_ylabel('RMS (LSB scale)')
        ax[0].legend(fontsize=9)
        envelope(ax[1], native[:, 0], tn, 'Tone cancelled I (LSB)')
        envelope(ax[2], native[:, 1], tn, 'Tone cancelled Q (LSB)')
        envelope(ax[3], work.real*32768, tw, 'Resampled I x32768')
        envelope(ax[4], work.imag*32768, tw, 'Resampled Q x32768')
        for a in ax:
            a.axvspan(0, 329161/737.28, color='orange', alpha=.12)
            a.set(xlim=(-100, 900), xlabel='Time from interpolated packet start (us)')
            a.grid(alpha=.2)
        fig.suptitle(f'{r["id"]} | {r["distance_m"]:g} m | fixed-tone projection suppression {suppression:.1f} dB\n'
                     '737.28 MS/s tone cancellation -> eq16 65/48 resampling -> 998.4 MS/s', fontsize=12)
        fig.savefig(out/(r['id']+'.png'), dpi=145)
        plt.close(fig)
        index.append(f'<h2>{r["id"]} · {r["distance_m"]} m</h2><p>'
                     f'<a href="{native_path.name}">消单音 SC16</a> · '
                     f'<a href="{work_path.name}">消单音并重采样 CF32</a> · '
                     f'<a href="{r["id"]}.json">采样参数</a></p>'
                     f'<img loading="lazy" src="{r["id"]}.png">')
        results.append(meta)
        print(r['id'], counts, round(suppression, 1), flush=True)
    (out/'manifest.json').write_text(json.dumps(results, indent=2)+'\n')
    (out/'index.html').write_text('\n'.join(index))
    (out/'README.md').write_text('''# 消单音和重采样后的 20 个窗口

打开 index.html 浏览波形，每类各 5 个。所有数据与上一级原始 1 ms 切片一一对应。

- *_tone_cancelled.sc16：737.28 MS/s，小端 int16 I,Q 交错，保持 ADC LSB 单位。
- *_tone_cancelled_resampled.cf32：998.4 MS/s，小端 complex64（float32 I,Q），
  重采样输入乘 1/32768，eq16 系数 1039 taps。含重采样 flush 尾部。
- *.png：原始/消单音/重采样 RMS 对照和处理后的 I/Q 波形。
- *.json / manifest.json：源位置、样点数、格式、群时延与单音投影抑制量。

处理顺序：原始 SC16 → 固定 −232.96 MHz 单音消减 → eq16 65/48 重采样。
使用生产 C++ 内核和原整段录波的对应训练 epoch（每 20 ms 的前 0.5 ms），
不以切片自身重新训练，保留 256 样点交叉渐变语义。
没有额外带阻、CFO 补偿、归一化或解调后处理；其他宽带/突发干扰仍保留。

窗口是预期包起点前 100 us 至后 900 us。图上已扣除重采样滤波群时延 519/48 工作样点。
为与旧原始波形直接比较，本次从 1 ms 观察窗起点重置重采样器；此前精确解调回放
使用约 0.5076 ms 的实际 DSP 窗，两者起点和边界状态不同，不应逐字节比较 CF32。
原始文件及上一级 SC16 切片均未修改。

Python 读取：
```python
import numpy as np
native = np.fromfile('no_window_01_tone_cancelled.sc16', dtype='<i2').reshape(-1, 2)
work = np.fromfile('no_window_01_tone_cancelled_resampled.cf32', dtype='<c8')
```
''')
    idx = args.examples/'index.html'
    text = idx.read_text()
    link = '<p><a href="preprocessed/index.html">查看消单音和重采样后的 20 个窗口</a></p>'
    if link not in text:
        idx.write_text(text.replace('<h1>', link+'<h1>', 1))


if __name__ == '__main__':
    main()
