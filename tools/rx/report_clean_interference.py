#!/usr/bin/env python3
"""Summarize and plot analyze_clean_interference.py output; no RF calibration."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

BANDS = [(-183, -173), (-118, -108), (-105, -86), (-44, -27), (-9, 1)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    results = []
    fig, axes = plt.subplots(2, 1, figsize=(13, 9), constrained_layout=True)
    waterfall, wax = plt.subplots(4, 1, figsize=(13, 12), constrained_layout=True)
    timeline, tax = plt.subplots(7, 1, figsize=(13, 14), sharex=True, constrained_layout=True)
    water_index = 0
    for ix, path in enumerate(sorted(a.input.glob('*.npz'))):
        z = np.load(path)
        r = json.loads(path.with_suffix('.json').read_text())
        f, p, t = z['frequency_hz'] / 1e6, z['psd'], z['times_s']
        df = 22500
        avg = p.mean(axis=0)
        floor = r['thermal_psd_lsb2_hz']
        snr = json.loads((Path(r['folder']) / 'rx_2s_r01.snr.json').read_text())
        sig = 10 ** (snr['signal_db'] / 10)
        distance = float(path.stem.split('_')[0][1:-1].replace('p', '.'))
        r.update(distance_m=distance, signal_dbfs=float(10*np.log10(sig/32768**2)),
                 prior_thermal_snr_db=snr['snr_db'])
        r['bands'] = []
        for lo, hi in BANDS:
            mask = (f >= lo) & (f < hi)
            power = p[:, mask].sum(axis=1) * df
            noise = floor * mask.sum() * df
            excess = max(float(power.mean()) - noise, 1e-12)
            b = dict(baseband_mhz=[lo, hi], rf_mhz=[6489.6+lo, 6489.6+hi],
                     excess_dbfs=float(10*np.log10(excess/32768**2)),
                     interference_over_signal_db=float(10*np.log10(excess/sig)),
                     mean_total_over_floor_db=float(10*np.log10(power.mean()/noise)),
                     total_over_floor_percentiles_db=dict(zip(['p50', 'p95', 'p99', 'max'],
                         (10*np.log10(np.percentile(power, [50, 95, 99, 100])/noise)).tolist())),
                     sampled_fraction_above_floor_6db=float(np.mean(power/noise > 10**.6)))
            r['bands'].append(b)
        r['lines'] = []
        for center in [-58.88, 94.72, 12.8, 115.2, -263.68, 258.57]:
            mask = abs(f-center) < .15
            local = np.median(avg[(abs(f-center) > .3) & (abs(f-center) < 1)])
            excess = max(float(avg[mask].sum()-local*mask.sum())*df, 1e-12)
            r['lines'].append(dict(baseband_mhz=center, rf_mhz=6489.6+center,
                                   excess_dbfs=float(10*np.log10(excess/32768**2))))
        mask = abs(f) <= 250
        power = p[:, mask].sum(axis=1)*df
        r['effective_sinr_db'] = float(10*np.log10(sig/power.mean()))
        r['median_window_sinr_db'] = float(10*np.log10(sig/np.median(power)))
        r['total_interference_excess_dbfs'] = float(10*np.log10(max(power.mean()-floor*mask.sum()*df, 1e-12)/32768**2))
        # Spectra at 1 MHz resolution avoid comparing arbitrary FFT-bin peak heights.
        edges = np.arange(-350, 351)
        coarse = np.array([p[:, (f>=lo) & (f<lo+1)].mean(axis=1) for lo in edges[:-1]]).T
        axes[0].plot(edges[:-1]+.5, 10*np.log10(coarse.mean(0)/32768**2)+60,
                     label=f'{distance:g} m', linewidth=1)
        axes[1].plot(edges[:-1]+.5+6489.6, 10*np.log10(coarse.mean(0)/floor),
                     label=f'{distance:g} m', linewidth=1)
        if distance in [.8, 2.5, 4.2, 5.2]:
            ax = wax[water_index]
            im = ax.pcolormesh(edges+6489.6, np.r_[t, t[-1]+.004],
                              10*np.log10(coarse/floor), shading='flat',
                              cmap='magma', vmin=0, vmax=25, rasterized=True)
            ax.set(ylabel='Time (s)', title=f'{distance:g} m: off-packet spectrum / median floor',
                   xlim=(6239.6, 6739.6))
            waterfall.colorbar(im, ax=ax, label='dB above floor')
            water_index += 1
        for lo, hi in [(-183, -173), (-105, -86), (-44, -27)]:
            mask = (f>=lo) & (f<hi)
            power = p[:, mask].sum(1)*df
            tax[ix].plot(t, 10*np.log10(power/(floor*mask.sum()*df)),
                         label=f'{lo} to {hi} MHz', linewidth=.8)
        tax[ix].set(ylabel=f'{distance:g} m\n(dB)', ylim=(-2, 37))
        tax[ix].grid(alpha=.2)
        results.append(r)
    axes[0].set(xlabel='Baseband frequency (MHz)', ylabel='PSD (dBFS/MHz)',
                title='Packet-free spectra, fixed -232.96 MHz tone removed; 1 MHz averages')
    axes[1].set(xlabel='RF frequency (MHz)', ylabel='Mean PSD / median floor (dB)',
                xlim=(6239.6, 6739.6))
    for ax in axes:
        ax.legend(ncol=7, fontsize=8)
        ax.grid(alpha=.2)
    wax[-1].set_xlabel('RF frequency (MHz)')
    tax[0].legend(ncol=3, fontsize=9)
    tax[0].set_title('Off-packet band power / same-band median thermal floor; 4 ms sample spacing')
    tax[-1].set_xlabel('Time within capture (s)')
    fig.savefig(a.out/'spectra.png', dpi=160)
    waterfall.savefig(a.out/'waterfalls.png', dpi=160)
    timeline.savefig(a.out/'band_time.png', dpi=160)
    (a.out/'metrics.json').write_text(json.dumps(results, indent=2))
    for r in results:
        print(r['distance_m'], 'SINR', round(r['effective_sinr_db'],2),
              '6311.6 MHz occupancy', round(100*r['bands'][0]['sampled_fraction_above_floor_6db'],1))


if __name__ == '__main__':
    main()
