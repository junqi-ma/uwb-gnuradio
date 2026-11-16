#!/usr/bin/env python3
"""Compare SC16 antenna recordings; all-record and folded packet-gap spectra.

Assumes 737.28 MS/s, 6489.6 MHz, 1 ms repeating UWB when present.
0 dBFS denotes complex power 32768**2, not calibrated antenna-port dBm.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import welch

FS, PERIOD, FFT, GAP = 737280000, 737280, 32768, 131072
BANDS = [(-183, -173), (-118, -108), (-105, -86), (-44, -27), (-9, 1)]


def analyze(path, out):
    meta = json.loads(Path(str(path)+'.json').read_text())
    assert meta['rate'] == FS and abs(meta['freq']-6489.6e6) < 1
    assert meta['got'] == meta['requested'] and not meta['errors']
    m = np.memmap(path, dtype=np.int16, mode='r').reshape(-1, 2)
    count = len(m)//PERIOD
    short = np.exp(2j*np.pi*(-91/288)*np.arange(288)).astype(np.complex64)
    lut = np.tile(short, PERIOD//288)

    def read(off, size):
        a = m[off:off+size].astype(np.float32)
        return a[:, 0]+1j*a[:, 1]

    profile = np.zeros(256)
    for k in range(0, count, 10):
        x = read(k*PERIOD, PERIOD)
        c = np.vdot(short, x.reshape(-1, 288).sum(0))/PERIOD
        x -= c*lut
        profile += (abs(x)**2).reshape(256, -1).mean(1)
    profile /= len(range(0, count, 10))
    span = round(329161/PERIOD*256)
    phase = max(range(256), key=lambda i: np.take(profile, np.arange(i, i+span), mode='wrap').sum())
    phase *= PERIOD//256
    off = (phase+329161+(PERIOD-329161-GAP)//2) % PERIOD
    gaplut = np.exp(2j*np.pi*(-91/288)*np.arange(GAP)).astype(np.complex64)
    tones, coarse, gapbands = [], [], []
    rawpow, peak, clips = 0., 0, 0
    allsum, gapsum = np.zeros(FFT), np.zeros(FFT)
    edges = np.arange(-369, 371)
    count = min(count, (len(m)-off-GAP)//PERIOD+1)
    for k in range(count):
        x = read(k*PERIOD, PERIOD)
        rawpow += float(np.sum(abs(x)**2, dtype=np.float64))
        block = m[k*PERIOD:(k+1)*PERIOD]
        clips += int(np.count_nonzero((block == -32768) | (block == 32767)))
        peak = max(peak, int(np.max(np.abs(block.astype(np.int32)))))
        c = np.vdot(short, x.reshape(-1, 288).sum(0))/PERIOD
        tones.append(float(abs(c)**2))
        x -= c*lut
        f, p = welch(x, fs=FS, nperseg=FFT, noverlap=0,
                     return_onesided=False, detrend=False)
        allsum += p
        g = read(k*PERIOD+off, GAP)
        g -= np.vdot(gaplut, g)/GAP*gaplut
        _, q = welch(g, fs=FS, nperseg=FFT, noverlap=FFT//2,
                     return_onesided=False, detrend=False)
        gapsum += q
        if k == 0:
            order = np.argsort(f)
            fm = f[order]/1e6
            # PSD density mean within each 1 MHz bin, including receiver edges.
            bins = np.clip(np.digitize(fm, edges)-1, 0, len(edges)-2)
            bin_count = np.bincount(bins, minlength=len(edges)-1)
        coarse.append(np.bincount(bins, weights=p[order], minlength=len(edges)-1)/np.maximum(bin_count, 1))
        gapbands.append([float(q[(f>=lo*1e6)&(f<hi*1e6)].sum()*FS/FFT) for lo,hi in BANDS])
        if k % 250 == 0:
            print(path.name, k, '/', count, flush=True)
    used = len(coarse)
    avg, gapavg = allsum/used, gapsum/used
    floor = float(np.median(gapavg))
    r = dict(metadata=meta, source=str(path), windows=used,
             fft_bin_hz=FS/FFT, full_record_welch_coverage_fraction=(PERIOD//FFT)*FFT/PERIOD,
             gap_window_samples=GAP, gap_phase_samples=off,
             folded_profile_contrast_db=float(10*np.log10(np.percentile(profile,90)/np.percentile(profile,10))),
             tone_dbfs=float(10*np.log10(np.mean(tones)/32768**2)),
             estimated_floor_dbfs_hz=float(10*np.log10(floor/32768**2)),
             raw_rms_lsb=float(np.sqrt(rawpow/(used*PERIOD))), peak_component_lsb=peak,
             clipped_components=clips, bands=[], lines=[])
    for j, (lo, hi) in enumerate(BANDS):
        mask=(f>=lo*1e6)&(f<hi*1e6)
        power=gapavg[mask].sum()*FS/FFT
        noise=floor*mask.sum()*FS/FFT
        r['bands'].append(dict(baseband_mhz=[lo,hi],rf_mhz=[6489.6+lo,6489.6+hi],
             total_dbfs=float(10*np.log10(power/32768**2)),
             total_over_floor_db=float(10*np.log10(power/noise)),
             excess_dbfs=float(10*np.log10(max(power-noise,1e-12)/32768**2)),
             total_dbfs_p50_p95_max=(10*np.log10(np.percentile(np.array(gapbands)[:,j],[50,95,100])/32768**2)).tolist()))
    for center in [-58.88,94.72,12.8,115.2,-263.68,258.57,74.24,-38.4]:
        mask=abs(f/1e6-center)<.15
        near=(abs(f/1e6-center)>.3)&(abs(f/1e6-center)<1)
        power=(gapavg[mask].sum()-np.median(gapavg[near])*mask.sum())*FS/FFT
        r['lines'].append(dict(baseband_mhz=center,rf_mhz=6489.6+center,
                              excess_dbfs=float(10*np.log10(power/32768**2)) if power>0 else None,
                              peak_over_local_db=float(10*np.log10(np.max(gapavg[mask])/np.median(gapavg[near])))))
    np.savez_compressed(out/(path.stem+'.npz'), frequency_hz=f[order], all_psd=avg[order],
                         gap_psd=gapavg[order], coarse_psd=coarse, edges_mhz=edges,
                         gap_band_power=gapbands, tone_power=tones, folded_profile=profile)
    return r


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--without', type=Path, required=True)
    ap.add_argument('--with-antenna', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    results=[analyze(p,a.out) for p in [a.without,a.with_antenna]]
    labels=['CH2 RX2: no antenna','CH3 RX2: antenna']
    fig,axes=plt.subplots(3,1,figsize=(13,11),constrained_layout=True)
    water,wax=plt.subplots(2,1,figsize=(13,8),constrained_layout=True)
    for r,label,ax in zip(results,labels,wax):
        z=np.load(a.out/(Path(r['source']).stem+'.npz'))
        f=z['frequency_hz']/1e6
        for dest, field in zip(axes[:2],['all_psd','gap_psd']):
            dest.plot(f+6489.6,10*np.log10(z[field]/32768**2)+60,label=label,linewidth=.8)
            dest.set(ylabel='PSD (dBFS/MHz)',xlim=(6239.6,6739.6))
            dest.grid(alpha=.2);dest.legend()
        axes[2].plot(np.arange(256)/256,10*np.log10(z['folded_profile']/32768**2),label=label)
        p=z['coarse_psd']
        im=ax.pcolormesh(z['edges_mhz']+6489.6,np.arange(len(p)+1)/1000,
                         10*np.log10(np.maximum(p,1e-30)/32768**2)+60,
                         vmin=-80,vmax=-45,cmap='magma',shading='flat',rasterized=True)
        ax.set(title=label,xlim=(6239.6,6739.6),ylabel='Time (s)')
        water.colorbar(im,ax=ax,label='PSD (dBFS/MHz)')
    axes[0].set_title('Full-record spectrum: -232.96 MHz fixed tone fitted and removed')
    axes[1].set_title('Estimated packet-gap spectrum: same fixed-tone removal')
    axes[1].set_xlabel('RF frequency (MHz)')
    axes[2].set(xlabel='Phase within 1 ms (ms)',ylabel='Residual power (dBFS)',title='Folded power after fixed-tone removal')
    axes[2].legend();axes[2].grid(alpha=.2)
    wax[-1].set_xlabel('RF frequency (MHz)')
    fig.savefig(a.out/'spectra_comparison.png',dpi=160)
    water.savefig(a.out/'waterfalls.png',dpi=160)
    (a.out/'metrics.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results,indent=2))


if __name__=='__main__':
    main()
