#!/usr/bin/env python3
"""Offline packet-free spectrum survey of the 2026-11-16 distance captures.

Uses one 177.8 us off-packet window every 4 ms. Powers use complex RMS,
0 dBFS = 32768**2 LSB**2; no conversion to antenna dBm is available.
"""
import argparse
import json
import re
from pathlib import Path

import numpy as np
from scipy.signal import welch

FS = 737280000
PERIOD = 737280
LENGTH = 131072
PACKET = 329161
FFT = 32768


def analyze(folder, out):
    raw = folder / 'rx_2s_r01.sc16'
    meta = json.loads(raw.with_suffix('.sc16.json').read_text())
    if meta['rate'] != FS or abs(meta['freq'] - 6489.6e6) > 1 or meta['errors']:
        raise ValueError(f'{raw}: unsupported rate/center frequency or capture errors')
    m = np.memmap(raw, dtype=np.int16, mode='r').reshape(-1, 2)
    starts = []
    for line in (folder / 'rx_2s_r01.demod.log').read_text().splitlines():
        if line.startswith('PACKET ') and 'status=OK ' in line:
            starts.append(int(re.search(r' start=(\d+)', line)[1]))
    lut_period = np.exp(2j * np.pi * (-91 / 288) * np.arange(288))

    def read(offset, size):
        a = m[offset:offset + size].astype(np.float32)
        return a[:, 0] + 1j * a[:, 1]

    if starts:
        phase = starts[0] % PERIOD
        phase_method = 'first CRC-valid preamble start modulo 1 ms'
    else:
        # A folded mean after removing the coherent tone also works without CRC.
        profile = np.zeros(256)
        for k in range(0, 2000, 50):
            x = read(k * PERIOD, PERIOD)
            c = np.vdot(lut_period, x.reshape(-1, 288).sum(axis=0)) / PERIOD
            x -= c * np.tile(lut_period, PERIOD // 288)
            profile += (abs(x) ** 2).reshape(256, -1).mean(axis=1)
        span = round(PACKET / PERIOD * 256)
        phase_bin = max(range(256), key=lambda i: np.take(profile, np.arange(i, i + span), mode='wrap').sum())
        phase = phase_bin * (PERIOD // 256)
        phase_method = 'maximum folded 446 us energy, 40 periods across 2 s'
    # Center a short window in the ~554 us gap. At least 180 us guard each side.
    off = (phase + PACKET + (PERIOD - PACKET - LENGTH) // 2) % PERIOD
    lut = np.exp(2j * np.pi * (-91 / 288) * np.arange(LENGTH)).astype(np.complex64)
    spectra, tones, times = [], [], []
    for k in range(0, 1999, 4):
        x = read(k * PERIOD + off, LENGTH)
        c = np.vdot(lut, x) / LENGTH
        tones.append(float(abs(c) ** 2))
        x -= c * lut
        f, p = welch(x, fs=FS, nperseg=FFT, noverlap=FFT // 2,
                     return_onesided=False, detrend=False)
        spectra.append(p)
        times.append((k * PERIOD + off) / FS)
    order = np.argsort(f)
    f = f[order]
    spectra = np.array(spectra)[:, order]
    mean = spectra.mean(axis=0)
    # Same robust floor definition as measure_rx_snr.py, after fixed-tone subtraction.
    floor = float(np.median(mean))
    np.savez_compressed(out / (folder.name + '.npz'), frequency_hz=f,
                        times_s=times, psd=spectra, tone_power=tones)
    r = dict(folder=str(folder), capture_metadata=meta, phase_samples=phase,
             phase_method=phase_method, off_window_phase_samples=off,
             off_window_samples=LENGTH, sampled_windows=len(times), step_ms=4,
             fft_bin_hz=FS / FFT, enbw_hz=1.5 * FS / FFT,
             thermal_psd_lsb2_hz=floor, thermal_dbfs_hz=float(10*np.log10(floor/32768**2)),
             tone_rms_lsb=float(np.sqrt(np.mean(tones))),
             tone_dbfs=float(10*np.log10(np.mean(tones)/32768**2)))
    (out / (folder.name + '.json')).write_text(json.dumps(r, indent=2))
    print(folder.name, 'phase', phase, 'off', off, 'floor', round(r['thermal_dbfs_hz'], 2),
          'tone', round(r['tone_dbfs'], 2), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    for folder in sorted(a.root.glob('d*m_period1ms')):
        analyze(folder, a.out)


if __name__ == '__main__':
    main()
