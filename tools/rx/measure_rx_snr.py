#!/usr/bin/env python3
"""Measure the received SNR of an X410 SC16 capture from raw sample statistics.

The measurement never looks at the demodulator: it compares the power inside
the packet with the power of the inter-packet noise, and separates the CW tone
from the thermal floor with a spectrum.

    python3 tools/rx/measure_rx_snr.py --input rx.sc16 --period-ms 1
    python3 tools/rx/measure_rx_snr.py --input rx.sc16 --period-ms 5 --json

Definitions used in the output:

  signal      mean power of the packet region minus the noise region,
              integrated over +-250 MHz (the 802.15.4z channel-5 band)
  thermal     median of the noise-region PSD, i.e. the floor with the CW tone
              and the WiFi bursts excluded
  in-band SNR signal / thermal over +-250 MHz

The wideband SNR (signal / everything in the 737.28 MHz Nyquist band) is also
printed; it is much lower because the receiver band is wider than the signal
and because a CW interferer sits inside it.
"""
import argparse
import json
import sys

import numpy as np
from scipy import signal

FS = 737.28e6
BLK = 3686                      # 5 us blocks for the duty-cycle profile
TONE = -91.0 / 288.0            # cycles/sample, = -232.96 MHz
NOTCHES = (-232.96e6, -58.88e6, 94.72e6)


def open_sc16(path):
    n = (__import__('os').path.getsize(path) // 4)
    return np.memmap(path, dtype=np.int16, mode='r', shape=(n, 2)), n


def profile(m, period, nper):
    nblk = period // BLK
    acc = np.zeros(nblk)
    used = 0
    for k in range(nper):
        if (k + 1) * period > len(m):
            break
        a = np.asarray(m[k * period:k * period + nblk * BLK]).astype(np.float32)
        x = a[:, 0] + 1j * a[:, 1]
        acc += (x.real ** 2 + x.imag ** 2).reshape(nblk, BLK).mean(axis=1)
        used += 1
    return acc / max(1, used)


def regions(p):
    """Packet mask = high blocks; noise = the longest contiguous low run.

    The noise run usually wraps around the period boundary, so the wrap is
    considered explicitly.
    """
    db = 10 * np.log10(p)
    lo = np.percentile(db, 10)
    on = db > lo + 0.5 * (db.max() - lo)
    n = len(on)
    best, start = -1, 0
    i = 0
    while i < n:
        if on[i]:
            i += 1
            continue
        j = i
        while j < n and not on[j]:
            j += 1
        if j - i > best:
            best, start = j - i, i
        i = j
    if not on[0] and not on[-1]:
        a = 0
        while a < n and not on[a]:
            a += 1
        b = n - 1
        while b >= 0 and not on[b]:
            b -= 1
        if a + (n - 1 - b) > best:
            best, start = a + (n - 1 - b), b + 1
    noise = np.zeros(n, bool)
    for k in range(best):
        noise[(start + k) % n] = True
    return ~noise, noise


def read_runs(m, o, blocks):
    out = []
    i = 0
    while i < len(blocks):
        j = i
        while j + 1 < len(blocks) and blocks[j + 1] == blocks[j] + 1:
            j += 1
        out.append(np.asarray(m[o + blocks[i] * BLK:o + (blocks[j] + 1) * BLK])
                   .astype(np.float32))
        i = j + 1
    a = np.concatenate(out)
    return a[:, 0] + 1j * a[:, 1]


def psd_avg(m, period, blocks, nper, nperseg=16384):
    acc, f = None, None
    for k in range(nper):
        if (k + 1) * period > len(m):
            break
        x = read_runs(m, k * period, blocks)
        n = (len(x) // nperseg) * nperseg
        if n < nperseg:
            continue
        x = x[:n]
        idx = np.arange(n)
        b = np.exp(-2j * np.pi * TONE * idx)
        a = np.dot(b, x) / n
        x = x - a * np.exp(2j * np.pi * TONE * idx)
        f, q = signal.welch(x, fs=FS, nperseg=nperseg, return_onesided=False,
                            noverlap=nperseg // 2)
        acc = q if acc is None else acc + q
    return f, acc / nper


def tone_power(m, period, blocks, nper=100):
    pw = []
    for k in range(nper):
        if (k + 1) * period > len(m):
            break
        x = read_runs(m, k * period, blocks)
        b = np.exp(-2j * np.pi * TONE * np.arange(len(x)))
        a = np.dot(b, x) / len(x)
        pw.append(abs(a) ** 2)
    return float(np.median(pw)) if pw else 0.0


def measure(path, period_ms, nper=40, profile_periods=200, band_mhz=250.0):
    m, n = open_sc16(path)
    period = int(round(period_ms * 1e-3 * FS))
    if period > n:
        raise SystemExit(f'{path}: shorter than one {period_ms} ms period')
    p = profile(m, period, profile_periods)
    pkt, noi = regions(p)
    pb, nb = np.flatnonzero(pkt), np.flatnonzero(noi)

    t_noi = tone_power(m, period, nb)
    f, pn = psd_avg(m, period, nb, nper)
    _, pp = psd_avg(m, period, pb, nper)
    o = np.argsort(f)
    f, pn, pp = f[o], pn[o], pp[o]
    df = f[1] - f[0]
    med = float(np.median(pn))
    sd = pp - pn

    band = np.abs(f) <= band_mhz * 1e6
    for c in NOTCHES:
        band &= np.abs(f - c) > 3e6
    sig = float(np.sum(np.maximum(sd[band], 0)) * df)
    thr = float(med * np.sum(band) * df)

    return {
        'file': path,
        'period_ms': period_ms,
        'packet_us': float(pkt.sum() * BLK / FS * 1e6),
        'duty_pct': float(100.0 * pkt.sum() / len(pkt)),
        'thermal_psd_db': float(10 * np.log10(med)),
        'thermal_rms_lsb_737': float(np.sqrt(med * FS)),
        'thermal_db_737': float(10 * np.log10(med * FS)),
        'tone_rms_lsb': float(np.sqrt(t_noi)),
        'tone_db': float(10 * np.log10(t_noi)),
        'tone_over_thermal_db': float(10 * np.log10(t_noi / (med * FS))),
        'signal_db': float(10 * np.log10(sig)),
        'signal_rms_lsb': float(np.sqrt(sig)),
        'thermal_db_band': float(10 * np.log10(thr)),
        'snr_db': float(10 * np.log10(sig / thr)),
        'snr_without_tone_cancel_db': float(10 * np.log10(sig / (thr + t_noi))),
        'snr_wideband_db': float(10 * np.log10(sig / (med * FS + t_noi))),
        'band_mhz': band_mhz,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--input', required=True, help='SC16 capture at 737.28 MS/s')
    ap.add_argument('--period-ms', type=float, default=1.0, help='packet period')
    ap.add_argument('--band-mhz', type=float, default=250.0,
                    help='one-sided signal band (default 250 = channel 5)')
    ap.add_argument('--nper', type=int, default=40, help='periods averaged for the PSD')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()

    r = measure(a.input, a.period_ms, a.nper, band_mhz=a.band_mhz)
    if a.json:
        print(json.dumps(r, indent=2))
        return 0
    print(f"{r['file']}   period {r['period_ms']} ms")
    print(f"  packet {r['packet_us']:.1f} us ({r['duty_pct']:.1f}% of the period)")
    print(f"  thermal floor    {r['thermal_psd_db']:6.1f} dB re 1 LSB^2/Hz  "
          f"rms {r['thermal_rms_lsb_737']:6.1f} LSB over 737 MHz "
          f"({20*np.log10(r['thermal_rms_lsb_737']/32768):6.1f} dBFS)")
    print(f"  CW tone          {r['tone_db']:6.1f} dB  rms {r['tone_rms_lsb']:6.1f} LSB  "
          f"({r['tone_over_thermal_db']:+.1f} dB vs the thermal floor)")
    print(f"  signal +-{r['band_mhz']:.0f} MHz {r['signal_db']:6.1f} dB  "
          f"rms {r['signal_rms_lsb']:6.1f} LSB")
    print(f"  thermal +-{r['band_mhz']:.0f} MHz {r['thermal_db_band']:6.1f} dB")
    print(f"  ==> in-band SNR                 {r['snr_db']:+6.2f} dB")
    print(f"      without tone cancellation   {r['snr_without_tone_cancel_db']:+6.2f} dB")
    print(f"      wideband (737.28 MHz)       {r['snr_wideband_db']:+6.2f} dB")
    return 0


if __name__ == '__main__':
    sys.exit(main())
