#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Remove a CW single-tone from a scheduled SC16 dump after it is written.

capture.iq is concatenated non-contiguous windows, so a CW is continuous
only inside each jsonl window.  This script notches each window separately
and overwrites capture.iq in place (same geometry; reuse capture.jsonl).
The un-notched IQ is not kept.

Default search is around RF 6200 MHz with LO 6489.6 MHz (baseband
-289.6 MHz, search ±80 MHz).  On the mixed 737.28 dump the spur sits at
about RF 6256.640 MHz.

Usage (repo root):
  python3 testdata/cancel_capture_tone.py DUMP_DIR
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

FS737 = 737.28e6
FC_X410 = 6489.6e6
RF_HINT = 6200e6
SEARCH_HZ = 80e6


def load_jsonl(path):
    metas = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                metas.append(json.loads(line))
    return metas


def fft_peak_hz(x, fs, f_lo=None, f_hi=None):
    n = int(len(x))
    nfft = 1 << int(np.ceil(np.log2(max(n, 4096))))
    w = np.hanning(n)
    spec = np.fft.fftshift(np.fft.fft(x * w, n=nfft))
    freq = np.fft.fftshift(np.fft.fftfreq(nfft, 1.0 / fs))
    mag = np.abs(spec)
    if f_lo is not None or f_hi is not None:
        lo = -0.5 * fs if f_lo is None else f_lo
        hi = 0.5 * fs if f_hi is None else f_hi
        band = (freq >= lo) & (freq <= hi)
        if not np.any(band):
            band = np.ones_like(freq, dtype=bool)
        mag = np.where(band, mag, 0.0)
    i = int(np.argmax(mag))
    if 0 < i < nfft - 1 and mag[i - 1] > 0 and mag[i + 1] > 0:
        a, b, c = mag[i - 1], mag[i], mag[i + 1]
        den = a - 2.0 * b + c
        delta = 0.5 * (a - c) / den if den != 0 else 0.0
        return float(freq[i] + delta * (freq[1] - freq[0])), float(mag[i])
    return float(freq[i]), float(mag[i])


def refine_freq(x, fs, f0, span_hz, steps=5):
    """Maximize |sum x exp(-j 2π f n / fs)| around f0."""
    n = np.arange(len(x), dtype=np.float64)
    best_f = float(f0)
    best = 0.0
    span = float(span_hz)
    for _ in range(steps):
        cands = np.linspace(best_f - span, best_f + span, 21)
        for f in cands:
            s = np.exp(-2j * np.pi * f * n / fs)
            m = abs(np.dot(x, s))
            if m > best:
                best = m
                best_f = float(f)
        span *= 0.25
    return best_f


def fit_and_subtract(x, fs, f_hz):
    n = np.arange(len(x), dtype=np.float64)
    s = np.exp(2j * np.pi * f_hz * n / fs)
    coef = np.dot(x, np.conj(s)) / len(x)
    y = x - coef * s
    before = float(np.mean(np.abs(x) ** 2))
    after = float(np.mean(np.abs(y) ** 2))
    bin_before = abs(coef)
    resid = np.dot(y, np.conj(s)) / len(x)
    return y, coef, before, after, bin_before, abs(resid)


def process_dump(dump_dir, *, fs, fc, rf_hint, search_hz, auto):
    iq_path = os.path.join(dump_dir, "capture.iq")
    jsonl_path = os.path.join(dump_dir, "capture.jsonl")
    out_path = os.path.join(dump_dir, "capture.iq.tmp")
    if not os.path.isfile(iq_path) or not os.path.isfile(jsonl_path):
        raise FileNotFoundError(f"need {iq_path} and {jsonl_path}")

    metas = load_jsonl(jsonl_path)
    raw = np.memmap(iq_path, dtype=np.int16, mode="r")
    out = np.memmap(out_path, dtype=np.int16, mode="w+", shape=raw.shape)

    f_bb_hint = rf_hint - fc
    f_lo, f_hi = f_bb_hint - search_hz, f_bb_hint + search_hz
    if auto:
        f_lo, f_hi = None, None

    # Estimate f from the first long scheduled/provisional window.
    f_hz = f_bb_hint
    probe = None
    for m in metas:
        if int(m.get("sample_count", 0)) >= 10000:
            probe = m
            if m.get("capture_mode") in ("scheduled", "provisional"):
                break
    if probe is None:
        raise RuntimeError("no usable window in jsonl")

    off = int(probe["file_offset_samples"])
    n = int(probe["sample_count"])
    sl = np.asarray(raw[off * 2:(off + n) * 2], dtype=np.float64)
    x = sl[0::2] + 1j * sl[1::2]
    f_coarse, _ = fft_peak_hz(x, fs, f_lo, f_hi)
    f_hz = refine_freq(x, fs, f_coarse, span_hz=max(200.0, fs / len(x) * 8))

    print(f"dump: {dump_dir}")
    print(f"probe packet_id={probe.get('packet_id')} n={n}")
    print(f"tone baseband={f_hz:.3f} Hz  RF={(fc + f_hz) / 1e6:.6f} MHz")
    print(f"search auto={auto} hint_RF={rf_hint / 1e6:.3f} MHz")

    reports = []
    for m in metas:
        n = int(m["sample_count"])
        off = int(m.get("file_offset_samples", 0))
        sl = np.asarray(raw[off * 2:(off + n) * 2], dtype=np.float64)
        if sl.size != n * 2:
            raise RuntimeError(f"short IQ packet_id={m.get('packet_id')}")
        x = sl[0::2] + 1j * sl[1::2]
        y, coef, p0, p1, b0, b1 = fit_and_subtract(x, fs, f_hz)
        sup = 10.0 * np.log10(p0 / p1) if p1 > 0 else float("inf")
        bin_db = 20.0 * np.log10((b0 + 1e-18) / (b1 + 1e-18))
        reports.append({
            "packet_id": int(m.get("packet_id", -1)),
            "capture_mode": m.get("capture_mode", ""),
            "coef_abs": float(abs(coef)),
            "power_suppression_db": float(sup),
            "tone_bin_suppression_db": float(bin_db),
        })
        yi = np.empty(n * 2, dtype=np.int16)
        yi[0::2] = np.clip(np.rint(y.real), -32768, 32767).astype(np.int16)
        yi[1::2] = np.clip(np.rint(y.imag), -32768, 32767).astype(np.int16)
        out[off * 2:(off + n) * 2] = yi

    out.flush()
    del out
    del raw

    powers = [r["power_suppression_db"] for r in reports]
    bins = [r["tone_bin_suppression_db"] for r in reports]
    print(f"windows={len(reports)}  "
          f"power_suppression_db mean={np.mean(powers):.2f} "
          f"min={np.min(powers):.2f}  "
          f"tone_bin_db mean={np.mean(bins):.1f}")
    report = {
        "input": iq_path,
        "output": iq_path,
        "sample_rate": fs,
        "center_hz": fc,
        "tone_baseband_hz": f_hz,
        "tone_rf_hz": fc + f_hz,
        "n_windows": len(reports),
        "power_suppression_db_mean": float(np.mean(powers)),
        "tone_bin_suppression_db_mean": float(np.mean(bins)),
        "windows": reports,
    }
    rep_path = os.path.join(dump_dir, "capture_notch_report.json")
    with open(rep_path, "w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")
    print(f"wrote {rep_path}")

    os.replace(out_path, iq_path)
    for leftover in ("capture_notch.iq", "capture_raw.iq"):
        p = os.path.join(dump_dir, leftover)
        if os.path.exists(p):
            os.remove(p)
    print(f"wrote {iq_path} (notched, un-notched IQ discarded)")
    return report


def parser():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump_dir",
                    help="directory with capture.iq + capture.jsonl")
    ap.add_argument("--fs", type=float, default=FS737)
    ap.add_argument("--center-hz", type=float, default=FC_X410,
                    help="RF LO used when the dump was captured")
    ap.add_argument("--rf-hz", type=float, default=RF_HINT,
                    help="approximate RF of the CW (default 6200 MHz)")
    ap.add_argument("--search-hz", type=float, default=SEARCH_HZ,
                    help="half-width around the RF hint (default 80 MHz)")
    ap.add_argument("--auto", action="store_true",
                    help="notch the strongest bin anywhere, ignore RF hint")
    return ap


def main():
    args = parser().parse_args()
    process_dump(
        os.path.abspath(args.dump_dir),
        fs=args.fs,
        fc=args.center_hz,
        rf_hint=args.rf_hz,
        search_hz=args.search_hz,
        auto=args.auto,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
