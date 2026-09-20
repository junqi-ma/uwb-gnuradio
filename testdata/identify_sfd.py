#!/usr/bin/env python3
"""Identify the transmitted SFD type of captured DW3000 packets.

Method (CFO-robust, FFT correlations, no demod guesses):

  1. Load selected windows from a native-rate dump, resample to 998.4.
  2. Locate the preamble symbol grid with a *magnitude* single-SYNC
     correlation (immune to CFO over a 1 us symbol).
  3. Estimate CFO from the phase slope across preamble SYNCs and derotate.
  4. Build candidate SFD waveforms kron(sfd_symbols, single-SYNC template)
     for every SFD the firmware knows (ieee / decawave / 4z1 / 4z2 / 4z3
     / 4z4) and correlate the derotated signal over the SFD region.
  5. Report the normalized peak per candidate.  The best-scoring candidate
     is the transmitted sfdType.

Usage:
  python3 testdata/identify_sfd.py DUMPDIR [--slots 20] [--code 10]
        [--reps 128] [--native-rate 491.52e6]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy.signal import fftconvolve, resample_poly

HERE = os.path.dirname(os.path.abspath(__file__))

SFD = {
    "ieee":     [0, 1, 0, -1, 1, 0, 0, -1],
    "decawave": [-1, -1, -1, -1, 1, -1, 0, 0],
    "4z1":      [-1, -1, 1, -1],
    "4z2":      [-1, -1, -1, 1, -1, -1, 1, -1],
    "4z3":      [-1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1],
    "4z4":      [-1, -1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1,
                 -1, 1, -1, -1, -1, 1, 1, -1, -1, -1, 1, -1, 1, 1, -1, -1],
}

TMPL = {
    9: "reference_preamble_code9_491p52.cf32",
    10: "reference_preamble_code10_491p52.cf32",
}
TMPL_737 = {
    9: "reference_preamble_code9_737p28.cf32",
    10: "reference_preamble_code10_737p28.cf32",
}


def normalized_corr(x, t):
    """FFT normalized correlation; returns (metric, offset_in_x)."""
    e_t = float(np.sum(np.abs(t) ** 2)) + 1e-30
    corr = fftconvolve(x, np.conj(t[::-1]), mode="valid")
    p = np.abs(x) ** 2
    csum = np.concatenate(([0.0], np.cumsum(p)))
    M = len(t)
    e = csum[M:] - csum[:-M]
    metric = (np.abs(corr) ** 2) / (e * e_t + 1e-30)
    return metric


def estimate_cfo(x, t, p0, period, n_sym, fs):
    """Coarse CFO search maximizing coherent sum of SYNC correlation peaks.

    More robust than a phase fit: works even when individual peak phases
    wrap, and returns 0 only when no frequency beats the DC hypothesis.
    """
    y = fftconvolve(x, np.conj(t[::-1]), mode="valid")
    if p0 < 0 or len(y) < 2:
        return 0.0
    ks = [k for k in range(n_sym)
          if 0 <= p0 + int(round(k * period)) < len(y)]
    if len(ks) < 4:
        return 0.0
    best_f, best_v = 0.0, 0.0
    for f in np.arange(-300e3, 300e3 + 1.0, 1e3):
        acc = 0.0 + 0.0j
        for k in ks:
            acc += y[p0 + int(round(k * period))] * np.exp(
                -1j * 2 * np.pi * f * k * period / fs)
        v = abs(acc)
        if v > best_v:
            best_v, best_f = v, float(f)
    return best_f


def recover_sfd_symbols(x, t, p0, period, cfo, fs, kmax=200):
    """Read the transmitted SFD signs directly from the preamble grid.

    Correlates each symbol slot with the SYNC template, derotates the CFO,
    then takes the sign of the in-phase component relative to the preamble
    phase.  The SFD is the last block of >=4 signed symbols before the
    correlation collapses into the (BPM-BPSK) payload.
    """
    y = fftconvolve(x, np.conj(t[::-1]), mode="valid")
    e = None
    half = int(period * 0.5)
    syms = []  # (k, peak_metric, phase_deg)
    for k in range(kmax):
        center = p0 + int(round(k * period))
        lo = max(0, center - half)
        hi = min(len(y), center + half + 1)
        if hi <= lo:
            break
        seg = np.abs(y[lo:hi])
        j = int(seg.argmax())
        # window energy for a normalized metric
        a = max(0, (lo + j) - half)
        b = min(len(x), (lo + j) + len(t) + half)
        pwr = float(np.sum(np.abs(x[a:b]) ** 2)) + 1e-30
        peak = float(np.abs(y[lo + j]) ** 2) / (
            pwr * float(np.sum(np.abs(t) ** 2)) / max(1, (b - a)) * len(t))
        ph = np.angle(y[lo + j] * np.exp(-1j * 2 * np.pi * cfo * k * period / fs))
        syms.append((k, peak, ph))
    if len(syms) < 16:
        return None
    # preamble reference phase = circular mean over the first 24 symbols
    ref = np.angle(np.mean([np.exp(1j * p) for _, _, p in syms[:24]]))
    # payload start = first k beyond 16 where metric collapses
    thr = 0.25
    k_end = None
    for k, peak, ph in syms:
        if k >= 16 and peak < thr:
            k_end = k - 1
            break
    if k_end is None:
        k_end = syms[-1][0]
    if k_end < 4:
        return None
    start = max(0, k_end - 7)
    signs = []
    for k, peak, ph in syms[start:k_end + 1]:
        signs.append(1 if np.cos(ph - ref) >= 0 else -1)
    return start, k_end, signs, syms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("--slots", type=int, default=20,
                    help="number of dump windows to analyze")
    ap.add_argument("--start-slot", type=int, default=1)
    ap.add_argument("--code", type=int, default=10)
    ap.add_argument("--reps", type=int, default=0,
                    help="known SYNC repetitions; 0 = sweep "
                         "{32,64,128,256,512}")
    ap.add_argument("--native-rate", type=float, default=491.52e6)
    a = ap.parse_args()

    tmpl_name = (TMPL if abs(a.native_rate - 491.52e6) < 1.0
                 else TMPL_737)[a.code]
    tmpl_path = os.path.join(HERE, tmpl_name)
    t = np.fromfile(tmpl_path, np.complex64)
    print(f"template code{a.code}: {tmpl_name} ({len(t)}) native domain")

    sym = len(t)  # one SYNC at native rate (~500 @491.52, ~751 @737.28)
    fs = a.native_rate
    period = 1016.0 * fs / 998.4e6  # true fractional SYNC period

    jsonl = os.path.join(a.dump, "capture.jsonl")
    iq_path = os.path.join(a.dump, "capture.iq")
    metas = [json.loads(l) for l in open(jsonl) if l.strip()]
    iq = np.memmap(iq_path, dtype=np.int16, mode="r")

    results = {k: [] for k in SFD}
    cfo_list = []
    best_by_rep = {}
    recovered = []
    used = 0
    reps_list = ([a.reps] if a.reps > 0 else [32, 64, 128, 256, 512])
    for meta in metas[a.start_slot:a.start_slot + a.slots]:
        off = int(meta.get("file_offset_samples", 0))
        n = int(meta.get("sample_count", 0))
        if n <= 0:
            continue
        x = (iq[off * 2:(off + n) * 2].astype(np.float32)[0::2]
             + 1j * iq[off * 2:(off + n) * 2].astype(np.float32)[1::2]
             ) / 32768.0

        mpre = normalized_corr(x, t)
        pk = int(mpre.argmax())
        if mpre[pk] < 0.15:
            continue
        strong = np.where(mpre > 0.6 * mpre[pk])[0]
        p0 = int(strong[0])
        cfo = estimate_cfo(x, t, p0, period, min(max(reps_list), 100), fs)
        cfo_list.append(cfo)
        xd = x * np.exp(-1j * 2 * np.pi * cfo * np.arange(len(x)) / fs)

        rec = recover_sfd_symbols(xd, t, p0, period, 0.0, fs)
        if rec is not None:
            recovered.append(rec)

        for reps in reps_list:
            expected = int(round(p0 + reps * period))
            lo = max(0, expected - 3 * sym)
            hi = min(len(xd) - 1, expected + 5 * sym)
            for name, seq in SFD.items():
                wf = np.concatenate([s * t for s in seq]).astype(np.complex64)
                mm = normalized_corr(xd, wf)
                if mm.size == 0:
                    continue
                lo2 = max(0, min(lo, mm.size - 1))
                hi2 = min(hi, mm.size)
                if hi2 <= lo2:
                    continue
                seg = mm[lo2:hi2]
                j = int(seg.argmax())
                peak = float(seg[j])
                results[name].append((peak, lo2 + j - expected, reps))
                if peak > best_by_rep.get(reps, (0, "", 0))[0]:
                    best_by_rep[reps] = (peak, name, lo2 + j - expected)
        used += 1

    if used == 0:
        print("FAIL: no usable window (preamble corr < 0.15)")
        return 2

    print(f"windows analyzed={used}  "
          f"CFO mean={np.mean(cfo_list)/1e3:.1f} kHz "
          f"(std {np.std(cfo_list)/1e3:.1f} kHz)")
    print(f"\n{'SFD':<10} {'peak_med':>9} {'peak_max':>9} "
          f"{'offset_med':>11} {'rep_med':>7}  votes")
    ranked = []
    for name in SFD:
        v = results[name]
        if not v:
            continue
        peaks = np.array([p for p, _, _ in v])
        offs = np.array([o for _, o, _ in v])
        reps = np.array([r for _, _, r in v])
        ranked.append((float(np.median(peaks)), name, float(np.max(peaks)),
                       float(np.median(offs)), float(np.median(reps))))
    for med, name, mx, off, rep in sorted(ranked, reverse=True):
        print(f"{name:<10} {med:9.3f} {mx:9.3f} {off:11.0f} {rep:7.0f}  "
              f"{len(results[name])}")
    if ranked:
        print(f"\nBEST SFD = {ranked[0][1]} "
              f"(median peak {ranked[0][0]:.3f})")
    print("\nbest candidate per SYNC-repetition guess:")
    for reps in sorted(best_by_rep):
        pk, name, off = best_by_rep[reps]
        print(f"  reps={reps:<4} -> {name:<9} peak={pk:.3f} offset={off}")

    print("\n=== Direct SFD symbol recovery (last signed block "
          "before payload) ===")
    if not recovered:
        print("  (no recoverable window)")
    else:
        from collections import Counter
        seqs = Counter(tuple(s) for _, _, s, _ in recovered)
        for seq, cnt in seqs.most_common(5):
            match = [n for n, v in SFD.items() if list(v) == list(seq)]
            print(f"  [{cnt}/{len(recovered)}] {list(seq)}"
                  f"{'  == ' + match[0] if match else '  (no exact match)'}")
        # also try matching length-4 suffix / prefix of the block
        for seq, cnt in seqs.most_common(3):
            for name, v in SFD.items():
                for off in range(0, max(1, len(seq) - len(v) + 1)):
                    if list(seq[off:off + len(v)]) == list(v):
                        print(f"    {name} matches at offset {off} of "
                              f"{list(seq)}")
        # print one representative per-symbol trace
        _, _, _, syms = recovered[0]
        print("  per-symbol (k: peak_deg):")
        for k, peak, ph in syms[:4] + syms[-14:]:
            print(f"    {k:3d}  peak={peak:.3f}  phase="
                  f"{np.degrees(ph):+7.1f}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
