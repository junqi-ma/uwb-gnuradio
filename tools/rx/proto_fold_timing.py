#!/usr/bin/env python3
"""Feasibility prototype: K-SYMBOL periodic fold + circular cross-correlation as
a low-SNR ACQUISITION confirmation / refinement stage, on the 998.4 MS/s WORK
grid.

Reference plan:
    docs/performance/低SNR进锁阶段优化与快速验证方案_2026-11-17.md  §2 修改 B

Measurement prototype only: no C++, no CMake, no existing test is touched.

Why the work grid
-----------------
The acquisition runs AFTER the 48/65 resampler, at 998.4 MS/s, where a SYNC
symbol is EXACTLY 1016 samples.  The fold stride is therefore exact and no
fractional-delay interpolation is needed.  (On the 737.28 MS/s native grid the
period is 750.2769 samples, i.e. 70.9 samples of drift over 256 SYMBOLs, which
destroys an integer-stride fold -- that analysis is dropped.)

What is measured
----------------
1. 100 packets at -6/-10/-14/-16/-20/-24 dB: folded metric min/median/max,
   residual distribution, margin against the noise-only folded metric.
2. K = 16/32/64/128/256 at -16 dB: metric and margin (the K/CFO trade-off).
3. CFO = 0/350/1000/4250 Hz at -16 dB for K = 64 and K = 256.
4. Coarse-offset sweep 0..4064 at -16 dB: metric and recovered total error,
   which pins the +/-508 (= half a SYMBOL period) requirement.
5. False alarm: 200 fold positions in pure noise -> max metric.
6. Cost: MACs per candidate.

Metric definition (identical in form to `stage_timing::local_best`)
------------------------------------------------------------------
    metric[i] = |sum_k buf[(i+k) mod 1016] conj(t[k])|^2 / (sum_k |buf[k]|^2 * Et)
with t L2-normalised so Et = 1.  The folded buffer is exactly one SYMBOL
period, so the sliding power of the production kernel is a constant here and
the correlation is circular over all 1016 lags (a full period).  Squared
normalised correlation in [0, 1]; `radar_verification_threshold = 0.25`.

The SYMBOL-period ambiguity
---------------------------
The preamble is 256 copies of one SYMBOL, so the fold+correlation has equal
peaks at EVERY SYMBOL start.  It measures the SYMBOL grid phase, not the
absolute packet start.  The absolute start is recovered only if the coarse
candidate is within +/-508 samples (half a period); beyond that the estimate
slips by whole SYMBOLs.  Section 4 measures exactly this.

Usage
-----
    python3 tools/rx/proto_fold_timing.py --packets 100 --seed 1 \
        --json docs/performance/data_low_snr_lock_20261117/fold_timing.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools", "rx"))
import proto_low_snr_signal as sig  # noqa: E402

FS_WORK = sig.FS_WORK                     # 998.4e6
P = sig.SAMPLES_PER_SYMBOL_WORK           # 1016, exact on the work grid
SYNC_REPS = sig.SYNC_REPETITIONS          # 256
PRE_WORK = sig.PREAMBLE_LEN_WORK          # 260,096

THRESHOLD = 0.25                          # radar_verification_threshold
ROI_WORK = 40960                          # timing_search_margin
LOCKED_ROI_WORK = 256                     # locked_roi_samples
HALF_PERIOD = P // 2                      # 508, the +/- requirement

SNRS = (-6.0, -10.0, -14.0, -16.0, -20.0, -24.0)
K_LIST = (16, 32, 64, 128, 256)
K_MAIN = (64, 256)                        # reported per SNR
K_SWEEP_SNR = -16.0
CFO_LIST = (0.0, 350.0, 1000.0, 4250.0)
CFO_SNR = -16.0
CFO_PACKETS = 20
COARSE_LIST = (0, 250, 500, 508, 600, 1016, 2032, 4064)
COARSE_SNR = -16.0
COARSE_PACKETS = 20
NOISE_POSITIONS = 200
NOISE_POSITIONS_SHORT = 100               # "same number of folds" as 100 packets
LOCK_VERIFY_OBS = 3                       # verify_observations
LOCK_TRIAL = 8                            # packets before REACQUIRE


# ---------------------------------------------------------------------------
# kernels
# ---------------------------------------------------------------------------
def fold_period(x: np.ndarray, base: int, k: int, n: int = P) -> np.ndarray:
    """buf[i] = sum_{m=0}^{k-1} x[base + i + m*P], one SYMBOL period long."""
    end = base + (k - 1) * P + n
    if base < 0 or end > x.size:
        raise IndexError(f"fold out of range: base={base} k={k} "
                         f"end={end} size={x.size}")
    buf = np.zeros(n, dtype=np.complex128)
    for m in range(k):
        s = base + m * P
        buf += x[s:s + n]
    return buf


def circ_metric(buf: np.ndarray, tmpl_n: np.ndarray):
    """Circular normalised correlation over all `len(buf)` lags.

    Returns (metric, acc) with
        acc[i] = sum_k buf[(i+k) mod N] conj(t[k]).
    The buffer is exactly one SYMBOL period, so the window power is constant.
    """
    n = buf.size
    L = tmpl_n.size
    if L != n:
        raise ValueError("circular metric needs len(buf) == len(template)")
    acc = np.fft.ifft(np.fft.fft(buf) * np.conj(np.fft.fft(tmpl_n)))
    pw = float(np.sum(np.abs(buf) ** 2))
    return np.abs(acc) ** 2 / (pw + 1e-12), acc


def norm_corr(x: np.ndarray, tmpl_n: np.ndarray, lo: int, hi: int):
    """Linear sliding normalised correlation (production `local_best` form)."""
    L = tmpl_n.size
    seg = np.asarray(x[lo:hi + L], dtype=np.complex128)
    n = seg.size
    from scipy.fft import next_fast_len
    nfft = int(next_fast_len(n + L))
    acc = np.fft.ifft(np.fft.fft(seg, nfft)
                      * np.conj(np.fft.fft(tmpl_n, nfft)))[: n - L + 1]
    cs = np.concatenate(([0.0], np.cumsum(np.abs(seg) ** 2)))
    pw = cs[L:] - cs[:-L]
    return np.abs(acc) ** 2 / (pw + 1e-12), acc


def wrap_half(i: int, period: int = P) -> int:
    """Map a lag to (-period/2, period/2]."""
    return i - period if i > period // 2 else i


def par_fallback(met: np.ndarray, j: int) -> dict:
    """Production PAR fallback: floor 0.06, PAR >= 20, guard 12 lags."""
    bm = float(met[j])
    mask = np.abs(np.arange(met.size) - j) > 12
    if not mask.any():
        return {"peak": bm, "par": None, "accept": False, "active": False}
    bg = float(met[mask].mean())
    par = bm / max(bg, 1e-9)
    return {"peak": bm, "bg_mean": bg, "par": float(par),
            "accept": bool(bm >= 0.06 and par >= 20.0),
            "active": bool(64 <= met.size <= 1024)}


def binom_tail_ge(p: float, n: int, k: int) -> float:
    from math import comb
    return float(sum(comb(n, i) * p ** i * (1.0 - p) ** (n - i)
                     for i in range(k, n + 1)))


def empirical_ge(flags, n: int, k: int):
    if len(flags) < n:
        return None
    hits = [sum(flags[i:i + n]) for i in range(len(flags) - n + 1)]
    return float(sum(1 for h in hits if h >= k) / len(hits))


def noise_stream(snr_db: float, n: int, seed: int) -> np.ndarray:
    """White complex noise at the in-band SNR convention of the generator."""
    p_sig = 542.0 ** 2
    p_total = p_sig * (FS_WORK / sig.INBAND_HZ) / (10.0 ** (snr_db / 10.0))
    rng = np.random.default_rng(seed + 7)
    x = (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    x *= np.sqrt(p_total / 2.0)
    return x.astype(np.complex64)


def apply_cfo(x: np.ndarray, starts, cfo_hz: float, span: int) -> None:
    """Multiply x[s : s+span] by exp(j2*pi*cfo*n/FS) with absolute n (in place)."""
    if cfo_hz == 0.0:
        return
    for s in starts:
        n = np.arange(s, s + span, dtype=np.float64)
        x[s:s + span] *= np.exp(2j * np.pi * cfo_hz * n / FS_WORK).astype(
            np.complex64)


def self_test() -> None:
    rng = np.random.default_rng(7)
    x = rng.standard_normal(5000) + 1j * rng.standard_normal(5000)
    t = rng.standard_normal(P) + 1j * rng.standard_normal(P)
    t = t / np.linalg.norm(t)
    buf = fold_period(x, 3, 4)
    ref = sum(x[3 + m * P:3 + m * P + P] for m in range(4))
    assert np.allclose(buf, ref)
    met, acc = circ_metric(buf, t)
    # circular definition check on a short case
    n = 64
    b = rng.standard_normal(n) + 1j * rng.standard_normal(n)
    tt = rng.standard_normal(n) + 1j * rng.standard_normal(n)
    tt = tt / np.linalg.norm(tt)
    m2, a2 = circ_metric(b, tt)
    ref_a = np.array([np.sum(b[(np.arange(n) + i) % n] * np.conj(tt))
                      for i in range(n)])
    assert np.max(np.abs(a2 - ref_a)) < 1e-9
    assert np.max(np.abs(m2 - np.abs(ref_a) ** 2
                         / np.sum(np.abs(b) ** 2))) < 1e-9
    assert wrap_half(600) == 600 - P and wrap_half(500) == 500


# ---------------------------------------------------------------------------
# one packet
# ---------------------------------------------------------------------------
def analyse_packet(x: np.ndarray, s: int, tmpl_n: np.ndarray,
                   ks=K_MAIN) -> dict:
    out = {"start": int(s)}
    # production-style single-SYMBOL search (linear, +/-40960 work samples)
    lo = max(0, s - ROI_WORK)
    hi = min(x.size - P, s + ROI_WORK)
    met, acc = norm_corr(x, tmpl_n, lo, hi)
    j = int(np.argmax(met))
    out["single_peak_metric"] = float(met[j])
    out["single_pass"] = bool(met[j] >= THRESHOLD)
    out["single_pos_err"] = int(lo + j - s)
    out["single_roi_lags"] = int(hi - lo)
    # single-SYMBOL inside the LockedBounded ROI (+/-256) with the PAR fallback
    nlo, nhi = max(0, s - LOCKED_ROI_WORK), min(x.size - P, s + LOCKED_ROI_WORK)
    nmet, _ = norm_corr(x, tmpl_n, nlo, nhi)
    nj = int(np.argmax(nmet))
    npar = par_fallback(nmet, nj)
    out["single_narrow"] = {
        "peak_metric": float(nmet[nj]), "par": npar["par"],
        "par_active": npar["active"],
        "pass": bool(nmet[nj] >= THRESHOLD), "pass_par": npar["accept"],
        "pass_any": bool(nmet[nj] >= THRESHOLD or npar["accept"]),
    }
    # the fold, at the true start
    out["fold"] = {}
    for k in ks:
        buf = fold_period(x, s, k)
        fmet, facc = circ_metric(buf, tmpl_n)
        fj = int(np.argmax(fmet))
        out["fold"][f"K{k}"] = {
            "peak_metric": float(fmet[fj]),
            "pass": bool(fmet[fj] >= THRESHOLD),
            "lag": fj,
            "residual": int(wrap_half(fj)),
            "metric_at_lag0": float(fmet[0]),
        }
    return out


def summarise_packets(wins, ks=K_MAIN) -> dict:
    n = len(wins)
    single = np.array([w["single_peak_metric"] for w in wins])
    res = {
        "packets": n,
        "baseline_single_symbol": {
            "pass": int(sum(1 for w in wins if w["single_pass"])),
            "pass_rate": sum(1 for w in wins if w["single_pass"]) / n,
            "peak_metric_min": float(single.min()),
            "peak_metric_median": float(np.median(single)),
            "peak_metric_max": float(single.max()),
            "roi_lags": wins[0]["single_roi_lags"],
            "pos_err_median": int(np.median([w["single_pos_err"]
                                             for w in wins])),
        },
        "baseline_single_symbol_narrow": {
            "roi_work": LOCKED_ROI_WORK,
            "pass": int(sum(1 for w in wins if w["single_narrow"]["pass"])),
            "pass_par": int(sum(1 for w in wins
                                if w["single_narrow"]["pass_par"])),
            "pass_any": int(sum(1 for w in wins
                                if w["single_narrow"]["pass_any"])),
            "pass_any_rate": sum(1 for w in wins
                                 if w["single_narrow"]["pass_any"]) / n,
            "peak_metric_median": float(np.median(
                [w["single_narrow"]["peak_metric"] for w in wins])),
            "peak_metric_max": float(np.max(
                [w["single_narrow"]["peak_metric"] for w in wins])),
            "par_median": float(np.median(
                [w["single_narrow"]["par"] for w in wins
                 if w["single_narrow"]["par"] is not None])),
        },
        "fold": {},
    }
    for k in ks:
        m = np.array([w["fold"][f"K{k}"]["peak_metric"] for w in wins])
        r = np.array([w["fold"][f"K{k}"]["residual"] for w in wins])
        res["fold"][f"K{k}"] = {
            "k": k,
            "pass": int(sum(1 for w in wins if w["fold"][f"K{k}"]["pass"])),
            "pass_rate": sum(1 for w in wins
                             if w["fold"][f"K{k}"]["pass"]) / n,
            "metric_min": float(m.min()),
            "metric_median": float(np.median(m)),
            "metric_max": float(m.max()),
            "residual_min": int(r.min()),
            "residual_median": int(np.median(r)),
            "residual_max": int(r.max()),
            "residual_nonzero": int(np.sum(r != 0)),
            "residual_abs_max": int(np.max(np.abs(r))),
        }
    return res


# ---------------------------------------------------------------------------
# noise floor
# ---------------------------------------------------------------------------
def noise_floor(snr_db: float, seed: int, tmpl_n: np.ndarray,
                ks=K_MAIN) -> dict:
    """Fold at random positions in pure noise: the false-candidate metric.

    The metric is |acc|^2/pw and the fold scales acc and pw equally, so the
    floor is essentially independent of K; both K values are measured to show
    that.
    """
    span = 2 * 1000 * P + 4 * P
    x = noise_stream(snr_db, span + 300 * P, seed)
    out = {}
    for k in ks:
        peaks = []
        for i in range(NOISE_POSITIONS):
            base = 1000 + int(i * (x.size - (k + 2) * P - 2000)
                              / max(1, NOISE_POSITIONS - 1))
            buf = fold_period(x, base, k)
            fmet, _ = circ_metric(buf, tmpl_n)
            peaks.append(float(fmet.max()))
        peaks = np.asarray(peaks)
        out[f"K{k}"] = {
            "fold_symbols": k,
            "positions": NOISE_POSITIONS,
            "lags_per_fold": P,
            "trials": NOISE_POSITIONS * P,
            "metric_median": float(np.median(peaks)),
            "metric_p95": float(np.percentile(peaks, 95)),
            "metric_max_100": float(np.max(peaks[:NOISE_POSITIONS_SHORT])),
            "metric_max": float(np.max(peaks)),
        }
    out["note"] = ("pure-noise fold at random positions; near K-independent "
                   "because the fold scales acc and pw equally")
    return out


# ---------------------------------------------------------------------------
# CFO sweep
# ---------------------------------------------------------------------------
def cfo_sweep(tmpl_n: np.ndarray, packets: int, seed: int) -> dict:
    x, meta = sig.make_stream_work(CFO_SNR, packets, 1.0, seed)
    starts = meta["packet_starts"]
    span = PRE_WORK
    out = {}
    for cfo in CFO_LIST:
        y = x.copy()
        apply_cfo(y, starts, cfo, span)
        rec = {"cfo_hz": cfo, "fold": {}}
        for k in K_MAIN:
            mets, resid, fails = [], [], 0
            for s in starts:
                try:
                    buf = fold_period(y, s, k)
                except IndexError:
                    fails += 1
                    continue
                fmet, _ = circ_metric(buf, tmpl_n)
                j = int(np.argmax(fmet))
                mets.append(float(fmet[j]))
                resid.append(int(wrap_half(j)))
            if not mets:
                rec["fold"][f"K{k}"] = {"valid": False}
                continue
            rec["fold"][f"K{k}"] = {
                "metric_min": float(np.min(mets)),
                "metric_median": float(np.median(mets)),
                "metric_max": float(np.max(mets)),
                "pass_rate": float(np.mean([m >= THRESHOLD for m in mets])),
                "residual_abs_max": int(np.max(np.abs(resid))),
                "residual_median": int(np.median(resid)),
                "skipped_out_of_range": fails,
                "per_symbol_rotation_rad": float(
                    2 * np.pi * cfo * P / FS_WORK),
                "total_rotation_rad": float(
                    2 * np.pi * cfo * k * P / FS_WORK),
            }
        out[f"{cfo:g}"] = rec
    return out


# ---------------------------------------------------------------------------
# coarse-offset sweep
# ---------------------------------------------------------------------------
def coarse_sweep(tmpl_n: np.ndarray, packets: int, seed: int) -> dict:
    x, meta = sig.make_stream_work(COARSE_SNR, packets, 1.0, seed)
    starts = meta["packet_starts"]
    out = {}
    for e in COARSE_LIST:
        rec = {"coarse_err": e, "fold": {}}
        for k in K_MAIN:
            mets, errs, resids, skipped = [], [], [], 0
            for s in starts:
                cand = s + e
                base = cand - HALF_PERIOD     # snap to the nearest SYMBOL
                try:
                    buf = fold_period(x, base, k)
                except IndexError:
                    skipped += 1
                    continue
                fmet, _ = circ_metric(buf, tmpl_n)
                j = int(np.argmax(fmet))
                rec_pos = base + j
                mets.append(float(fmet[j]))
                errs.append(int(rec_pos - s))
                resids.append(int(wrap_half(j)))
            if not mets:
                rec["fold"][f"K{k}"] = {"valid": False,
                                        "skipped_out_of_range": skipped}
                continue
            rec["fold"][f"K{k}"] = {
                "metric_min": float(np.min(mets)),
                "metric_median": float(np.median(mets)),
                "metric_max": float(np.max(mets)),
                "pass_rate": float(np.mean([m >= THRESHOLD for m in mets])),
                "total_err_median": int(np.median(errs)),
                "total_err_min": int(np.min(errs)),
                "total_err_max": int(np.max(errs)),
                "total_err_abs_max": int(np.max(np.abs(errs))),
                "residual_abs_max": int(np.max(np.abs(resids))),
                "correct_abs_le_half_symbol": bool(
                    np.max(np.abs(errs)) <= HALF_PERIOD),
                "skipped_out_of_range": skipped,
            }
        out[f"{e}"] = rec
    return out


# ---------------------------------------------------------------------------
# cost
# ---------------------------------------------------------------------------
def cost_accounting() -> dict:
    out = {}
    for k in (64, 256):
        fold_adds = k * P
        corr_direct = P * P
        # 3 complex FFTs of length 1016 (buf, template, inverse)
        fft_ops = 3 * 5 * P * int(np.ceil(np.log2(P)))
        for tag, corr in (("direct", corr_direct), ("fft", fft_ops)):
            total = fold_adds + corr
            out[f"K{k}_{tag}"] = {
                "fold_complex_adds": fold_adds,
                "corr_complex_ops": corr,
                "per_candidate_complex_ops": total,
                "per_candidate_real_flops": total * 4,
                "real_flops_per_s_at_1_per_ms": total * 4 * 1000,
                "pct_of_one_core_at_10_gflops": total * 4 * 1000 / 1e10 * 100,
            }
    out["note"] = ("one fold of K SYMBOLs into a 1016-sample buffer plus one "
                   "1016-point circular correlation; 1 candidate/ms = 1000 "
                   "candidates/s; complex op counted as 4 real flops")
    return out


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--packets", type=int, default=100)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--json", default=None)
    ap.add_argument("--snr", default=None,
                    help="override the SNR list, e.g. -16,-20")
    a = ap.parse_args(argv)

    self_test()

    tw = sig.load_sync_template().astype(np.complex128)
    tmpl_n = tw / np.linalg.norm(tw)

    snrs = ([float(v) for v in a.snr.split(",") if v.strip()]
            if a.snr else list(SNRS))
    out = {
        "script": "tools/rx/proto_fold_timing.py",
        "purpose": "K-SYMBOL fold + circular cross-correlation as a low-SNR "
                   "acquisition confirmation/refinement stage (work grid)",
        "plan": "docs/performance/低SNR进锁阶段优化与快速验证方案_2026-11-17.md",
        "note": "prototype only; not the production stage_timing",
        "grid": {"fs_hz": FS_WORK, "samples_per_symbol": P,
                 "preamble_len": PRE_WORK, "period_exact": True,
                 "generator": "proto_low_snr_signal.make_stream_work"},
        "parameters": {
            "threshold": THRESHOLD, "roi_work": ROI_WORK,
            "locked_roi_work": LOCKED_ROI_WORK, "half_period": HALF_PERIOD,
            "k_list": list(K_LIST), "k_main": list(K_MAIN),
            "cfo_list_hz": list(CFO_LIST), "coarse_list": list(COARSE_LIST),
            "noise_positions": NOISE_POSITIONS,
            "packets": a.packets, "seed": a.seed,
        },
        "cost": cost_accounting(),
        "results": {},
        "k_sweep": {},
        "cfo_sweep": {},
        "coarse_sweep": {},
        "noise_floor": {},
    }

    # ---- 1. per-SNR statistics over `packets` packets ---------------------
    for snr in snrs:
        t0 = time.time()
        x, meta = sig.make_stream_work(snr, a.packets, 1.0, a.seed)
        starts = meta["packet_starts"]
        wins = [analyse_packet(x, s, tmpl_n) for s in starts]
        del x
        r = summarise_packets(wins)
        r["runtime_s"] = round(time.time() - t0, 2)
        r["per_packet"] = [
            {"start": w["start"],
             "single_peak_metric": w["single_peak_metric"],
             "single_pass": w["single_pass"],
             "single_narrow_pass_any": w["single_narrow"]["pass_any"],
             "fold_K64_metric": w["fold"]["K64"]["peak_metric"],
             "fold_K64_pass": w["fold"]["K64"]["pass"],
             "fold_K64_residual": w["fold"]["K64"]["residual"],
             "fold_K256_metric": w["fold"]["K256"]["peak_metric"],
             "fold_K256_pass": w["fold"]["K256"]["pass"],
             "fold_K256_residual": w["fold"]["K256"]["residual"]}
            for w in wins]
        out["results"][f"{snr:g}"] = r
        b = r["baseline_single_symbol"]
        bn = r["baseline_single_symbol_narrow"]
        f64 = r["fold"]["K64"]
        f256 = r["fold"]["K256"]
        print(f"[{snr:+.0f} dB] single {b['pass']}/{r['packets']} "
              f"(med {b['peak_metric_median']:.4f}) | narrow {bn['pass_any']}"
              f"/{r['packets']} (med {bn['peak_metric_median']:.4f}) | "
              f"fold64 {f64['pass']}/{r['packets']} med {f64['metric_median']:.3f} "
              f"[{f64['metric_min']:.3f},{f64['metric_max']:.3f}] resid "
              f"{f64['residual_abs_max']} | fold256 {f256['pass']}"
              f"/{r['packets']} med {f256['metric_median']:.3f} "
              f"[{f256['metric_min']:.3f},{f256['metric_max']:.3f}] resid "
              f"{f256['residual_abs_max']} | {r['runtime_s']:.1f}s",
              flush=True)

    # ---- 5. noise floor ---------------------------------------------------
    nf = noise_floor(min(snrs), a.seed, tmpl_n)
    out["noise_floor"] = nf
    for snr in snrs:
        r = out["results"][f"{snr:g}"]
        for k in K_MAIN:
            key = f"K{k}"
            nmax = nf[key]["metric_max_100"]
            m = r["fold"][key]["metric_median"]
            r["fold"][key]["noise_metric_max_100"] = nmax
            r["fold"][key]["noise_metric_max_200"] = nf[key]["metric_max"]
            r["fold"][key]["margin_db_vs_noise_max_100"] = (
                float(20 * np.log10(m / nmax)) if nmax > 0 else None)
            r["fold"][key]["margin_db_vs_noise_max_200"] = (
                float(20 * np.log10(m / nf[key]["metric_max"]))
                if nf[key]["metric_max"] > 0 else None)
            r["fold"][key]["margin_db_min_vs_noise_max_100"] = (
                float(20 * np.log10(r["fold"][key]["metric_min"] / nmax))
                if nmax > 0 else None)
        # lock closure: ACQUIRE->VERIFY needs verify_observations=3 successes
        # within 8 consecutive packets (7 consecutive misses tolerated)
        for k in K_MAIN:
            flags = [bool(w[f"fold_K{k}_pass"]) for w in r["per_packet"]]
            p = float(np.mean(flags))
            r.setdefault("lock_closure", {})[f"K{k}"] = {
                "per_packet_p": p,
                "p_ge3_in_8_binomial": binom_tail_ge(p, LOCK_TRIAL,
                                                     LOCK_VERIFY_OBS),
                "empirical_ge3_in_8": empirical_ge(flags, LOCK_TRIAL,
                                                   LOCK_VERIFY_OBS),
                "empirical_windows": max(0, len(flags) - LOCK_TRIAL + 1),
            }
        flags = [bool(w["single_narrow_pass_any"]) for w in r["per_packet"]]
        p = float(np.mean(flags))
        r["lock_closure"]["single_symbol_narrow"] = {
            "per_packet_p": p,
            "p_ge3_in_8_binomial": binom_tail_ge(p, LOCK_TRIAL,
                                                 LOCK_VERIFY_OBS),
            "empirical_ge3_in_8": empirical_ge(flags, LOCK_TRIAL,
                                              LOCK_VERIFY_OBS),
            "empirical_windows": max(0, len(flags) - LOCK_TRIAL + 1),
        }

    # ---- 2. K sweep at -16 dB --------------------------------------------
    x, meta = sig.make_stream_work(K_SWEEP_SNR, a.packets, 1.0, a.seed)
    starts = meta["packet_starts"]
    wins = [analyse_packet(x, s, tmpl_n, ks=K_LIST) for s in starts]
    del x
    for k in K_LIST:
        m = np.array([w["fold"][f"K{k}"]["peak_metric"] for w in wins])
        r = np.array([w["fold"][f"K{k}"]["residual"] for w in wins])
        nmax = nf[f"K{k}"]["metric_max_100"] if f"K{k}" in nf else None
        out["k_sweep"][f"K{k}"] = {
            "k": k,
            "metric_min": float(m.min()),
            "metric_median": float(np.median(m)),
            "metric_max": float(m.max()),
            "pass_rate": float(np.mean(m >= THRESHOLD)),
            "residual_abs_max": int(np.max(np.abs(r))),
            "noise_metric_max_100": nmax,
            "margin_db_vs_noise_max_100": (
                float(20 * np.log10(np.median(m) / nmax)) if nmax else None),
            "total_rotation_rad_at_4250hz": float(
                2 * np.pi * 4250.0 * k * P / FS_WORK),
            "coherent_loss_db_at_4250hz": float(20 * np.log10(max(
                1e-12, abs(np.sin(k * np.pi * 4250.0 * P / FS_WORK)
                           / (k * np.sin(np.pi * 4250.0 * P / FS_WORK)))))),
        }

    # extra noise floors for the K values not in K_MAIN
    extra = noise_floor(K_SWEEP_SNR, a.seed, tmpl_n,
                        ks=tuple(k for k in K_LIST if k not in K_MAIN))
    for k in K_LIST:
        key = f"K{k}"
        nmax = (nf.get(key) or extra.get(key) or {}).get("metric_max_100")
        if nmax:
            out["k_sweep"][key]["noise_metric_max_100"] = nmax
            out["k_sweep"][key]["margin_db_vs_noise_max_100"] = float(
                20 * np.log10(out["k_sweep"][key]["metric_median"] / nmax))
    out["noise_floor"]["extra_K"] = extra

    # ---- 3. CFO sweep -----------------------------------------------------
    out["cfo_sweep"] = cfo_sweep(tmpl_n, CFO_PACKETS, a.seed)

    # ---- 4. coarse-offset sweep ------------------------------------------
    out["coarse_sweep"] = coarse_sweep(tmpl_n, COARSE_PACKETS, a.seed)

    # ---- console summary --------------------------------------------------
    print("\nK sweep @ -16 dB:")
    for k in K_LIST:
        v = out["k_sweep"][f"K{k}"]
        print(f"  K={k:3d} metric {v['metric_median']:.3f} "
              f"[{v['metric_min']:.3f},{v['metric_max']:.3f}] "
              f"pass {v['pass_rate']*100:.0f}% "
              f"margin {v['margin_db_vs_noise_max_100']:.1f} dB "
              f"resid<={v['residual_abs_max']}")
    print("\nCFO sweep @ -16 dB:")
    for cfo, rec in out["cfo_sweep"].items():
        parts = []
        for k in K_MAIN:
            v = rec["fold"][f"K{k}"]
            if v.get("valid") is False:
                parts.append(f"K{k}=n/a")
            else:
                parts.append(f"K{k}={v['metric_median']:.3f}"
                             f"(pass {v['pass_rate']*100:.0f}%,"
                             f"resid {v['residual_abs_max']})")
        print(f"  {cfo:>6s} Hz: " + "  ".join(parts))
    print("\nCoarse-offset sweep @ -16 dB:")
    for e, rec in out["coarse_sweep"].items():
        parts = []
        for k in K_MAIN:
            v = rec["fold"][f"K{k}"]
            if v.get("valid") is False:
                parts.append(f"K{k}=n/a")
            else:
                parts.append(f"K{k}: m={v['metric_median']:.3f} "
                             f"err={v['total_err_median']:+d}")
        print(f"  {e:>5s}: " + "  ".join(parts))
    print("\nnoise floor: "
          + "  ".join(f"{k} med {v['metric_median']:.4f} "
                      f"max100 {v['metric_max_100']:.4f} "
                      f"max200 {v['metric_max']:.4f}"
                      for k, v in out["noise_floor"].items()
                      if k.startswith("K")))

    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)
        print("wrote", a.json)
    else:
        print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
