#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent frequency-response verifier for the 65/48 anti-imaging FIR.

This script is deliberately standalone: it reads the *binary* float32 tap file
that the runtime loads and re-derives every number from scratch.  It does not
import, call or trust `design_resampler_equiripple.py`, and it never consumes a
"passed" flag produced by a designer.

Frozen specification (docs/performance/OpenCode_65_48等波纹FIR设计实验任务书_2026-11-11.md §2)

    Fv = 65 * 737.28 MHz = 47.9232 GHz          (design rate)
    passband   [0, B]              : 20log10(max|H|/min|H|) <= 0.1 dB
    stopband   [Fin-B, Fv/2]      : 20log10(max|H_stop|/min|H_pass|) <= -70 dB
    sum(h32) accumulated in float64: |sum - 65| < 1e-3
    all finite, real, symmetric (Type I, odd length)

The stopband is the FULL upper half band [Fin-B, Fv/2], not the historical
[Fin-B, 500 MHz] slice used by the original Kaiser script.  Both stopband
references (passband min, and the older passband max) are reported side by
side; they are never silently mixed.

Usage
-----
    python3 testdata/verify_resampler_equiripple.py \
        --taps testdata/resampler_65_48/taps_realtime_minorder.txt \
        --profile realtime --json /tmp/verify.json --plot /tmp/verify.png
"""
import argparse
import hashlib
import json
import os
import sys

import numpy as np

R_IN = 737.28e6
R_OUT = 998.4e6
L, M = 65, 48
FV = L * R_IN                      # 47.9232 GHz design rate

PASSBAND_MHZ = {"realtime": 290.0, "quality": 330.0}
RIPPLE_DB = 0.1
REL_STOP_DB = -70.0
DC_TARGET = float(L)
DC_TOL = 1e-3


# --------------------------------------------------------------- helpers ---

def _jsonable(o):
    """numpy scalars appear anywhere a comparison result is stored."""
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"not JSON serialisable: {type(o).__name__}")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_taps(path):
    """Load the float32 binary exactly as the runtime does."""
    raw = np.fromfile(path, dtype="<f4")
    if raw.size == 0:
        raise SystemExit(f"{path}: empty tap file")
    return raw


def dtft(taps, freqs_hz):
    """Direct DTFT, double precision, at explicit frequencies (Hz)."""
    h = np.asarray(taps, dtype=np.float64)
    n = np.arange(h.size, dtype=np.float64)
    w = -2j * np.pi * np.asarray(freqs_hz, dtype=np.float64)[:, None] * n[None, :] / FV
    return np.exp(w) @ h


def grid_response(taps, nfft):
    """FFT response on [0, Fv/2] with nfft points (rfft)."""
    h = np.asarray(taps, dtype=np.float64)
    H = np.fft.rfft(h, nfft)
    f = np.arange(H.size, dtype=np.float64) * (FV / nfft)
    return f, np.abs(H)


def refine_extremum(taps, f_center, half_width, maximize, iters=80):
    """Golden-section refinement of |H| around f_center.

    The FFT grid can only bracket an extremum; the reported worst value must
    come from a continuous search, not from a grid point.
    """
    gr = (np.sqrt(5.0) - 1.0) / 2.0
    a = max(0.0, f_center - half_width)
    b = min(FV / 2.0, f_center + half_width)
    if b <= a:
        return f_center, float(np.abs(dtft(taps, [f_center])[0]))
    sign = 1.0 if maximize else -1.0
    c = b - gr * (b - a)
    d = a + gr * (b - a)
    fc = sign * float(np.abs(dtft(taps, [c])[0]))
    fd = sign * float(np.abs(dtft(taps, [d])[0]))
    for _ in range(iters):
        if fc > fd:
            b, d, fd = d, c, fc
            c = b - gr * (b - a)
            fc = sign * float(np.abs(dtft(taps, [c])[0]))
        else:
            a, c, fc = c, d, fd
            d = a + gr * (b - a)
            fd = sign * float(np.abs(dtft(taps, [d])[0]))
    f_best = 0.5 * (a + b)
    return f_best, float(np.abs(dtft(taps, [f_best])[0]))


def band_extrema(taps, f_lo, f_hi, f_grid, mag, n_extrema=40):
    """Locate the strongest local extrema of |H| inside [f_lo, f_hi]."""
    sel = (f_grid >= f_lo) & (f_grid <= f_hi)
    idx = np.flatnonzero(sel)
    if idx.size < 3:
        return []
    m = mag[idx]
    out = []
    # local maxima
    for kind, mask in (("max", (m[1:-1] >= m[:-2]) & (m[1:-1] >= m[2:])),
                       ("min", (m[1:-1] <= m[:-2]) & (m[1:-1] <= m[2:]))):
        loc = np.flatnonzero(mask) + 1
        if loc.size == 0:
            continue
        order = np.argsort(m[loc])
        pick = loc[order[-n_extrema:]] if kind == "max" else loc[order[:n_extrema]]
        for j in pick:
            out.append((kind, f_grid[idx[j]]))
    return out


# ------------------------------------------------------------- the check ---

def verify(path, profile, nfft=1 << 21, plot=None, verbose=True):
    taps32 = load_taps(path)
    T = int(taps32.size)
    pb_hz = PASSBAND_MHZ[profile] * 1e6
    sb_hz = R_IN - pb_hz

    res = {
        "file": path,
        "sha256": sha256_file(path),
        "profile": profile,
        "taps": T,
        "passband_mhz": pb_hz / 1e6,
        "stopband_start_mhz": sb_hz / 1e6,
        "design_rate_hz": FV,
        "spec": {"ripple_db_max": RIPPLE_DB,
                 "rel_stop_db_max": REL_STOP_DB,
                 "dc_target": DC_TARGET, "dc_tol": DC_TOL},
    }

    # --- structural checks on the float32 vector as loaded ---------------
    h64 = taps32.astype(np.float64)
    finite = bool(np.all(np.isfinite(taps32)))
    symmetric = bool(np.array_equal(taps32, taps32[::-1]))
    dc = float(h64.sum())
    res["structural"] = {
        "all_finite": bool(finite),
        "symmetric_exact": bool(symmetric),
        "odd_length": bool(T % 2 == 1),
        "dc_sum_float64": dc,
        "dc_error": dc - DC_TARGET,
        "group_delay_out_samples": (T - 1) / (2.0 * M),
        "group_delay_ns": (T - 1) / (2.0 * FV) * 1e9,
    }

    # --- dense grid, two resolutions, to show the worst value converges ---
    f1, m1 = grid_response(taps32, nfft)
    f2, m2 = grid_response(taps32, 2 * nfft)
    res["grid"] = {
        "nfft": nfft,
        "step_mhz": FV / nfft / 1e6,
        "required_step_mhz_max": FV / (64.0 * T) / 1e6,
        "step_ratio": (FV / (64.0 * T)) / (FV / nfft),
    }

    def band_stats(f, mag):
        pb = f <= pb_hz
        sb = f >= sb_hz
        return mag[pb], mag[sb]

    pb1, sb1 = band_stats(f1, m1)
    pb2, sb2 = band_stats(f2, m2)

    # --- explicit critical frequencies, exact DTFT -----------------------
    crit = [0.0, pb_hz, sb_hz, FV / 2.0]
    crit_mag = np.abs(dtft(taps32, crit))

    # --- refine the extrema that the grid bracketed ----------------------
    half = 2.0 * (FV / nfft)
    pb_ref = []
    for kind, fc in band_extrema(taps32, 0.0, pb_hz, f1, m1):
        f, v = refine_extremum(taps32, fc, half, maximize=(kind == "max"))
        pb_ref.append((kind, f, v))
    sb_ref = []
    for kind, fc in band_extrema(taps32, sb_hz, FV / 2.0, f1, m1):
        f, v = refine_extremum(taps32, fc, half, maximize=(kind == "max"))
        sb_ref.append((kind, f, v))

    pb_max = max([v for k, _, v in pb_ref if k == "max"] + [pb1.max(), pb2.max()])
    pb_min = min([v for k, _, v in pb_ref if k == "min"] + [pb1.min(), pb2.min()])
    sb_max = max([v for k, _, v in sb_ref if k == "max"] + [sb1.max(), sb2.max()])

    # include the exact critical frequencies in the extremes
    pb_max = max(pb_max, crit_mag[0], crit_mag[1])
    pb_min = min(pb_min, crit_mag[0], crit_mag[1])
    sb_max = max(sb_max, crit_mag[2], crit_mag[3])

    ripple = 20.0 * np.log10(pb_max / pb_min)
    rel_min = 20.0 * np.log10(sb_max / pb_min)
    rel_max = 20.0 * np.log10(sb_max / pb_max)

    # --- convergence of the worst values under grid doubling -------------
    conv = {
        "ripple_grid1": 20 * np.log10(pb1.max() / pb1.min()),
        "ripple_grid2": 20 * np.log10(pb2.max() / pb2.min()),
        "rel_stop_grid1": 20 * np.log10(sb1.max() / pb1.min()),
        "rel_stop_grid2": 20 * np.log10(sb2.max() / pb2.min()),
    }
    conv["ripple_delta"] = abs(conv["ripple_grid2"] - conv["ripple_grid1"])
    conv["rel_stop_delta"] = abs(conv["rel_stop_grid2"] - conv["rel_stop_grid1"])

    # --- transition band peak and time-domain ringing --------------------
    trans = (f1 > pb_hz) & (f1 < sb_hz)
    trans_peak_db = (20 * np.log10(m1[trans].max() / pb_min)) if trans.any() else None

    # Ringing: peak sidelobe of the impulse response.  The main lobe is the
    # contiguous run around the peak that ends at the first local minimum on
    # each side; everything outside it is sidelobe.  (A fixed +-2 window would
    # just re-measure the main lobe itself.)
    ah = np.abs(h64)
    peak_idx = int(np.argmax(ah))
    lo = peak_idx
    while lo > 0 and ah[lo - 1] < ah[lo]:
        lo -= 1
    hi = peak_idx
    while hi < ah.size - 1 and ah[hi + 1] < ah[hi]:
        hi += 1
    outside = np.concatenate([ah[:lo], ah[hi + 1:]])
    ringing_db = (20 * np.log10(outside.max() / ah[peak_idx])
                  if outside.size and outside.max() > 0 else None)
    main_lobe_taps = hi - lo + 1

    res["response"] = {
        "passband_max_abs": float(pb_max),
        "passband_min_abs": float(pb_min),
        "stopband_max_abs": float(sb_max),
        "passband_gain_db": 20 * np.log10(pb_max),
        "ripple_db": float(ripple),
        "rel_stop_vs_pass_min_db": float(rel_min),
        "rel_stop_vs_pass_max_db": float(rel_max),
        "stopband_worst_freq_mhz": None,
        "passband_max_freq_mhz": None,
        "passband_min_freq_mhz": None,
        "transition_peak_db": None if trans_peak_db is None else float(trans_peak_db),
        "time_ringing_db": None if ringing_db is None else float(ringing_db),
        "main_lobe_taps": int(main_lobe_taps),
    }
    if sb_ref:
        k, f, v = max(sb_ref, key=lambda t: t[2])
        res["response"]["stopband_worst_freq_mhz"] = f / 1e6
    if pb_ref:
        k, f, v = max([t for t in pb_ref if t[0] == "max"], key=lambda t: t[2])
        res["response"]["passband_max_freq_mhz"] = f / 1e6
        k, f, v = min([t for t in pb_ref if t[0] == "min"], key=lambda t: t[2])
        res["response"]["passband_min_freq_mhz"] = f / 1e6

    res["convergence"] = conv

    # --- verdict ---------------------------------------------------------
    checks = {
        "finite": bool(finite),
        "symmetric": bool(symmetric),
        "odd_length": bool(T % 2 == 1),
        "dc": abs(dc - DC_TARGET) < DC_TOL,
        "ripple": ripple <= RIPPLE_DB,
        "rel_stopband": rel_min <= REL_STOP_DB,
    }
    res["checks"] = checks
    res["pass"] = all(checks.values())
    res["margin"] = {
        "ripple_db": RIPPLE_DB - float(ripple),
        "rel_stop_db": REL_STOP_DB - float(rel_min),
        "dc": DC_TOL - abs(dc - DC_TARGET),
    }

    if verbose:
        r = res["response"]
        print(f"[{profile}] {os.path.basename(path)}  T={T}  "
              f"sha256={res['sha256'][:16]}...")
        print(f"  ripple      {r['ripple_db']:.5f} dB  (limit {RIPPLE_DB})"
              f"   margin {res['margin']['ripple_db']:+.5f} dB")
        print(f"  rel stopband {r['rel_stop_vs_pass_min_db']:.3f} dB "
              f"(limit {REL_STOP_DB})  margin {res['margin']['rel_stop_db']:+.3f} dB"
              f"   [vs passband max: {r['rel_stop_vs_pass_max_db']:.3f} dB]")
        print(f"  worst stop @ {r['stopband_worst_freq_mhz']} MHz, "
              f"transition peak {r['transition_peak_db']:.2f} dB, "
              f"sidelobe {r['time_ringing_db']:.2f} dB "
              f"(main lobe {r['main_lobe_taps']} taps)")
        print(f"  dc={res['structural']['dc_sum_float64']:.9f} "
              f"(err {res['structural']['dc_error']:+.2e})  "
              f"group delay {res['structural']['group_delay_out_samples']:.4f} out-samples")
        print(f"  grid step {res['grid']['step_mhz']:.5f} MHz "
              f"({res['grid']['step_ratio']:.1f}x finer than required); "
              f"doubling delta ripple {conv['ripple_delta']:.2e} dB, "
              f"stop {conv['rel_stop_delta']:.2e} dB")
        print(f"  => {'PASS' if res['pass'] else 'FAIL'}  "
              f"{[k for k, v in checks.items() if not v] or ''}")

    if plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(3, 1, figsize=(11, 10))
            db = 20 * np.log10(np.maximum(m2, 1e-30))
            ax[0].plot(f2 / 1e6, db, lw=0.6)
            ax[0].axvline(pb_hz / 1e6, color="g", ls="--", lw=0.8, label="passband edge")
            ax[0].axvline(sb_hz / 1e6, color="r", ls="--", lw=0.8, label="stopband edge")
            ax[0].set_xlim(0, FV / 2e6)
            ax[0].set_xlabel("MHz")
            ax[0].set_ylabel("|H| dB")
            ax[0].set_title(f"{profile} T={T} full band")
            ax[0].legend(loc="upper right", fontsize=8)
            ax[0].grid(alpha=0.3)

            m = f2 <= pb_hz
            ax[1].plot(f2[m] / 1e6, db[m], lw=0.9)
            ax[1].set_xlabel("MHz")
            ax[1].set_ylabel("|H| dB")
            ax[1].set_title(f"passband (ripple {ripple:.5f} dB)")
            ax[1].grid(alpha=0.3)

            m = f2 >= sb_hz
            ax[2].plot(f2[m] / 1e6, db[m], lw=0.6)
            ax[2].set_xlabel("MHz")
            ax[2].set_ylabel("|H| dB")
            ax[2].set_title(f"stopband (rel {rel_min:.2f} dB)")
            ax[2].grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(plot, dpi=110)
            print(f"  wrote {plot}")
        except Exception as e:      # plotting must never gate the verdict
            print(f"  (plot skipped: {e})")

    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--taps", required=True, help="float32 binary tap file")
    ap.add_argument("--profile", required=True, choices=sorted(PASSBAND_MHZ))
    ap.add_argument("--nfft", type=int, default=1 << 21)
    ap.add_argument("--json", default=None)
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()

    res = verify(args.taps, args.profile, nfft=args.nfft, plot=args.plot)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(res, f, indent=2, default=_jsonable)
        print(f"  wrote {args.json}")
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
