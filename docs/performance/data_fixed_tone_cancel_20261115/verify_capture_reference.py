#!/usr/bin/env python3
"""Independent fixed-frequency reference + metrics for the 40 ms capture.

This script does NOT call the C++ core.  It re-implements the documented
algorithm (fixed frequency, 288-point period, int32 fold, double projection,
nearest-even tone quantisation, int16 saturating subtract, linear cross-fade)
in numpy and compares the result byte for byte with the stream produced by
`fixed_tone_cancel_replay` (the production decorator).

It also reports the checks the task requires: coherent residual per 1 ms, the
deployed 8-SYMBOL autocorrelation metric, a threshold scan with candidate
groups, and code-10 template correlation / arrival times.  Total power
reduction alone is never reported as success.

Usage:
  python3 verify_capture_reference.py \
      --input testdata/capture_compare_20261115_ch3/rx_ch3_rx1_40ms.sc16 \
      --clean /tmp/ftc/clean_40ms.sc16 \
      --rate 737280000 --freq -232960000 \
      --period-ms 20 --training-ms 0.5 --crossfade 256 \
      --template testdata/reference_preamble_code10_737p28.cf32 \
      --json /tmp/ftc/verify.json
"""

import argparse
import json
import math
import numpy as np


def nearest_even(v):
    """Nearest integer, ties to even, independent of numpy rounding."""
    lo = np.floor(v)
    frac = v - lo
    inc = (frac > 0.5) | ((frac == 0.5) & (np.fmod(lo, 2.0) != 0.0))
    return lo + inc


def build_model(samples, epoch, ntrain, period, omega, prev):
    """Fold + project exactly as documented, then build the int16 tone table."""
    seg = samples[epoch:epoch + ntrain].astype(np.int64).reshape(-1, 2)
    n = seg.shape[0]
    full = (n // period) * period
    fold = np.zeros((period, 2), dtype=np.int64)
    if full:
        fold += seg[:full].reshape(-1, period, 2).sum(axis=0)
    rem = n - full
    if rem:
        fold[:rem] += seg[full:]
    k = np.arange(period)
    cos = np.cos(omega * k)
    sin = np.sin(omega * k)
    re = float(np.sum(fold[:, 0] * cos + fold[:, 1] * sin)) / n
    im = float(np.sum(fold[:, 1] * cos - fold[:, 0] * sin)) / n
    lut = np.empty((period, 2), dtype=np.int16)
    lut[:, 0] = nearest_even(re * cos - im * sin).astype(np.int16)
    lut[:, 1] = nearest_even(re * sin + im * cos).astype(np.int16)
    blend = None
    if prev is not None and prev["crossfade"] > 0:
        c = prev["crossfade"]
        j = np.arange(c)
        a = (j + 1.0) / float(c)
        rel = (epoch + j - prev["epoch"]) % period
        blend = np.empty((c, 2), dtype=np.int16)
        blend[:, 0] = nearest_even((1 - a) * prev["lut"][rel, 0] + a * lut[j % period, 0])
        blend[:, 1] = nearest_even((1 - a) * prev["lut"][rel, 1] + a * lut[j % period, 1])
    return {
        "epoch": epoch,
        "lut": lut,
        "blend": blend,
        "crossfade": 0 if blend is None else blend.shape[0],
        "coef": (re, im),
        "amp": math.hypot(re, im),
        "phase": math.atan2(im, re),
        "ntrain": n,
    }


def apply_models(x, models, np_, crossfade):
    out = x.astype(np.int32).copy()
    period = 288
    n = out.shape[0]
    for idx, m in enumerate(models):
        start = m["epoch"]
        end = models[idx + 1]["epoch"] if idx + 1 < len(models) else np_
        end = min(end, np_)
        if start >= end:
            continue
        idxs = np.arange(start, end)
        tone = np.empty((end - start, 2), dtype=np.int32)
        rel = idxs - start
        if m["blend"] is not None and crossfade > 0:
            c = m["blend"].shape[0]
            head = rel < c
            tone[head] = m["blend"][rel[head]]
            rest = ~head
            tone[rest] = m["lut"][rel[rest] % period]
        else:
            tone = m["lut"][rel % period].astype(np.int32)
        out[start:end] -= tone
    np.clip(out, -32768, 32767, out=out)
    return out.astype(np.int16)


def coherent_amp(x, freq, fs, block):
    """|mean(x[k] exp(-j w k))| per block, in ADC units."""
    n = (x.shape[0] // block) * block
    xb = x[:n].reshape(-1, block, 2)
    k = np.arange(block)
    w = 2 * np.pi * freq / fs
    ph = np.exp(-1j * w * k)
    z = (xb[:, :, 0] + 1j * xb[:, :, 1]) @ ph / block
    return np.abs(z)


def autocorr_metric(x, lag, window):
    """Deployed metric: |sum x[n] conj(x[n+lag])|^2 / (E1 E2) over `window`."""
    z = x[:, 0].astype(np.float64) + 1j * x[:, 1].astype(np.float64)
    m = z.shape[0] - lag
    prod = z[:m] * np.conj(z[lag:])
    cp = np.concatenate(([0], np.cumsum(prod)))
    e1 = np.concatenate(([0], np.cumsum(np.abs(z[:m]) ** 2)))
    e2 = np.concatenate(([0], np.cumsum(np.abs(z[lag:]) ** 2)))
    num = np.abs(cp[window:] - cp[:-window]) ** 2
    den = (e1[window:] - e1[:-window]) * (e2[window:] - e2[:-window])
    return num / np.maximum(den, 1e-30)


def groups(mask, times, gap_s, metric):
    """Contiguous above-threshold groups with their peak time and value."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    splits = np.flatnonzero(np.diff(times[idx]) > gap_s) + 1
    out = []
    for g in np.split(idx, splits):
        best = g[int(np.argmax(metric[g]))]
        out.append(
            {
                "start_ms": float(times[g[0]] * 1e3),
                "end_ms": float(times[g[-1]] * 1e3),
                "peak_ms": float(times[best] * 1e3),
                "peak": float(metric[best]),
            }
        )
    return out


def template_metric(x, tmpl):
    """Normalized |<x, t>|^2 / (|x|^2 |t|^2) for one SYNC template."""
    z = x[:, 0].astype(np.float64) + 1j * x[:, 1].astype(np.float64)
    L = tmpl.shape[0]
    m = z.shape[0] - L + 1
    csum_z = np.concatenate(([0j], np.cumsum(z)))
    csum_e = np.concatenate(([0.0], np.cumsum(np.abs(z) ** 2)))
    # dot[n] = sum_{k<L} z[n+k] conj(t[k])  computed in blocks via FFT.
    nfft = 1 << int(math.ceil(math.log2(m + L)))
    zf = np.fft.fft(z[: m + L - 1], nfft)
    tf = np.fft.fft(np.conj(tmpl[::-1]), nfft)
    dot = np.fft.ifft(zf * tf)[L - 1: m + L - 1]
    e = csum_e[L:] - csum_e[:-L]
    et = float(np.vdot(tmpl, tmpl).real)
    return np.abs(dot) ** 2 / np.maximum(e * et, 1e-30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--clean", required=True)
    ap.add_argument("--rate", type=float, default=737.28e6)
    ap.add_argument("--freq", type=float, default=-232.96e6)
    ap.add_argument("--period-ms", type=float, default=20.0)
    ap.add_argument("--training-ms", type=float, default=0.5)
    ap.add_argument("--crossfade", type=int, default=256)
    ap.add_argument("--template", default="")
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    fs = args.rate
    period = 288
    cycles = int(round(args.freq * period / fs))
    omega = 2 * math.pi * cycles / period
    np_eriod = int(round(args.period_ms * 1e-3 * fs))
    ntrain = int(round(args.training_ms * 1e-3 * fs))
    print(f"fs={fs} f0={args.freq} cycles={cycles} omega={omega}")
    print(f"Np={np_eriod} Nt={ntrain} crossfade={args.crossfade}")

    raw = np.fromfile(args.input, dtype="<i2")
    if raw.size % 2:
        raw = raw[:-1]
    x = raw.reshape(-1, 2)
    n_total = x.shape[0]
    clean = np.fromfile(args.clean, dtype="<i2").reshape(-1, 2)
    print(f"input samples={n_total} clean samples={clean.shape[0]}")
    if clean.shape[0] != n_total:
        raise SystemExit("ERROR: clean length differs from input")

    # --- independent models + output -------------------------------------
    models = []
    prev = None
    for epoch in range(0, n_total - ntrain + 1, np_eriod):
        m = build_model(x, epoch, ntrain, period, omega, prev)
        m["crossfade"] = args.crossfade
        models.append(m)
        prev = m
    print("models:")
    for i, m in enumerate(models):
        print(
            f"  {i+1}: epoch={m['epoch']} amp={m['amp']:.6f} "
            f"phase={m['phase']:.9f} re={m['coef'][0]:.9f} im={m['coef'][1]:.9f}"
        )
    ref = apply_models(x, models, n_total, args.crossfade)

    diff = ref.astype(np.int32) - clean.astype(np.int32)
    n_diff = int(np.count_nonzero(diff))
    max_abs = int(np.abs(diff).max()) if n_diff else 0
    print(f"reference_vs_cpp: differing_components={n_diff} max_abs={max_abs}")
    # Compare coefficients against the models.csv printed by the C++ driver.
    cpp_models = []
    try:
        with open(args.clean + ".models.csv") as f:
            next(f)
            for line in f:
                p = line.strip().split(",")
                cpp_models.append((int(p[1]), float(p[4]), float(p[5])))
    except OSError:
        cpp_models = []

    # --- coherent residual per 1 ms --------------------------------------
    block = int(round(1e-3 * fs))
    before = coherent_amp(x, args.freq, fs, block)
    after = coherent_amp(clean, args.freq, fs, block)
    # exclude the first training window from the suppression table
    first = int(math.ceil(args.training_ms))
    supp = []
    for i in range(first, before.shape[0]):
        db = 20 * math.log10(max(before[i], 1e-12) / max(after[i], 1e-12))
        supp.append({"ms": i, "before": float(before[i]), "after": float(after[i]),
                     "db": float(db)})

    # --- deployed autocorrelation metric --------------------------------
    lag = int(round(1016 * fs / 998.4e6))
    window = 8 * lag
    ac_before = autocorr_metric(x, lag, window)
    ac_after = autocorr_metric(clean, lag, window)
    t_ac = (np.arange(ac_before.size) + window) / fs
    thr_scan = {}
    for thr in (0.05, 0.1, 0.15, 0.2):
        gb = groups(ac_before > thr, t_ac, 20e-6, ac_before)
        ga = groups(ac_after > thr, t_ac, 20e-6, ac_after)
        thr_scan[str(thr)] = {
            "before_groups": len(gb),
            "after_groups": len(ga),
            "after": ga,
        }

    result = {
        "rate": fs,
        "freq": args.freq,
        "cycles_per_period": cycles,
        "period_samples": np_eriod,
        "training_samples": ntrain,
        "crossfade": args.crossfade,
        "models": [
            {
                "epoch": m["epoch"],
                "amp": m["amp"],
                "phase": m["phase"],
                "re": m["coef"][0],
                "im": m["coef"][1],
            }
            for m in models
        ],
        "cpp_models": cpp_models,
        "reference_vs_cpp": {"differing_components": n_diff, "max_abs": max_abs},
        "coherent_1ms": supp,
        "autocorr": {
            "lag_samples": lag,
            "window_samples": window,
            "median_before": float(np.median(ac_before)),
            "median_after": float(np.median(ac_after)),
            "p99_before": float(np.percentile(ac_before, 99)),
            "p99_after": float(np.percentile(ac_after, 99)),
            "max_before": float(ac_before.max()),
            "max_after": float(ac_after.max()),
            "thresholds": thr_scan,
        },
    }

    # --- code-10 template correlation / arrival times --------------------
    if args.template:
        tmpl = np.fromfile(args.template, dtype=np.complex64)
        tb = template_metric(x, tmpl)
        ta = template_metric(clean, tmpl)
        t_t = (np.arange(tb.size) + tmpl.shape[0] - 1) / fs
        out = {}
        for thr in (0.05, 0.1, 0.2):
            gb = groups(tb > thr, t_t, 20e-6, tb)
            ga = groups(ta > thr, t_t, 20e-6, ta)
            out[str(thr)] = {
                "before_groups": len(gb),
                "after_groups": len(ga),
                "after": ga,
                "before": gb,
            }
        # Arrival-time agreement between before/after on the same threshold.
        gb = groups(tb > 0.1, t_t, 20e-6, tb)
        ga = groups(ta > 0.1, t_t, 20e-6, ta)
        delta_us = []
        if len(gb) == len(ga):
            for a, b in zip(ga, gb):
                delta_us.append((a["peak_ms"] - b["peak_ms"]) * 1e3)
        result["template"] = {
            "path": args.template,
            "samples": int(tmpl.shape[0]),
            "max_before": float(tb.max()),
            "max_after": float(ta.max()),
            "p99_before": float(np.percentile(tb, 99)),
            "p99_after": float(np.percentile(ta, 99)),
            "thresholds": out,
            "arrival_delta_us_at_0.1": delta_us,
        }

    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=2)
        print(f"wrote {args.json}")

    print(
        "autocorr median before=%.5f after=%.5f | p99 before=%.5f after=%.5f"
        % (
            result["autocorr"]["median_before"],
            result["autocorr"]["median_after"],
            result["autocorr"]["p99_before"],
            result["autocorr"]["p99_after"],
        )
    )
    for thr, v in result["autocorr"]["thresholds"].items():
        print(f"  autocorr thr={thr}: before={v['before_groups']} groups, "
              f"after={v['after_groups']} groups")
    if "template" in result:
        for thr, v in result["template"]["thresholds"].items():
            print(f"  template thr={thr}: before={v['before_groups']} groups, "
                  f"after={v['after_groups']} groups")
        print("  template arrival delta (us, after-before):",
              ["%.3f" % d for d in result["template"]["arrival_delta_us_at_0.1"]])
        print("  template peaks after (ms, value):",
              ["%.4f/%.3f" % (g["peak_ms"], g["peak"]) for g in
               result["template"]["thresholds"]["0.1"]["after"]])
    print("suppression dB (1 ms blocks, ms: before/after/dB):")
    for row in supp:
        print(
            "  %2d ms: %.3f -> %.3f (%.2f dB)"
            % (row["ms"], row["before"], row["after"], row["db"])
        )


if __name__ == "__main__":
    main()
