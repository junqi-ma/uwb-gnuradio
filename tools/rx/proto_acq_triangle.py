#!/usr/bin/env python3
"""Low-SNR lock-in study: candidate acquisition on the 998.4 MS/s WORK grid.

Scope
-----
This is only the coarse candidate-acquisition stage.  Its single job is to
produce a packet-time candidate good enough for the following fold +
cross-correlation stage, which recovers the exact residual for any offset
inside +-508 work samples (half a SYNC symbol) and slips by one symbol (1016)
outside it.  Timing / SFD / FCS are out of scope.

Primary table (work grid, 998.4 MS/s, SYNC period exactly 1016):
    lag = 1016, span = 260096 (256 SYMBOLS), block/step = 64,
    triangle-peak trigger, low = 9e-5, peak = 1.8e-4, persist = 12000
    plus a lowered-threshold pre-filter variant (low 2e-5 / peak 4e-5).

The earlier native-grid (737.28 MS/s, fractional 750.5-sample period) rows are
kept under `native_legacy` for traceability; they are NOT the primary table.

Two metric definitions are implemented, and the difference is the point of
修改 A:

* `decimated` (production today, native grid).  The C++ ring holds `span/step`
  entries, each the product of ONE sample pair, so the sum runs over
  `span/step` positions spaced `step` apart:

      metric[j] = |sum_{m=j}^{j+L-1} x[s*m] * conj(x[s*m+lag])|^2
                  / max(1, sum|x[s*m]|^2 * sum|x[s*m+lag]|^2),   L = span/s

* `contiguous` (修改 A).  The ring holds `span/step` entries, each the sum of
  `step` CONSECUTIVE sample pairs, so the sum runs over all `span` samples
  (mathematically identical to `step = 1`) while the metric is still only
  evaluated every `step` samples.

Noise convention (matters for the margin): the in-band SNR is defined over
500 MHz, and the fixture puts white noise over the WHOLE grid bandwidth.  On
the work grid that is 998.4 MHz, i.e. 2.0x the in-band noise power, so the
full-band SNR is 3.0 dB below the in-band SNR.  A real 48/65 resampler output
is band-limited to +-368.64 MHz (737.28 MHz wide), so this fixture is 1.32 dB
(2.6 dB of metric) more pessimistic than a real resampler output.

Usage
-----
    python3 tools/rx/proto_acq_triangle.py                     # work grid
    python3 tools/rx/proto_acq_triangle.py --grid both
    python3 tools/rx/proto_acq_triangle.py --snr -16 --config work
    python3 tools/rx/proto_acq_triangle.py --json out.json
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

# --- configurations ---------------------------------------------------------
# `grid` selects the fixture generator and the tolerance set.
CONFIGS = {
    # ---- primary: 998.4 MS/s work grid, SYNC period exactly 1016 ----------
    "work": dict(
        grid="work", span=260096, step=64, lag=1016, metric="contiguous",
        low=9e-5, peak=1.8e-4, persist=12000, gap=30000, holdoff=589824,
        mode="triangle_peak", fall_frac=0.6, fall_after=16384,
        tight_tol=508,
    ),
    # Same metric, thresholds lowered so the stage can only be a pre-filter:
    # peak_thr sits BELOW the pure-noise max of this metric, so noise alone can
    # satisfy it.  Included to show what the margin costs.
    "work_prefilter": dict(
        grid="work", span=260096, step=64, lag=1016, metric="contiguous",
        low=2e-5, peak=4e-5, persist=12000, gap=30000, holdoff=589824,
        mode="triangle_peak", fall_frac=0.6, fall_after=16384,
        tight_tol=508,
    ),
    # Same metric and thresholds, but the seed is the CENTRE of the metric's
    # flat top (peak_at + 508 = half a work symbol).  The top is exactly one
    # symbol wide because the delayed product is coherent on [start,
    # start+span-lag) and any window start in [start-lag, start] contains that
    # whole region, so the raw argmax is uniform over one symbol.
    "work_centre": dict(
        grid="work", span=260096, step=64, lag=1016, metric="contiguous",
        low=9e-5, peak=1.8e-4, persist=12000, gap=30000, holdoff=589824,
        mode="triangle_peak", fall_frac=0.6, fall_after=16384,
        seed_shift=508, tight_tol=508,
    ),
    # ---- legacy: 737.28 MS/s native grid, fractional 750.5-sample period --
    "current": dict(
        grid="native", span=64000, step=8, lag=750, metric="decimated",
        low=0.0015, peak=0.0028, persist=12000, gap=30000, holdoff=589824,
        mode="leading_edge", seed_shift=32000, tight_tol=8000,
    ),
    "proposed": dict(
        grid="native", span=192000, step=64, lag=750, metric="contiguous",
        low=9e-5, peak=1.8e-4, persist=12000, gap=30000, holdoff=589824,
        mode="triangle_peak", fall_frac=0.6, fall_after=16384, tight_tol=8000,
    ),
    "prefilter": dict(
        grid="native", span=192000, step=64, lag=750, metric="contiguous",
        low=2e-5, peak=4e-5, persist=12000, gap=30000, holdoff=589824,
        mode="triangle_peak", fall_frac=0.6, fall_after=16384, tight_tol=8000,
    ),
}

GRID_CONFIGS = {
    "work": ["work", "work_centre", "work_prefilter"],
    "native": ["current", "proposed", "prefilter"],
}
GRID_FS = {"work": sig.FS_WORK, "native": sig.FS_NATIVE}

ASSOC_TOL = 36864      # outer association window (a seed outside is a false alarm)
PERIOD_MS = 1.0


# ---------------------------------------------------------------------------
# metric
# ---------------------------------------------------------------------------
def metric_series(x: np.ndarray, span: int, step: int, lag: int,
                  mode: str = "decimated",
                  chunk_evals: int = 100_000) -> np.ndarray:
    """Normalised delayed autocorrelation on the `step` grid.

    Returns `metric[j]` for `at = step*j`, `j = 0 .. M-L` where
    `M = (len(x)-lag)//step` and `L = span//step`.  Chunked so that peak
    memory stays O(chunk*step + span) instead of O(len(x)).
    """
    L = span // step
    M = (x.size - lag) // step
    if M < L:
        return np.empty(0, dtype=np.float64)
    nj = M - L + 1
    out = np.empty(nj, dtype=np.float64)
    for j0 in range(0, nj, chunk_evals):
        j1 = min(nj, j0 + chunk_evals)
        n = j1 - j0
        p0 = step * j0
        p1 = step * (j1 + L - 1)         # exclusive; n+L-1 blocks of `step`
        a = x[p0:p1]
        b = x[p0 + lag:p1 + lag]
        prod = a * np.conj(b)
        e1 = a.real * a.real + a.imag * a.imag
        e2 = b.real * b.real + b.imag * b.imag
        del a, b
        if mode == "contiguous":
            nb = (p1 - p0) // step
            prod = prod[:nb * step].reshape(nb, step).sum(axis=1,
                                                          dtype=np.complex128)
            e1 = e1[:nb * step].reshape(nb, step).sum(axis=1, dtype=np.float64)
            e2 = e2[:nb * step].reshape(nb, step).sum(axis=1, dtype=np.float64)
        else:
            prod = prod[::step]
            e1 = e1[::step]
            e2 = e2[::step]
        re = np.cumsum(prod.real, dtype=np.float64)
        im = np.cumsum(prod.imag, dtype=np.float64)
        s1 = np.cumsum(e1, dtype=np.float64)
        s2 = np.cumsum(e2, dtype=np.float64)
        del prod, e1, e2

        def win(c):
            w = c[L - 1: L - 1 + n].copy()
            if n > 1:
                w[1:] -= c[:n - 1]
            return w

        wr, wi = win(re), win(im)
        w1, w2 = win(s1), win(s2)
        out[j0:j1] = (wr * wr + wi * wi) / np.maximum(1.0, w1 * w2)
    return out


# ---------------------------------------------------------------------------
# state machine (line-for-line semantics of Acquisition::scan + 修改 A)
# ---------------------------------------------------------------------------
def state_machine(metric: np.ndarray, cfg: dict, diag: list = None) -> list:
    """Run the acquisition trigger over `metric`; return committed seeds.

    Cluster/gap handling, the persistence gate, the peak gate and the holdoff
    are identical to the C++ ring-buffer loop.  The only difference between the
    two trigger modes is where inside the cluster the commit happens and what
    is reported as the seed.  When `diag` is a list, one trigger label per
    committed seed is appended ("persist" for the leading-edge config,
    "fall" / "timeout" for the triangle-peak config).
    """
    step = cfg["step"]
    low, peak_thr = cfg["low"], cfg["peak"]
    persist, gap, holdoff_len = cfg["persist"], cfg["gap"], cfg["holdoff"]
    seeds: list = []
    if metric.size == 0:
        return seeds
    idx = np.flatnonzero(metric >= low)
    if idx.size == 0:
        return seeds
    # A cluster survives a dip below `low` as long as the gap is <= gap_max.
    brk = np.flatnonzero(np.diff(idx) * step > gap)
    cstart = np.concatenate(([idx[0]], idx[brk + 1]))
    cend = np.concatenate((idx[brk], [idx[-1]]))       # inclusive
    holdoff = 0
    for cs, ce in zip(cstart, cend):
        pos = int(cs)
        while pos <= ce:
            if step * pos < holdoff:                   # production: `at < holdoff`
                pos = max(pos, -(-holdoff // step))
                if pos > ce:
                    break
            seg = metric[pos:ce + 1]
            ats = step * np.arange(pos, ce + 1, dtype=np.int64)
            runmax = np.maximum.accumulate(seg)
            if cfg["mode"] == "leading_edge":
                ok = (ats - ats[0] >= persist) & (runmax > peak_thr)
                if not ok.any():
                    break
                k = int(np.argmax(ok))
                seed = int(ats[0] + cfg["seed_shift"])
                if diag is not None:
                    diag.append("persist")
            else:
                # running argmax (first occurrence of the running maximum)
                prev = np.concatenate(([-np.inf], runmax[:-1]))
                rec = np.flatnonzero(seg > prev)
                ridx = np.searchsorted(rec, np.arange(seg.size), side="right") - 1
                peakv = runmax[rec[ridx]]
                peak_at = ats[0] + step * rec[ridx]
                fell = seg <= cfg["fall_frac"] * peakv
                timed = ats - peak_at >= cfg["fall_after"]
                ok = ((ats - ats[0] >= persist)
                      & (peakv > peak_thr)
                      & (fell | timed))
                if not ok.any():
                    break
                k = int(np.argmax(ok))
                seed = int(peak_at[k]) + int(cfg.get("seed_shift", 0))
                if diag is not None:
                    diag.append("fall" if fell[k] else "timeout")
            seeds.append(seed)
            holdoff = seed + holdoff_len
            pos = pos + k + 1
    return seeds


# ---------------------------------------------------------------------------
# association / statistics
# ---------------------------------------------------------------------------
def associate(seeds, starts, tol=ASSOC_TOL, tight=None):
    seeds = np.asarray(seeds, dtype=np.int64)
    starts = np.asarray(starts, dtype=np.int64)
    res = dict(n_seeds=int(seeds.size), detected=0, detected_slots=0,
               ambiguous=0, false_alarms=0, errs=[], within_tight=0)
    if seeds.size == 0 or starts.size == 0:
        res["false_alarms"] = int(seeds.size)
        return res
    lo = np.searchsorted(seeds, starts - tol, side="left")
    hi = np.searchsorted(seeds, starts + tol, side="right")
    cnt = hi - lo
    res["detected"] = int((cnt == 1).sum())
    res["ambiguous"] = int((cnt > 1).sum())
    res["detected_slots"] = int(starts.size)
    res["errs"] = [int(seeds[lo[i]] - starts[i]) for i in np.flatnonzero(cnt == 1)]
    d = np.abs(seeds[:, None] - starts[None, :]).min(axis=1)
    res["false_alarms"] = int((d > tol).sum())
    if tight is not None:
        lo2 = np.searchsorted(seeds, starts - tight, side="left")
        hi2 = np.searchsorted(seeds, starts + tight, side="right")
        res["within_tight"] = int(((hi2 - lo2) == 1).sum())
    return res


def err_stats(errs, tight=None):
    if not errs:
        return dict(n=0)
    a = np.abs(np.asarray(errs, dtype=np.int64))
    d = dict(n=int(a.size), min=int(a.min()), median=float(np.median(a)),
             p95=float(np.percentile(a, 95)), max=int(a.max()),
             signed_min=int(np.min(errs)), signed_max=int(np.max(errs)))
    if tight is not None:
        d["tight_tol"] = int(tight)
        d["within_tight"] = int((a <= tight).sum())
    return d


def signal_peak_stats(metric, starts, step, half):
    """Per-packet signal metric.

    `max/min/...` are the max inside the triangle (+-half samples): that is
    what the detector sees.  `at_start_*` samples the metric at the true packet
    start, which is a single draw of a wide distribution at low SNR (the
    correlation SNR is only ~3 dB at -16 dB), so it is reported for reference
    only.
    """
    if metric.size == 0 or not starts:
        return dict(n=0)
    vals, ats = [], []
    for s in starts:
        j0 = max(0, (s - half) // step)
        j1 = min(metric.size, (s + half) // step + 1)
        if j1 > j0:
            vals.append(float(metric[j0:j1].max()))
            ats.append(float(metric[min(metric.size - 1, max(0, s // step))]))
    if not vals:
        return dict(n=0)
    a = np.asarray(vals)
    b = np.asarray(ats)
    return dict(n=int(a.size), min=float(a.min()), mean=float(a.mean()),
                median=float(np.median(a)), p95=float(np.percentile(a, 95)),
                max=float(a.max()),
                at_start_min=float(b.min()), at_start_median=float(np.median(b)),
                at_start_max=float(b.max()))


def metric_dist(metric, same_window_evals=None):
    """Distribution of the metric over a noise-only stretch."""
    if metric.size == 0:
        return dict(n=0)
    d = dict(n=int(metric.size), mean=float(metric.mean()),
             p99=float(np.percentile(metric, 99)), max=float(metric.max()))
    if same_window_evals:
        k = min(int(same_window_evals), metric.size)
        d["max_same_windows"] = float(metric[:k].max())
        d["p99_same_windows"] = float(np.percentile(metric[:k], 99))
        d["n_same_windows"] = int(k)
    return d


def run_on_metric(metric, cfg, starts, label, samples, metric_seconds=None):
    t0 = time.perf_counter()
    diag: list = []
    seeds = state_machine(metric, cfg, diag)
    t_sm = time.perf_counter() - t0
    tight = cfg.get("tight_tol")
    assoc = associate(seeds, starts, tight=tight)
    out = dict(
        label=label, config=cfg, grid=cfg["grid"], samples=int(samples),
        n_metric_evals=int(metric.size),
        metric_max=float(metric.max()) if metric.size else None,
        metric_mean=float(metric.mean()) if metric.size else None,
        seeds=seeds,
        triggers=diag,
        trigger_counts={t: diag.count(t) for t in sorted(set(diag))},
        n_seeds=len(seeds),
        detected=assoc["detected"],
        detected_slots=assoc["detected_slots"],
        ambiguous=assoc["ambiguous"],
        false_alarms=assoc["false_alarms"],
        within_tight=assoc["within_tight"],
        err=err_stats(assoc["errs"], tight),
        metric_seconds=metric_seconds,
        state_machine_seconds=round(t_sm, 3),
    )
    if starts:
        out["signal_peak"] = signal_peak_stats(
            metric, starts, cfg["step"], cfg["span"] // 2)
        # packet 0 starts at sample 0, so its leading edge is truncated by the
        # start of the stream (impossible in a live stream).  Report the
        # warm-up-excluded view as well: packets 1..N-1 and their seeds.
        keep = [s for s in seeds if s >= starts[1] - ASSOC_TOL]
        a2 = associate(keep, starts[1:], tight=tight)
        out["warmup_excluded"] = dict(
            detected=a2["detected"], detected_slots=a2["detected_slots"],
            false_alarms=a2["false_alarms"], within_tight=a2["within_tight"],
            err=err_stats(a2["errs"], tight))
    return out


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------
def make_noise(seconds: float, fs: float, seed: int = 1, rms_lsb: float = 542.0,
               snr_ref_db: float = -14.0, chunk: int = 8_000_000) -> np.ndarray:
    """Chunked pure AWGN at the same power as a `snr_ref_db` stream.

    Same construction/power as the shared generator (see
    `proto_low_snr_signal.noise_power_for` / `make_stream_work`), generated in
    chunks only to bound peak memory for the 0.5 s case.
    """
    n = int(round(seconds * fs))
    x = np.empty(n, dtype=np.complex64)
    p = (rms_lsb ** 2) * (fs / sig.INBAND_HZ) / (10.0 ** (snr_ref_db / 10.0))
    rng = np.random.default_rng(seed)
    for s in range(0, n, chunk):
        e = min(n, s + chunk)
        m = e - s
        v = rng.standard_normal(m) + 1j * rng.standard_normal(m)
        v *= np.sqrt(p / 2.0)
        x[s:e] = v
    return x


def make_stream(grid, snr, packets, seed):
    if grid == "work":
        return sig.make_stream_work(snr_db=snr, packets=packets, seed=seed)
    return sig.make_stream(snr_db=snr, packets=packets, seed=seed)


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", choices=["work", "native", "both"], default="work")
    ap.add_argument("--snr", default="10,0,-10,-14,-16,-18",
                    help="comma list of in-band SNR in dB")
    ap.add_argument("--packets", type=int, default=100)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--config", default=None,
                    help="comma list of config names (default: all for the grid)")
    ap.add_argument("--noise-seconds", type=float, default=0.5)
    ap.add_argument("--skip-noise", action="store_true")
    ap.add_argument("--fall-after", type=int, default=None,
                    help="override the triangle-peak timeout (samples); "
                         "e.g. 100000000 for a fall-only trigger")
    ap.add_argument("--seed-shift", type=int, default=None,
                    help="add this to every triangle-peak seed (e.g. 508 = half "
                         "a work-grid symbol, the centre of the flat top)")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)

    snrs = [float(s) for s in a.snr.split(",") if s.strip()]
    grids = ["work", "native"] if a.grid == "both" else [a.grid]
    if a.fall_after is not None:
        for c in CONFIGS.values():
            if c["mode"] == "triangle_peak":
                c["fall_after"] = a.fall_after
    if a.seed_shift is not None:
        for c in CONFIGS.values():
            if c["mode"] == "triangle_peak":
                c["seed_shift"] = a.seed_shift
    names = []
    for g in grids:
        names += ([n for n in a.config.split(",") if n.strip()]
                  if a.config else GRID_CONFIGS[g])
    names = [n for n in names if n in CONFIGS]
    # configs that share a metric signature share one (expensive) metric pass
    groups = {}
    for name in names:
        c = CONFIGS[name]
        groups.setdefault((c["grid"], c["span"], c["step"], c["lag"],
                           c["metric"]), []).append(name)

    t_start = time.perf_counter()
    report = dict(
        generated=time.strftime("%Y-%m-%dT%H:%M:%S"),
        primary_grid="work",
        fixture=("tools/rx/proto_low_snr_signal.py: make_stream_work (998.4 MS/s) "
                 "for the primary table, make_stream (737.28 MS/s) for "
                 "native_legacy"),
        note=("synthetic AWGN only; no UHD/multipath/CFO/SFO/interference. "
              "Coarse stage only: the target is a candidate within +-508 work "
              "samples for the following fold+cross-correlation stage. "
              "Noise is white over the whole grid band, so the work-grid "
              "fixture is 1.32 dB (2.6 dB of metric) more pessimistic than a "
              "real 48/65 resampler output, whose noise stops at +-368.64 MHz. "
              "packet 0 is reported both ways: its leading edge is truncated "
              "by the start of the stream."),
        assoc_tol=ASSOC_TOL, packets=a.packets, noise_seed=a.seed + 7,
        runs=[], noise=[], margin_table=[], native_legacy=[], scaling={},
    )

    for grid in grids:
        fs = GRID_FS[grid]
        for snr in snrs:
            t0 = time.perf_counter()
            x, meta = make_stream(grid, snr, a.packets, a.seed)
            gen = round(time.perf_counter() - t0, 2)
            starts = meta["packet_starts"]
            print(f"[{grid} {snr:+.1f} dB] {x.size} samples "
                  f"({x.size / fs * 1e3:.1f} ms, gen {gen}s)", flush=True)
            for key, group in groups.items():
                if key[0] != grid:
                    continue
                t0 = time.perf_counter()
                metric = metric_series(x, key[1], key[2], key[3], mode=key[4],
                                       chunk_evals=100_000
                                       if key[4] == "contiguous" else 2_000_000)
                t_metric = round(time.perf_counter() - t0, 3)
                for name in group:
                    r = run_on_metric(metric, CONFIGS[name], starts,
                                      f"{name}@{snr:+.1f}dB", x.size, t_metric)
                    r["snr_db"] = snr
                    r["config_name"] = name
                    r["gen_seconds"] = gen
                    (report["runs"] if grid == "work"
                     else report["native_legacy"]).append(r)
                    e = r["err"]
                    sp = r.get("signal_peak", {})
                    print(f"   {name:15s} det={r['detected']}/{r['detected_slots']} "
                          f"fa={r['false_alarms']} "
                          f"<={CONFIGS[name]['tight_tol']}: {r['within_tight']}"
                          f"/{r['detected_slots']} "
                          f"|err| med={e.get('median')} p95={e.get('p95')} "
                          f"max={e.get('max')} "
                          f"sig med={sp.get('median', 0):.2e} "
                          f"min={sp.get('min', 0):.2e}", flush=True)
                del metric
            del x

        if a.skip_noise:
            continue
        t0 = time.perf_counter()
        x = make_noise(a.noise_seconds, fs, seed=a.seed + 7)
        gen = round(time.perf_counter() - t0, 2)
        print(f"[{grid} noise] {x.size} samples ({a.noise_seconds} s, gen {gen}s)",
              flush=True)
        same = a.packets * int(round(PERIOD_MS * 1e-3 * fs))
        for key, group in groups.items():
            if key[0] != grid:
                continue
            t0 = time.perf_counter()
            metric = metric_series(x, key[1], key[2], key[3], mode=key[4],
                                   chunk_evals=100_000
                                   if key[4] == "contiguous" else 2_000_000)
            t_metric = round(time.perf_counter() - t0, 3)
            same_evals = max(1, (same - key[3]) // key[2] - key[1] // key[2] + 1)
            dist = metric_dist(metric, same_evals)
            for name in group:
                r = run_on_metric(metric, CONFIGS[name], [], f"{name}@noise",
                                  x.size, t_metric)
                r["noise_seconds"] = a.noise_seconds
                r["config_name"] = name
                r["gen_seconds"] = gen
                r["metric_dist"] = dist
                r["false_alarms_per_second"] = r["false_alarms"] / a.noise_seconds
                r["low_margin"] = (CONFIGS[name]["low"] / dist["max"]
                                   if dist["max"] else None)
                r["peak_margin"] = (CONFIGS[name]["peak"] / dist["max"]
                                    if dist["max"] else None)
                report["noise"].append(r)
                print(f"   {name:15s} fa={r['false_alarms']} "
                      f"({r['false_alarms_per_second']:.1f}/s) "
                      f"noise mean={dist['mean']:.3e} p99={dist['p99']:.3e} "
                      f"max={dist['max']:.3e} "
                      f"max(100ms)={dist['max_same_windows']:.3e} "
                      f"low/thr={CONFIGS[name]['low']/dist['max']:.2f}x "
                      f"peak/thr={CONFIGS[name]['peak']/dist['max']:.2f}x",
                      flush=True)
            del metric
        del x

    # ---- explicit detection margin table (primary grid only) --------------
    noise_by_name = {r["config_name"]: r["metric_dist"] for r in report["noise"]}
    for r in report["runs"]:
        sp = r.get("signal_peak")
        nd = noise_by_name.get(r["config_name"])
        if not sp or not nd or not nd.get("n"):
            continue
        nm = nd.get("max_same_windows") or nd["max"]
        row = dict(
            config=r["config_name"], snr_db=r["snr_db"],
            signal_metric_min=sp["min"], signal_metric_median=sp["median"],
            signal_metric_max=sp["max"],
            noise_mean=nd["mean"], noise_p99=nd["p99"],
            noise_max_0p5s=nd["max"], noise_max_same_windows=nm,
            low=r["config"]["low"], peak_thr=r["config"]["peak"],
            detected=r["detected"], detected_slots=r["detected_slots"],
            within_tight=r["within_tight"],
            tight_tol=r["config"]["tight_tol"],
            false_alarms=r["false_alarms"],
            margin_weakest_packet=sp["min"] / nm,
            margin_median_packet=sp["median"] / nm,
            margin_peak_thr_vs_noise=r["config"]["peak"] / nm,
            margin_low_vs_noise=r["config"]["low"] / nm,
        )
        report["margin_table"].append(row)

    # ---- quadratic (S/(S+N))^2 scaling: how far down can this stage go ----
    # For S/N << 1 the metric peak is rho^2 with rho = S/(S+N), so
    # peak(snr) = peak0 * 10^((snr-snr0)/5): 2 dB of metric per 1 dB of SNR.
    # Anchor on the LOWEST SNR that still detects >= 90% of the packets (stays
    # in the rho << 1 regime); anchoring at +10 dB is wrong because rho^2
    # saturates there.
    for name in [n for n in names if CONFIGS[n]["grid"] == "work"]:
        runs = [r for r in report["runs"] if r["config_name"] == name
                and r.get("signal_peak")]
        anchor = None
        for r in runs:
            if r["detected_slots"] == 0:
                continue
            if r["detected"] < 0.9 * r["detected_slots"]:
                continue
            if anchor is None or r["snr_db"] < anchor["snr_db"]:
                anchor = dict(snr_db=r["snr_db"],
                              med=r["signal_peak"]["median"],
                              mn=r["signal_peak"]["min"])
        if not anchor:
            continue
        nd = noise_by_name.get(name)
        nm = nd.get("max_same_windows") or nd["max"]
        report["scaling"][name] = dict(
            law="metric_peak ~ (S/(S+N))^2 -> 2 dB of metric per 1 dB of SNR",
            anchor=anchor, noise_max_same_windows=nm,
            snr_at_3x_margin_median=round(
                anchor["snr_db"] + 5.0 * np.log10(3.0 * nm / anchor["med"]), 2),
            snr_at_3x_margin_weakest=round(
                anchor["snr_db"] + 5.0 * np.log10(3.0 * nm / anchor["mn"]), 2),
        )

    report["wall_seconds"] = round(time.perf_counter() - t_start, 1)
    report["cpu_estimate"] = dict(
        note=("arithmetic only; the numpy timings above are NOT a C++ benchmark. "
              "Cost model calibrated on the measured frontend_us of the current "
              "native config (300,008.6 us for 0.5 s / 368.64 M samples, "
              "docs/performance/AWGN_SNR仿真与阈值分析_2026-11-17.md L201)."),
        per_sample=dict(complex_product_mults=4, complex_product_adds=2,
                        energy_mults=2, energy_adds=1, accumulator_adds=4,
                        total_mults=6, total_adds=4),
        work_samples_per_0p5s=int(0.5 * sig.FS_WORK),
        work_mults_per_0p5s=6 * int(0.5 * sig.FS_WORK),
        work_metric_evals_per_0p5s=int(0.5 * sig.FS_WORK) // 64,
        calibrated=dict(
            eval_cycles=24.0, product_cycles_per_sample=1.0,
            work_model_ms=round((int(0.5 * sig.FS_WORK) // 64 * 24.0
                                 + 0.5 * sig.FS_WORK * 1.0) / 3.7e9 * 1e3, 1),
            native_current_model_ms=round(
                (46071907 * 24.0 + 46071907 * 1.0) / 3.7e9 * 1e3, 1),
            native_current_measured_ms=300.0),
    )
    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, "w") as f:
            json.dump(report, f, indent=1)
        print(f"wrote {a.json} ({report['wall_seconds']}s total)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
