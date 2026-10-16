#!/usr/bin/env python3
"""NumPy reference for the S0 preamble-only SIC math core (spec witness).

This module mirrors the FROZEN contract in ``docs/sic/S0_sic_core_api.md`` so the
C++ core (``gr-uwb/.../uwb_preamble_sic_core.{h,cc}``) can be cross-checked
against it.  It is deliberately *simple and readable*, not clever, and it has no
repository imports (``numpy`` only, tested with numpy 1.21.5).

Truth isolation
---------------
``estimate_sic(y, sync, preamble, cfg)`` is the only mode usable for effect /
real-time acceptance.  Its signature has **no truth argument** and it never
reads injected delay / CFO / paths / interferer IQ.  ``oracle_sic(...)`` takes an
explicit ``truth`` struct, sets ``uses_truth=True`` and is an upper-bound
diagnostic only.

Modes
-----
``SicMode``   Off / Estimate / Oracle
``SicState``  Bypassed / Committed / Failed / Ambiguous

Golden vectors
--------------
``--emit-golden`` regenerates ``testdata/sic_preamble/golden/`` deterministically.
The io schema is documented in ``golden/README.md``; every ``io.npz`` holds
complex64 arrays ``y, sync, preamble, i, i_hat_expected, out_expected`` (``i`` is
truth; never pass it to ``estimate_sic``).

Usage
-----
    python3 tools/sic/sic_reference.py --self-test
    python3 tools/sic/sic_reference.py --emit-golden
"""
from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import os
import time

import numpy as np

SCHEMA_VERSION = 1
GOLDEN_SUBDIR = os.path.join("testdata", "sic_preamble", "golden")

# SicTimings fields, same names, same order.
TIMING_NAMES = (
    "detect_coarse", "detect_fine", "cfo", "cir", "path_select",
    "fractional_refine", "matrix_build", "gain_fit", "reconstruct",
    "validate_commit", "core_total",
)

MODE_OFF = "Off"
MODE_ESTIMATE = "Estimate"
MODE_ORACLE = "Oracle"

STATE_BYPASSED = "Bypassed"
STATE_COMMITTED = "Committed"
STATE_FAILED = "Failed"
STATE_AMBIGUOUS = "Ambiguous"


def _us(t0):
    return (time.perf_counter() - t0) * 1e6


# ---------------------------------------------------------------------------
# Config / result plumbing
# ---------------------------------------------------------------------------
def default_config(**over):
    cfg = {
        "mode": MODE_ESTIMATE,
        "comm_code": 10,
        "sync_reps": 16,            # nominal; the true start is unknown
        "samples_per_symbol": 256,
        "sample_rate": 998.4e6,
        "k_paths": 4,               # 1,2,4,8,16
        "fit_reps": 8,
        "validation_reps": 8,       # holdout, disjoint from fit_reps, >=8
        "roi_lo": 0,
        "roi_hi": 0,                # 0 => full window
        "coarse_stride": 14,
        "path_min_spacing": 0,      # 0 => derive from template autocorr mainlobe
        "post_tail": 64,
        "enable_fractional": True,
        "enable_cfo": True,
        "detect_threshold": 0.40,
        "validate_threshold": 0.25,
    }
    cfg.update(over)
    return cfg


def _blank_result():
    return {
        "state": STATE_BYPASSED,
        "detected": False,
        "committed": False,
        "partial": False,
        "ambiguous": False,
        "bypass_reason": None,
        "uses_truth": False,
        "detected_start": -1,
        "support_lo": -1,
        "support_hi": -1,
        "cfo_hz": 0.0,
        "n_paths": 0,
        "n_path_candidates": 0,
        "gain_condition": 0.0,
        "residual_corr": 0.0,
        "coarse_candidates": 0,
        "fine_candidates": 0,
        "correlations": 0,
        "roi_samples": 0,
        "reject_reason": None,
        "timings": {k: 0.0 for k in TIMING_NAMES},
        # reference-only extras (not part of the C++ SicResult struct)
        "taus": [],
        "gains": [],
        "i_hat": None,
        "out": None,
    }


def _bypass(result, y, reason):
    result["state"] = STATE_BYPASSED
    result["detected"] = False
    result["committed"] = False
    result["bypass_reason"] = reason
    result["i_hat"] = np.zeros(len(y), dtype=np.complex128)
    result["out"] = np.asarray(y, dtype=np.complex64).copy()
    return result


# ---------------------------------------------------------------------------
# Fractional-delay operators (the core OWNS one fixed band-limited model)
# ---------------------------------------------------------------------------
def _windowed_sinc(tau, half, win):
    """Kernel h[j], j = -half..half, h[j] = sinc(j - tau) * window(j)."""
    j = np.arange(-half, half + 1, dtype=np.float64)
    if win == "hamming":
        w = np.hamming(2 * half + 1)
    elif win == "blackman":
        w = np.blackman(2 * half + 1)
    else:
        raise ValueError("unknown window %r" % win)
    h = np.sinc(j - tau) * w
    s = h.sum()
    if s != 0.0:
        h = h / s                       # preserve DC gain (unit delay)
    return h


# The core's fixed band-limited interpolation.  Both the estimator and the
# model-consistent generator use this exact operator.
_CORE_MODEL = {"half": 16, "win": "hamming"}
# A deliberately different band-limited operator for the cross-model case.
_ALT_MODEL = {"half": 4, "win": "blackman"}


def _model_params(model):
    if model == "alt":
        return _ALT_MODEL
    return _CORE_MODEL


def _delay(x, tau, model="core"):
    """Return y with y[n] = x[n - tau], fractional via windowed-sinc.

    Integer tau is an exact shift; fractional tau uses the model's fixed
    band-limited interpolation (integer part placed first).
    """
    x = np.asarray(x, dtype=np.complex128)
    n_int = int(math.floor(tau))
    frac = float(tau - n_int)
    if abs(frac) < 1e-12:
        out = np.zeros_like(x)
    else:
        half = _model_params(model)["half"]
        win = _model_params(model)["win"]
        h = _windowed_sinc(frac, half, win)
        conv = np.convolve(x, h)
        out = conv[half:half + len(x)]
    # integer shift of `out`
    y = np.zeros_like(x)
    if n_int >= 0:
        if n_int < len(x):
            y[n_int:] = out[:len(x) - n_int]
    else:
        k = min(-n_int, len(x))
        y[:len(x) - k] = out[k:]
    return y


def _delay_into(dst, x, offset, model="core"):
    """dst[n] += x[n - offset] for offset possibly fractional (adds to dst)."""
    x = np.asarray(x, dtype=np.complex128)
    n_int = int(math.floor(offset))
    frac = float(offset - n_int)
    xs = _delay(x, frac, model) if abs(frac) > 1e-12 else x
    if n_int >= 0:
        m = min(len(xs), len(dst) - n_int)
        if m > 0:
            dst[n_int:n_int + m] += xs[:m]
    else:
        m = min(len(xs) + n_int, len(dst))
        if m > 0:
            dst[:m] += xs[-n_int:-n_int + m]
    return dst


def _cfo_ramp(n_start, m, f, fs, reanchor=256):
    """exp(j 2*pi*f*n/fs) for n = n_start .. n_start+m-1.

    Recurrence rotation with a periodic absolute re-anchor, as the contract
    recommends (avoids per-point sin/cos).
    """
    out = np.empty(m, dtype=np.complex128)
    if m == 0:
        return out
    step = np.exp(1j * 2.0 * np.pi * f / fs)
    i = 0
    while i < m:
        block = min(reanchor, m - i)
        out[i] = np.exp(1j * 2.0 * np.pi * f * (n_start + i) / fs)
        for j in range(1, block):
            out[i + j] = out[i + j - 1] * step
        i += block
    return out


# ---------------------------------------------------------------------------
# Correlation / peak helpers
# ---------------------------------------------------------------------------
def _template_mainlobe(sync):
    """Autocorrelation mainlobe width (samples) of the sync template."""
    a = np.abs(np.correlate(np.asarray(sync, dtype=np.complex128),
                            np.asarray(sync, dtype=np.complex128), mode="full"))
    c = len(sync) - 1
    peak = a[c]
    if peak <= 0:
        return 1
    for lag in range(1, len(sync)):
        if a[c + lag] < 0.5 * peak:
            return max(1, lag)
    return 1


def _local_peaks(mag):
    peaks = []
    for i in range(1, len(mag) - 1):
        if mag[i] > mag[i - 1] and mag[i] >= mag[i + 1]:
            peaks.append(i)
    peaks.sort(key=lambda i: -mag[i])
    return peaks


# ---------------------------------------------------------------------------
# SicMode::Estimate  --  no truth, ever
# ---------------------------------------------------------------------------
def estimate_sic(y, sync, preamble, cfg):
    """Mirror of the Estimate mode. See docs/sic/S0_sic_core_api.md."""
    t_core = time.perf_counter()
    y = np.asarray(y, dtype=np.complex128)
    sync = np.asarray(sync, dtype=np.complex128)
    preamble = np.asarray(preamble, dtype=np.complex128)
    cfg = dict(cfg)
    N = len(y)
    Ls = len(sync)
    fs = float(cfg["sample_rate"])
    sps = int(cfg["samples_per_symbol"])
    k_max = int(cfg["k_paths"])
    fit_reps = int(cfg["fit_reps"])
    val_reps = int(cfg["validation_reps"])
    post_tail = int(cfg["post_tail"])
    detect_thr = float(cfg["detect_threshold"])
    validate_thr = float(cfg["validate_threshold"])
    model = "core"

    result = _blank_result()
    times = result["timings"]

    roi_lo = int(cfg["roi_lo"])
    roi_hi = int(cfg["roi_hi"])
    if roi_hi <= roi_lo:
        roi_lo, roi_hi = 0, N
    roi_hi = min(roi_hi, N - Ls + 1)
    roi_lo = max(roi_lo, 0)
    result["roi_samples"] = max(0, roi_hi - roi_lo)
    stride = max(1, int(cfg["coarse_stride"]))
    sync_norm = float(np.linalg.norm(sync))

    def norm_at(d):
        seg = y[d:d + Ls]
        c = np.vdot(sync, seg)
        e = float(np.linalg.norm(seg))
        nrm = abs(c) / (e * sync_norm) if e > 0.0 else 0.0
        return c, nrm

    # --- 1. detect_coarse --------------------------------------------------
    t = time.perf_counter()
    coarse_best = (-1, 0.0)
    for d in range(roi_lo, roi_hi, stride):
        _, nrm = norm_at(d)
        result["coarse_candidates"] += 1
        result["correlations"] += 1
        if nrm > coarse_best[1]:
            coarse_best = (d, nrm)
    times["detect_coarse"] = _us(t)

    # --- 1b. detect_fine ---------------------------------------------------
    t = time.perf_counter()
    fine_best = coarse_best
    if coarse_best[0] >= 0:
        lo = max(roi_lo, coarse_best[0] - stride)
        hi = min(roi_hi, coarse_best[0] + stride + 1)
        for d in range(lo, hi):
            _, nrm = norm_at(d)
            result["fine_candidates"] += 1
            result["correlations"] += 1
            if nrm > fine_best[1]:
                fine_best = (d, nrm)
    times["detect_fine"] = _us(t)

    detected_start, peak_norm = fine_best
    # --- 2. repetition confirmation (resolve period ambiguity) -------------
    # A peak may land on any repeated SYNC symbol.  The comm signal is absent
    # *before* the preamble origin, so walking back one symbol at a time while
    # the correlation stays confirmed gives the true origin (never a mid-period
    # peak).  Then snap to the local peak at stride 1.
    t = time.perf_counter()
    if detected_start >= 0:
        origin = int(detected_start)
        while origin - sps >= max(roi_lo, 0):
            d = origin - sps
            nrm = norm_at(d)[1] if d + Ls <= N else 0.0
            result["correlations"] += 1
            if nrm >= detect_thr:
                origin = d
            else:
                break
        lo = max(roi_lo, origin - stride)
        hi = min(roi_hi, origin + stride + 1)
        for d in range(lo, hi):
            _, nrm = norm_at(d)
            result["fine_candidates"] += 1
            result["correlations"] += 1
            if nrm > peak_norm:
                origin, peak_norm = d, nrm
        detected_start = origin
    times["detect_fine"] += _us(t)
    if detected_start < 0 or peak_norm < detect_thr:
        times["core_total"] = _us(t_core)
        return _bypass(result, y, "below_detect_threshold")
    result["detected"] = True
    result["detected_start"] = int(detected_start)

    # --- 2. repetition confirmation ---------------------------------------
    rtot = fit_reps + val_reps
    rep_norms = []
    rep_corr = []
    for k in range(rtot):
        d = detected_start + k * sps
        if d + Ls <= N:
            c, nrm = norm_at(d)
        else:
            c, nrm = 0j, 0.0
        rep_norms.append(nrm)
        rep_corr.append(c)
    confirmed = sum(1 for x in rep_norms if x >= detect_thr)
    if confirmed < 2:
        times["core_total"] = _us(t_core)
        _bypass(result, y, "repetition_not_confirmed")
        result["state"] = STATE_AMBIGUOUS
        result["ambiguous"] = True
        result["reject_reason"] = "repetition_not_confirmed"
        return result
    if confirmed < rtot:
        result["partial"] = True

    # --- 3. cfo ------------------------------------------------------------
    t = time.perf_counter()
    f_est = 0.0
    if cfg["enable_cfo"]:
        # The first SYNC symbol carries the leading-edge transient, which biases
        # its complex correlation phase; estimate CFO from the steady-state
        # (repeated) symbols only.
        phis = []
        k_lo = 1 if rtot >= 3 else 0
        for k in range(k_lo, rtot - 1):
            if rep_norms[k] >= detect_thr and rep_norms[k + 1] >= detect_thr:
                phis.append(np.angle(rep_corr[k + 1] * np.conj(rep_corr[k])))
        if phis:
            z = np.mean(np.exp(1j * np.asarray(phis)))
            dphi = float(np.angle(z))
            f_est = dphi / (2.0 * np.pi * sps / fs)
    times["cfo"] = _us(t)
    result["cfo_hz"] = float(f_est)

    # --- 4. cir / path_select ---------------------------------------------
    t = time.perf_counter()
    dmin, dmax = -4, post_tail + 4
    cir = np.zeros(dmax - dmin, dtype=np.complex128)
    for idx, d in enumerate(range(dmin, dmax)):
        pos = detected_start + d
        if pos >= 0 and pos + Ls <= N:
            cir[idx] = np.vdot(sync, y[pos:pos + Ls])
    times["cir"] = _us(t)

    t = time.perf_counter()
    mag = np.abs(cir)
    peaks = _local_peaks(mag)
    result["n_path_candidates"] = len(peaks)
    min_spacing = int(cfg["path_min_spacing"]) or _template_mainlobe(sync)
    floor = 0.15 * float(mag.max()) if mag.size else 0.0
    selected = []
    for i in peaks:
        if mag[i] < floor:
            continue
        if all(abs(i - j) >= min_spacing for j in selected):
            selected.append(i)
        if len(selected) >= k_max:
            break
    delays = [float(i + dmin) for i in selected]
    times["path_select"] = _us(t)

    if not delays:
        times["core_total"] = _us(t_core)
        _bypass(result, y, "no_paths")
        result["state"] = STATE_FAILED
        result["reject_reason"] = "no_paths"
        return result

    # --- 5/6/7. fractional_refine, matrix_build, gain_fit, reconstruct -----
    def shifted(tau):
        out = np.zeros(N, dtype=np.complex128)
        _delay_into(out, preamble, tau, model)
        return out

    ramp = _cfo_ramp(0, N, f_est, fs)
    fit_lo = detected_start
    fit_hi = detected_start + fit_reps * sps
    fit_hi = min(fit_hi, N)
    Wf = np.arange(fit_lo, fit_hi)

    def col_at(tau):
        return (shifted(tau)[Wf] * ramp[Wf])

    def solve_ls(cols, target):
        A = np.stack(cols, axis=1)
        Q, R = np.linalg.qr(A, mode="reduced")       # no explicit inverse
        h = np.linalg.solve(R, Q.conj().T @ target)
        return A, h, float(np.linalg.cond(A))

    # 5. fractional_refine (integer delays are closed first, then refined)
    t = time.perf_counter()
    taus = [float(detected_start) + d for d in delays]
    if cfg["enable_fractional"]:
        _, h, _ = solve_ls([col_at(tau) for tau in taus], y[Wf])
        offsets = np.arange(-0.5, 0.5 + 1e-9, 0.05)
        for k in range(len(taus)):
            # residual from all other paths (coordinate descent)
            r = y[Wf].copy()
            for j in range(len(taus)):
                if j != k:
                    r = r - h[j] * col_at(taus[j])
            best = (taus[k], -1.0)
            for delta in offsets:
                col = col_at(taus[k] + delta)
                num = abs(np.vdot(col, r)) ** 2
                den = float(np.vdot(col, col).real)
                score = num / den if den > 0 else 0.0
                if score > best[1]:
                    best = (taus[k] + delta, score)
            taus[k] = best[0]
    times["fractional_refine"] = _us(t)

    # 6a. matrix_build
    t = time.perf_counter()
    basis = [shifted(tau) for tau in taus]
    cols = [basis[k][Wf] * ramp[Wf] for k in range(len(taus))]
    A = np.stack(cols, axis=1)
    times["matrix_build"] = _us(t)

    # 6b. gain_fit (complex LS via QR; conditioning reported, no inverse)
    t = time.perf_counter()
    Q, R = np.linalg.qr(A, mode="reduced")
    h = np.linalg.solve(R, Q.conj().T @ y[Wf])
    cond = float(np.linalg.cond(A))
    times["gain_fit"] = _us(t)

    result["gain_condition"] = cond
    result["n_paths"] = len(taus)
    result["taus"] = [float(x) for x in taus]
    result["gains"] = [complex(x) for x in h]

    t = time.perf_counter()
    basis = [shifted(tau) for tau in taus]
    i_hat = np.zeros(N, dtype=np.complex128)
    for k in range(len(taus)):
        i_hat = i_hat + h[k] * basis[k] * ramp
    times["reconstruct"] = _us(t)

    # --- 7b. subtraction mask + out ---------------------------------------
    support_lo = int(detected_start)
    support_hi = int(detected_start) + confirmed * sps + post_tail
    support_hi = min(support_hi, N)
    result["support_lo"] = max(support_lo, 0)
    result["support_hi"] = support_hi
    sub_lo = max(support_lo, roi_lo, 0)
    sub_hi = min(support_hi, roi_hi, N)
    out = y.astype(np.complex64).copy()
    if sub_hi > sub_lo:
        sub = (y[sub_lo:sub_hi] - i_hat[sub_lo:sub_hi]).astype(np.complex64)
        out[sub_lo:sub_hi] = sub
    result["i_hat"] = i_hat
    result["out"] = out

    # --- 8. validate_commit (disjoint holdout) ----------------------------
    t = time.perf_counter()
    hstart = detected_start + fit_reps * sps
    hend = detected_start + (fit_reps + val_reps) * sps
    finite_ok = bool(np.all(np.isfinite(h)) and all(
        np.all(np.isfinite(b)) for b in basis))
    cond_ok = cond < 1e6
    commit = False
    if hend <= N and finite_ok and cond_ok:
        hold = slice(hstart, hend)
        r = y[hold] - i_hat[hold]
        p_col = shifted(detected_start)[hold] * ramp[hold]
        den = float(np.linalg.norm(r) * np.linalg.norm(p_col))
        rc = abs(np.vdot(p_col, r)) / den if den > 0 else 1.0
        result["residual_corr"] = float(rc)
        commit = rc < validate_thr
        if not commit:
            result["reject_reason"] = "holdout_residual_high"
    else:
        result["residual_corr"] = float("nan")
        result["reject_reason"] = "validation_window_unavailable"
    times["validate_commit"] = _us(t)

    if commit:
        result["state"] = STATE_COMMITTED
        result["committed"] = True
    else:
        # On any failure: bypass, out == y exactly.
        result["out"] = y.astype(np.complex64).copy()
        if result["reject_reason"] is None:
            result["reject_reason"] = "not_committed"

    times["core_total"] = _us(t_core)
    return result


# ---------------------------------------------------------------------------
# SicMode::Oracle  --  diagnostic upper bound, uses_truth=True
# ---------------------------------------------------------------------------
def oracle_sic(y, sync, preamble, cfg, truth):
    """Exact reconstruction from injected truth. Diagnostic only."""
    t_core = time.perf_counter()
    y = np.asarray(y, dtype=np.complex128)
    preamble = np.asarray(preamble, dtype=np.complex128)
    cfg = dict(cfg)
    N = len(y)
    fs = float(cfg["sample_rate"])
    model = truth.get("model", "core")
    start = float(truth["start"])
    f = float(truth["cfo_hz"])

    result = _blank_result()
    result["state"] = STATE_COMMITTED
    result["detected"] = True
    result["committed"] = True
    result["uses_truth"] = True
    result["detected_start"] = int(round(start))
    result["cfo_hz"] = f
    result["n_paths"] = len(truth["paths"])
    result["taus"] = [start + float(p["delay"]) for p in truth["paths"]]
    result["gains"] = [complex(p["gain"]) for p in truth["paths"]]

    i_hat = np.zeros(N, dtype=np.complex128)
    for p in truth["paths"]:
        _delay_into(i_hat, preamble * complex(p["gain"]),
                    start + float(p["delay"]), model)
    i_hat = i_hat * _cfo_ramp(0, N, f, fs)

    post_tail = int(cfg["post_tail"])
    support_lo = int(round(start))
    support_hi = min(support_lo + int(cfg["sync_reps"]) *
                     int(cfg["samples_per_symbol"]) + post_tail, N)
    result["support_lo"] = support_lo
    result["support_hi"] = support_hi
    out = y.astype(np.complex64).copy()
    out[support_lo:support_hi] = (
        y[support_lo:support_hi] - i_hat[support_lo:support_hi]).astype(np.complex64)
    result["i_hat"] = i_hat
    result["out"] = out
    result["timings"]["core_total"] = _us(t_core)
    return result


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def nmse(i_true, i_hat):
    i_true = np.asarray(i_true, dtype=np.complex128)
    i_hat = np.asarray(i_hat, dtype=np.complex128)
    den = float(np.sum(np.abs(i_true) ** 2))
    if den <= 0.0:
        return 0.0
    return float(np.sum(np.abs(i_true - i_hat) ** 2)) / den


def suppression_db(i_true, i_hat, eps=1e-15):
    n = nmse(i_true, i_hat)
    return 10.0 * math.log10(1.0 / max(n, eps))


# ---------------------------------------------------------------------------
# Synthetic model-consistent case generator (truth-bearing)
# ---------------------------------------------------------------------------
def _make_pulse(length=17, fc=0.25):
    n = np.arange(length) - (length - 1) // 2
    h = 2.0 * fc * np.sinc(2.0 * fc * n) * np.hamming(length)
    return h / h.sum()


def _msequence(m, taps):
    """Maximal-length LFSR sequence mapped to +/-1 (length 2**m - 1)."""
    st = [0] * (m - 1) + [1]
    out = np.empty((1 << m) - 1, dtype=np.float64)
    for i in range(out.size):
        out[i] = st[-1]
        fb = 0
        for t in taps:
            fb ^= st[t - 1]
        st = [fb] + st[:-1]
    return out * 2.0 - 1.0


# Different comm codes -> different m-sequences (independent comm namespace).
_CODE_TAPS = {9: (7, 6), 10: (7, 1), 11: (7, 3), 12: (7, 4)}


def _make_templates(sps, reps, upsample, code, pulse):
    """One shaped SYNC waveform repeated ``reps`` times, plus a pulse tail.

    The per-symbol chip pattern is the SAME every repetition (the real preamble
    code repeats), so the steady-state symbols are periodic.  The first symbol
    differs only by the pulse's causal ramp-up (the leading-edge transient).
    An m-sequence gives the low autocorrelation sidelobes a real preamble code
    has, so the matched-filter CIR has unambiguous path peaks.
    """
    n_chips = sps // upsample
    if n_chips == 127:
        chips = _msequence(7, _CODE_TAPS.get(int(code), (7, 6)))
    else:  # deterministic fallback for non-canonical lengths
        rng = np.random.default_rng(int(code) * 131 + n_chips)
        chips = rng.integers(0, 2, n_chips) * 2.0 - 1.0
    chips = np.tile(chips, reps)
    up = np.zeros(chips.size * upsample, dtype=np.float64)
    up[::upsample] = chips
    shaped = np.convolve(up, pulse)
    preamble = shaped.astype(np.complex128)
    sync = preamble[:sps].copy()
    return sync, preamble


CASE_DEFAULTS = dict(
    sps=127, reps=16, upsample=1, start=64, N=2160,
    fs=998.4e6, isr_db=10.0, snr_db=30.0,
    code=10, sig_seed=2222, noise_seed=3333,
    pulse_len=9, pulse_fc=0.4,
)


def build_case(spec):
    """Return a dict with y/sync/preamble/i plus truth/cfg/meta, deterministic."""
    p = dict(CASE_DEFAULTS)
    p.update(spec.get("params", {}))
    sps = int(p["sps"])
    reps = int(p["reps"])
    upsample = int(p["upsample"])
    start = float(p["start"])
    N = int(p["N"])
    fs = float(p["fs"])
    pulse = _make_pulse(int(p["pulse_len"]), float(p["pulse_fc"]))

    code = int(spec.get("code", p["code"]))
    sync, preamble = _make_templates(sps, reps, upsample, code, pulse)

    gen_model = spec.get("gen_model", "core")
    paths = spec["paths"]                       # list[(delay, gain)]
    cfo_hz = float(spec.get("cfo_hz", 0.0))

    # interferer i[n] = (sum_k gain_k * preamble[n - start - delay_k]) * ramp[n]
    i = np.zeros(N, dtype=np.complex128)
    for delay, gain in paths:
        _delay_into(i, preamble * complex(gain), start + float(delay), gen_model)
    i = i * _cfo_ramp(0, N, cfo_hz, fs)

    # independent sensing stand-in s (never the SIC target)
    rng_s = np.random.default_rng(int(p["sig_seed"]))
    cs = rng_s.standard_normal(N) + 1j * rng_s.standard_normal(N)
    s = np.convolve(cs, pulse)[:N]
    rms_i = math.sqrt(float(np.mean(np.abs(i) ** 2)))
    rms_s = math.sqrt(float(np.mean(np.abs(s) ** 2))) or 1.0
    s = s * (rms_i / (10.0 ** (float(p["isr_db"]) / 20.0)) / rms_s)

    # complex AWGN; noise_std is per-I/Q component
    sigma = math.sqrt(float(np.mean(np.abs(s) ** 2)) /
                      (2.0 * 10.0 ** (float(p["snr_db"]) / 10.0)))
    rng_w = np.random.default_rng(int(p["noise_seed"]))
    w = (rng_w.standard_normal(N) + 1j * rng_w.standard_normal(N)) * sigma

    y = (s + i + w).astype(np.complex64)
    i_truth = i.astype(np.complex64)

    cfg = default_config(
        comm_code=int(spec.get("comm_code", 10)),
        sync_reps=reps,
        samples_per_symbol=sps,
        sample_rate=fs,
        k_paths=int(spec.get("k_paths", 4)),
        fit_reps=int(spec.get("fit_reps", 8)),
        validation_reps=int(spec.get("validation_reps", 8)),
        roi_lo=0,
        roi_hi=N,
        coarse_stride=int(p.get("coarse_stride", 14)),
        path_min_spacing=0,
        post_tail=int(spec.get("post_tail", 64)),
        enable_fractional=bool(spec.get("enable_fractional", True)),
        enable_cfo=bool(spec.get("enable_cfo", True)),
        detect_threshold=float(spec.get("detect_threshold", 0.40)),
        validate_threshold=float(spec.get("validate_threshold", 0.25)),
    )

    truth = {
        "start": float(start),
        "cfo_hz": cfo_hz,
        "model": gen_model,
        "paths": [{"delay": float(d), "gain": complex(g)} for d, g in paths],
    }
    return {
        "name": spec["name"],
        "description": spec.get("description", ""),
        "y": y,
        "sync": sync.astype(np.complex64),
        "preamble": preamble.astype(np.complex64),
        "i": i_truth,
        "s": s.astype(np.complex64),
        "w": w.astype(np.complex64),
        "truth": truth,
        "cfg": cfg,
    }


# ---------------------------------------------------------------------------
# Golden case catalogue
# ---------------------------------------------------------------------------
def golden_cases():
    four = [(0.0, 1.0 + 0j), (8.0, 0.5j),
            (24.0, 0.3 * np.exp(-0.4j)), (52.0, 0.2 + 0j)]
    cases = []

    cases.append(dict(
        name="int_single",
        description="one integer path (delay 0, unit gain); no CFO",
        paths=[(0.0, 1.0 + 0j)],
        cfo_hz=0.0,
        gen_model="core",
        code=10,
        enable_fractional=False,
        expected=dict(detected_start=64, cfo_hz=0.0,
                      taus=[64.0], gains=[1.0 + 0j]),
        pass_criteria=dict(oracle_nmse_max=1e-6, estimate_nmse_max=0.02,
                           suppression_min_db=15.0, detected_start_tol=1,
                           cfo_hz_tol=2000.0, require_commit=True),
    ))
    cases.append(dict(
        name="int_four",
        description="four integer paths (0,8,24,52); no CFO",
        paths=four,
        cfo_hz=0.0,
        gen_model="core",
        code=10,
        enable_fractional=False,
        expected=dict(detected_start=64, cfo_hz=0.0,
                      taus=[64.0, 72.0, 88.0, 116.0],
                      gains=[1.0 + 0j, 0.5j,
                             0.3 * np.exp(-0.4j), 0.2 + 0j]),
        pass_criteria=dict(oracle_nmse_max=1e-6, estimate_nmse_max=0.05,
                           suppression_min_db=12.0, detected_start_tol=1,
                           cfo_hz_tol=2000.0, require_commit=True),
    ))
    cases.append(dict(
        name="frac",
        description="one fractional path (delay 7.4); generator==core model",
        paths=[(7.4, 1.0 + 0j)],
        cfo_hz=0.0,
        gen_model="core",
        code=10,
        enable_fractional=True,
        expected=dict(detected_start=71, cfo_hz=0.0,
                      taus=[71.4], gains=[1.0 + 0j]),
        pass_criteria=dict(oracle_nmse_max=1e-6, estimate_nmse_max=0.05,
                           suppression_min_db=12.0, detected_start_tol=2,
                           cfo_hz_tol=2000.0, require_commit=True),
    ))
    cases.append(dict(
        name="cfo",
        description="integer path (delay 0) + CFO +50 kHz",
        paths=[(0.0, 1.0 + 0j)],
        cfo_hz=50000.0,
        gen_model="core",
        code=10,
        enable_fractional=False,
        expected=dict(detected_start=64, cfo_hz=50000.0,
                      taus=[64.0], gains=[1.0 + 0j]),
        pass_criteria=dict(oracle_nmse_max=1e-6, estimate_nmse_max=0.05,
                           suppression_min_db=12.0, detected_start_tol=1,
                           cfo_hz_tol=3000.0, require_commit=True),
    ))
    cases.append(dict(
        name="k_sweep",
        description="4-path channel swept over K=1/2/4/8/16",
        paths=four,
        cfo_hz=0.0,
        gen_model="core",
        code=10,
        enable_fractional=False,
        k_sweep=[1, 2, 4, 8, 16],
        expected=dict(detected_start=64, cfo_hz=0.0,
                      taus=[64.0, 72.0, 88.0, 116.0],
                      gains=[1.0 + 0j, 0.5j,
                             0.3 * np.exp(-0.4j), 0.2 + 0j]),
        pass_criteria=dict(oracle_nmse_max=1e-6,
                           per_k={"1": dict(estimate_nmse_max=1.5, declared="limited"),
                                  "2": dict(estimate_nmse_max=0.9, declared="limited"),
                                  "4": dict(estimate_nmse_max=0.05),
                                  "8": dict(estimate_nmse_max=0.05),
                                  "16": dict(estimate_nmse_max=0.05)},
                           detected_start_tol=1, cfo_hz_tol=2000.0,
                           require_commit=True),
    ))
    cases.append(dict(
        name="cross_model",
        description="fractional path 7.4; generator uses an independent "
                    "band-limited kernel, estimator uses the core model",
        paths=[(7.4, 1.0 + 0j)],
        cfo_hz=0.0,
        gen_model="alt",
        code=10,
        enable_fractional=True,
        declared_stress=True,
        expected=dict(detected_start=71, cfo_hz=0.0,
                      taus=[71.4], gains=[1.0 + 0j]),
        pass_criteria=dict(oracle_nmse_max=1e-6, estimate_nmse_max=0.10,
                           suppression_min_db=10.0, detected_start_tol=2,
                           cfo_hz_tol=2000.0, require_commit=True),
    ))
    return cases


# ---------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------
def _fmt(x, nd=3):
    return ("%.*g" % (nd, x)) if isinstance(x, float) else str(x)


def _run_case(case, cfg_over=None):
    built = build_case(case)
    cfg = dict(built["cfg"])
    if cfg_over:
        cfg.update(cfg_over)
    y, sync, preamble = built["y"], built["sync"], built["preamble"]
    est = estimate_sic(y, sync, preamble, cfg)
    orc = oracle_sic(y, sync, preamble, cfg, built["truth"])
    return built, cfg, est, orc


def self_test(golden_dir=None):
    failures = []
    print("== sic_reference self-test ==")

    # 1..: per-case estimate + oracle
    for name in ("int_single", "int_four", "frac", "cfo"):
        case = next(c for c in golden_cases() if c["name"] == name)
        built, cfg, est, orc = _run_case(case)
        i = built["i"]
        nm = nmse(i, est["i_hat"])
        supp = suppression_db(i, est["i_hat"])
        orc_nmse = nmse(i, orc["i_hat"])
        pc = case["pass_criteria"]
        ok = True
        msgs = []
        if orc_nmse > pc["oracle_nmse_max"]:
            ok = False
            msgs.append("oracle_nmse=%.3g>%.3g" % (orc_nmse, pc["oracle_nmse_max"]))
        if nm > pc["estimate_nmse_max"]:
            ok = False
            msgs.append("nmse=%.3g>%.3g" % (nm, pc["estimate_nmse_max"]))
        if supp < pc["suppression_min_db"]:
            ok = False
            msgs.append("supp=%.2f<%.2f" % (supp, pc["suppression_min_db"]))
        if abs(est["detected_start"] - case["expected"]["detected_start"]) > \
                pc["detected_start_tol"]:
            ok = False
            msgs.append("start=%d" % est["detected_start"])
        if abs(est["cfo_hz"] - case["expected"]["cfo_hz"]) > pc["cfo_hz_tol"]:
            ok = False
            msgs.append("cfo=%.1f" % est["cfo_hz"])
        if pc["require_commit"] and not est["committed"]:
            ok = False
            msgs.append("not_committed:%s" % est["reject_reason"])
        if est["uses_truth"]:
            ok = False
            msgs.append("uses_truth_true")
        status = "PASS" if ok else "FAIL"
        if not ok:
            failures.append("%s: %s" % (name, "; ".join(msgs)))
        print("%-5s %-12s nmse=%-10.3g supp=%-7.2fdB oracle_nmse=%-10.3g "
              "start=%-4d cfo=%-9.1f commit=%s%s"
              % (status, name, nm, supp, orc_nmse, est["detected_start"],
                 est["cfo_hz"], est["committed"],
                 ("  [" + "; ".join(msgs) + "]") if msgs else ""))

    # oracle NMSE ~ 0 on a model-consistent case (int_four)
    case = next(c for c in golden_cases() if c["name"] == "int_four")
    built, cfg, est, orc = _run_case(case)
    o = nmse(built["i"], orc["i_hat"])
    if o >= 1e-6:
        failures.append("oracle int_four nmse=%.3g" % o)
    print("%-5s %-12s oracle_nmse=%.3g (model-consistent upper bound)"
          % ("PASS" if o < 1e-6 else "FAIL", "oracle", o))

    # k_sweep: per-K declared criteria
    case = next(c for c in golden_cases() if c["name"] == "k_sweep")
    for k in case["k_sweep"]:
        built = build_case(case)
        cfg = dict(built["cfg"])
        cfg["k_paths"] = k
        est = estimate_sic(built["y"], built["sync"], built["preamble"], cfg)
        nm = nmse(built["i"], est["i_hat"])
        supp = suppression_db(built["i"], est["i_hat"])
        crit = case["pass_criteria"]["per_k"][str(k)]
        ok = (nm <= crit["estimate_nmse_max"]) and est["committed"]
        decl = crit.get("declared")
        tag = "" if not decl else "  declared=%s" % decl
        if not ok:
            failures.append("k_sweep K=%d nmse=%.3g>%.3g" %
                            (k, nm, crit["estimate_nmse_max"]))
        print("%-5s %-12s K=%-2d nmse=%-10.3g supp=%-7.2fdB n_paths=%d%s"
              % ("PASS" if ok else "FAIL", "k_sweep", k, nm, supp,
                 est["n_paths"], tag))

    # cross_model: meet criterion OR explicit declared stress failure
    case = next(c for c in golden_cases() if c["name"] == "cross_model")
    built, cfg, est, orc = _run_case(case)
    nm = nmse(built["i"], est["i_hat"])
    supp = suppression_db(built["i"], est["i_hat"])
    pc = case["pass_criteria"]
    ok = (nm <= pc["estimate_nmse_max"] and est["committed"] and
          abs(est["detected_start"] - case["expected"]["detected_start"]) <=
          pc["detected_start_tol"])
    if ok:
        print("%-5s %-12s nmse=%-10.3g supp=%-7.2fdB (declared stress, met criterion)"
              % ("PASS", "cross_model", nm, supp))
    elif case.get("declared_stress"):
        print("%-5s %-12s nmse=%-10.3g supp=%-7.2fdB  DECLARED-STRESS-FAILURE "
              "(not silently passed)" % ("WARN", "cross_model", nm, supp))
    else:
        failures.append("cross_model nmse=%.3g" % nm)
        print("%-5s %-12s nmse=%-10.3g" % ("FAIL", "cross_model", nm))

    # bypass: wrong code and no interference -> out == y byte-for-byte
    y = build_case(golden_cases()[0])["y"]
    # a code-11 (wrong) template
    pulse = _make_pulse()
    sync_w, pre_w = _make_templates(127, 16, 1, 11, pulse)
    cfg = default_config()
    r = estimate_sic(y, sync_w.astype(np.complex64),
                     pre_w.astype(np.complex64), cfg)
    ok = (not r["committed"]) and np.array_equal(r["out"], y)
    if not ok:
        failures.append("bypass_wrong_code out!=y or committed")
    print("%-5s %-12s state=%s committed=%s out==y=%s"
          % ("PASS" if ok else "FAIL", "bypass_wrong", r["state"],
             r["committed"], np.array_equal(r["out"], y)))

    # bypass: no comm present (sensing only) -> out == y exactly
    built = build_case(golden_cases()[0])
    y_no = (built["s"] + built["w"]).astype(np.complex64)
    r2 = estimate_sic(y_no, built["sync"], built["preamble"], default_config())
    ok2 = (not r2["committed"]) and np.array_equal(r2["out"], y_no)
    if not ok2:
        failures.append("bypass_no_comm out!=y or committed")
    print("%-5s %-12s state=%s out==y=%s"
          % ("PASS" if ok2 else "FAIL", "bypass_nocomm", r2["state"],
             np.array_equal(r2["out"], y_no)))

    # truth isolation: estimate_sic must have no truth argument
    params = list(inspect.signature(estimate_sic).parameters)
    ok3 = (params == ["y", "sync", "preamble", "cfg"])
    if not ok3:
        failures.append("estimate_sic signature %r" % params)
    print("%-5s %-12s signature=%s" % ("PASS" if ok3 else "FAIL",
                                       "truth_iso", params))

    # committed golden vectors must reproduce (schema + hashes + outputs)
    if golden_dir and os.path.isdir(golden_dir):
        _check_golden(golden_dir, failures)
    else:
        print("%-5s %-12s (golden dir absent)" % ("SKIP", "golden"))

    print("== %s ==" % ("ALL PASS" if not failures
                        else "FAILURES: " + " | ".join(failures)))
    return 1 if failures else 0


def _check_golden(golden_dir, failures):
    """Verify committed golden vectors load, hash, and reproduce exactly."""
    index = json.load(open(os.path.join(golden_dir, "index.json")))
    bad = []
    for entry in index["cases"]:
        io_path = os.path.join(golden_dir, entry["path"])
        meta = json.load(open(os.path.join(
            golden_dir, entry["name"], "meta.json")))
        z = np.load(io_path)
        h = _sha256_file(io_path)
        est = estimate_sic(z["y"], z["sync"], z["preamble"],
                           meta["runtime_cfg"])
        ok = (h == entry["sha256"]
              and sorted(z.files) == sorted(["y", "sync", "preamble", "i",
                                             "i_hat_expected", "out_expected"])
              and np.array_equal(est["out"], z["out_expected"])
              and np.allclose(est["i_hat"], z["i_hat_expected"], atol=1e-5))
        if not ok:
            bad.append(entry["name"])
            failures.append("golden %s: hash/output mismatch" % entry["name"])
    print("%-5s %-12s %d cases, hash+out_expected verified"
          % ("PASS" if not bad else "FAIL", "golden", len(index["cases"])))
    return bad


# ---------------------------------------------------------------------------
# Golden emission
# ---------------------------------------------------------------------------
def _sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _json_safe(obj):
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, complex):
        return [obj.real, obj.imag]
    if isinstance(obj, np.ndarray):
        return _json_safe(obj.tolist())
    return obj


def _criterion_summary(case):
    pc = case["pass_criteria"]
    return {k: v for k, v in pc.items()}


def _emit_one(out_dir, case):
    built = build_case(case)
    case_dir = os.path.join(out_dir, case["name"])
    os.makedirs(case_dir, exist_ok=True)

    if "k_sweep" in case:
        nominal_k = 4
    else:
        nominal_k = built["cfg"]["k_paths"]
    cfg = dict(built["cfg"])
    cfg["k_paths"] = nominal_k
    est = estimate_sic(built["y"], built["sync"], built["preamble"], cfg)

    io_path = os.path.join(case_dir, "io.npz")
    np.savez(
        io_path,
        y=built["y"].astype(np.complex64),
        sync=built["sync"].astype(np.complex64),
        preamble=built["preamble"].astype(np.complex64),
        i=built["i"].astype(np.complex64),
        i_hat_expected=est["i_hat"].astype(np.complex64),
        out_expected=est["out"].astype(np.complex64),
    )

    truth = built["truth"]
    meta = {
        "schema_version": SCHEMA_VERSION,
        "case": case["name"],
        "description": case["description"],
        "sample_rate": float(built["cfg"]["sample_rate"]),
        "samples_per_symbol": int(built["cfg"]["samples_per_symbol"]),
        "sync_reps": int(built["cfg"]["sync_reps"]),
        "arrays": {
            "y": "complex64 received window (s + i + w)",
            "sync": "complex64 ONE shaped SYNC template (comm code)",
            "preamble": "complex64 FULL comm preamble template",
            "i": "complex64 TRUTH interferer (never pass to estimate_sic)",
            "i_hat_expected": "complex64 reference estimate reconstruction",
            "out_expected": "complex64 reference estimate output (masked)",
        },
        "generator": {
            "paths": [{"delay_samples": float(d), "gain_complex": [float(g.real),
                       float(g.imag)]} for d, g in case["paths"]],
            "cfo_hz": float(case.get("cfo_hz", 0.0)),
            "frac_model": case.get("gen_model", "core"),
            "code": int(case.get("code", CASE_DEFAULTS["code"])),
        },
        "truth": {
            "start": truth["start"],
            "cfo_hz": truth["cfo_hz"],
            "model": truth["model"],
            "paths": [{"delay_samples": p["delay"],
                       "gain_complex": [p["gain"].real, p["gain"].imag]}
                      for p in truth["paths"]],
        },
        "runtime_cfg": dict(built["cfg"]),
        "expected": {
            "detected_start": case["expected"]["detected_start"],
            "cfo_hz": case["expected"]["cfo_hz"],
            "taus": case["expected"]["taus"],
            "gains": _json_safe(case["expected"]["gains"]),
        },
        "thresholds": {
            "detect_threshold": built["cfg"]["detect_threshold"],
            "validate_threshold": built["cfg"]["validate_threshold"],
        },
        "pass_criteria": _json_safe(_criterion_summary(case)),
        "nominal_k_used_for_expected_arrays": nominal_k,
        "declared_stress": bool(case.get("declared_stress", False)),
    }
    if "k_sweep" in case:
        meta["k_sweep"] = case["k_sweep"]
    with open(os.path.join(case_dir, "meta.json"), "w") as fh:
        json.dump(_json_safe(meta), fh, indent=2, sort_keys=False)
        fh.write("\n")

    return {
        "name": case["name"],
        "path": "%s/io.npz" % case["name"],
        "sha256": _sha256_file(io_path),
        "arrays": ["y", "sync", "preamble", "i", "i_hat_expected", "out_expected"],
        "samples_per_symbol": int(built["cfg"]["samples_per_symbol"]),
        "sync_reps": int(built["cfg"]["sync_reps"]),
        "sample_rate": float(built["cfg"]["sample_rate"]),
    }


def _golden_readme(cases):
    names = ", ".join(c["name"] for c in cases)
    return """# S0 preamble-only SIC golden vectors

Small, deterministic, model-consistent cross-check vectors for the frozen SIC
core contract (`docs/sic/S0_sic_core_api.md`).  The NumPy reference that
produced them is `tools/sic/sic_reference.py`.

Regenerate deterministically:

```bash
python3 tools/sic/sic_reference.py --emit-golden
python3 tools/sic/sic_reference.py --self-test
```

## Size note

These vectors use a **reduced** `samples_per_symbol` (=127) and only
`16` SYNC repetitions so the files stay small and commit-able.  The real work
grid uses `samples_per_symbol=1016`; the algorithm is SPS-agnostic, so the C++
QA **must read `samples_per_symbol`, `sample_rate`, `sync_reps`, ROI and
thresholds from `meta.json`** (never hard-code 1016).

## Layout

```
golden/
  README.md            this file
  index.json           case list + sha256 of every io.npz
  <case>/
    meta.json          cfg / truth / expected / thresholds / PASS criteria
    io.npz             y, sync, preamble, i, i_hat_expected, out_expected
```

Cases: %s

## io.npz schema (stable)

All arrays are `complex64`; all indices are 0-based.

| array | shape | meaning |
|---|---|---|
| `y` | `(N,)` | received window `s + i + w`; the only signal `estimate_sic` may see |
| `sync` | `(sps,)` | ONE shaped SYNC template for the comm code (detection) |
| `preamble` | `(Lp,)` | FULL comm preamble template (reconstruction) |
| `i` | `(N,)` | **TRUTH** interferer; never pass to `estimate_sic` |
| `i_hat_expected` | `(N,)` | reference `Estimate` reconstruction of `i` |
| `out_expected` | `(N,)` | reference `Estimate` output (`out == y` outside the mask) |

`i` is included so the oracle / NMSE diagnostics can be evaluated; the online
`estimate_sic` function has no truth argument and never reads it.  The QA
harness must call `estimate_sic(y, sync, preamble, cfg)` only.

## meta.json

* `generator` / `truth` — truth-bearing (generator side only).
* `runtime_cfg` — the exact `SicConfig` the reference used.  `mode` is the
  string `"Estimate"` (map it to `SicMode::Estimate`); `path_min_spacing = 0`
  means "derive from the template autocorrelation mainlobe".
* `expected` — expected `detected_start`, `cfo_hz`, `taus` (absolute work
  samples, `start + delay_k`), `gains`.
* `thresholds` — `detect_threshold`, `validate_threshold`.
* `pass_criteria` — per-case bounds used by `--self-test`.

## Fractional-delay model note

The core owns a **fixed band-limited (windowed-sinc, 16-tap Hamming)**
fractional-delay operator.  `frac` is model-consistent (generator uses the same
operator).  `cross_model` uses an **independent band-limited kernel** (4-tap
Blackman) on the generator side, so generator and estimator are not the same
interpolator; it is a declared stress case.

## Comparison tolerance

The reference uses `complex128` internally.  A faithful C++ port will not match
bit-exactly: compare `detected_start` exactly (or +/-1 sample), `cfo_hz` within
a few kHz, and `i_hat_expected` / `out_expected` by NMSE (a small tolerance,
e.g. re-committed self-consistency < 1e-3), and require `out == y` byte-for-byte
outside the estimated support mask.
""" % names


def emit_golden(golden_dir):
    cases = golden_cases()
    index = {"schema_version": SCHEMA_VERSION, "cases": []}
    for case in cases:
        entry = _emit_one(golden_dir, case)
        index["cases"].append(entry)
    with open(os.path.join(golden_dir, "index.json"), "w") as fh:
        json.dump(index, fh, indent=2)
        fh.write("\n")
    with open(os.path.join(golden_dir, "README.md"), "w") as fh:
        fh.write(_golden_readme(cases))
    print("emitted %d cases to %s" % (len(cases), golden_dir))
    for e in index["cases"]:
        print("  %-12s %s  %s" % (e["name"], e["sha256"][:16], e["path"]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--emit-golden", action="store_true")
    parser.add_argument("--golden-dir", default=None)
    args = parser.parse_args(argv)

    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    golden_dir = args.golden_dir or os.path.join(root, GOLDEN_SUBDIR)

    did = False
    rc = 0
    if args.self_test:
        rc = self_test(golden_dir)
        did = True
    if args.emit_golden:
        os.makedirs(golden_dir, exist_ok=True)
        emit_golden(golden_dir)
        did = True
    if not did:
        parser.print_help()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
