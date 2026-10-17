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
import struct
import time

import numpy as np

SCHEMA_VERSION = 1
GOLDEN_SUBDIR = os.path.join("testdata", "sic_preamble", "golden")

# SicTimings fields, same names, same order.
TIMING_NAMES = (
    "detect_coarse", "detect_fine", "cfo", "cir", "path_select",
    "fractional_refine", "matrix_build", "gain_fit", "reconstruct",
    "validate_commit", "core_total",
    # S1 decomposition (sequential; never summed into core_total)
    "estimate_total", "model_prepare", "apply", "backfill", "apply_forward",
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
        # S1: strict 8-SYNC estimation, frozen model, full-nominal cancellation.
        "support_mode": SUPPORT_CONFIGURED_NOMINAL,
        "observed_end_sample": 0,      # 0 => whole window visible (legacy)
        "online_validation_reps": 0,   # 0 => no holdout wait (default)
        "fit_residual_max": 0.8,       # calibration-fixed sanity ceiling
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
        # reference-only extras / S1 diagnostics
        "taus": [],
        "gains": [],
        "support_source": None,
        "fit_residual": 0.0,
        "training_reps_confirmed": 0,
        "observed_end_sample": 0,
        "model_id": 0,
        "model": None,
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
# S1 model: frozen estimate + apply (mirrors estimate_model / apply_model in the
# C++ core, including the STRICT-PREFIX rule).
# ---------------------------------------------------------------------------
SUPPORT_CONFIGURED_NOMINAL = "configured_nominal"
SUPPORT_CONFIRMED_LEGACY = "confirmed_legacy"

_FNV_OFFSET = 1469598103934665603
_FNV_PRIME = 1099511628211


def _resolve_visible_end(n, observed_end):
    """0 => whole window (legacy callers); >0 => min(declared, n); <0 invalid."""
    if observed_end == 0:
        return int(n)
    if observed_end < 0:
        return -1
    return int(min(observed_end, n))


def _blank_model():
    return {
        "valid": False,
        "state": STATE_FAILED,
        "status": "not_estimated",
        "reason": None,
        "model_id": 0,
        "origin": -1,
        "cfo_hz": 0.0,
        "n_paths": 0,
        "taus": [],            # absolute work samples
        "gains": [],           # complex, phase_reference_sample frame
        "gain_condition": 0.0,
        "fit_residual": 0.0,
        "training_lo": -1,
        "training_hi": -1,
        "observed_end_sample": 0,
        "estimation_ready_sample": -1,
        "predicted_support_lo": -1,
        "predicted_support_hi": -1,
        "support_source": "none",
        "sub_lo": -1,
        "sub_hi": -1,
        "training_reps_confirmed": 0,
        "nominal_sync_reps": 0,
        "samples_per_symbol": 0,
        "sample_rate": 0.0,
        "phase_reference_sample": 0,
    }


def _model_id(model):
    """FNV-1a-64 over the FROZEN fields (same field set as the C++ core).

    The byte layout follows the C++ hash (little-endian scalars, float32 gains),
    but this is a per-implementation identity: use the FIELD comparison, not this
    number, as the language-independent oracle.
    """
    h = _FNV_OFFSET

    def mix(b):
        nonlocal h
        for byte in b:
            h ^= byte
            h = (h * _FNV_PRIME) & 0xFFFFFFFFFFFFFFFF

    mix(struct.pack("<Q", 0x53314D4F44454C31))          # "S1MODEL1"
    mix(struct.pack("<Q", int(model.get("template_hash", 0))))
    mix(struct.pack("<Q", int(model.get("config_hash", 0))))
    mix(struct.pack("<q", int(model["origin"])))
    mix(struct.pack("<d", float(model["cfo_hz"])))
    mix(struct.pack("<Q", int(model["n_paths"])))
    taus = list(model["taus"]) + [0.0] * (16 - len(model["taus"]))
    gains = list(model["gains"]) + [0.0] * (16 - len(model["gains"]))
    for k in range(16):
        mix(struct.pack("<d", float(taus[k])))
        mix(struct.pack("<f", float(np.real(gains[k]))))
        mix(struct.pack("<f", float(np.imag(gains[k]))))
    mix(struct.pack("<d", float(model["gain_condition"])))
    mix(struct.pack("<d", float(model["fit_residual"])))
    for key in ("training_lo", "training_hi", "observed_end_sample",
                "estimation_ready_sample", "predicted_support_lo",
                "predicted_support_hi"):
        mix(struct.pack("<q", int(model[key])))
    mix(str(model["support_source"]).encode("ascii") + b"\x00")
    mix(struct.pack("<q", int(model["sub_lo"])))
    mix(struct.pack("<q", int(model["sub_hi"])))
    mix(struct.pack("<Q", int(model["training_reps_confirmed"])))
    mix(struct.pack("<Q", int(model["nominal_sync_reps"])))
    mix(struct.pack("<Q", int(model["samples_per_symbol"])))
    mix(struct.pack("<d", float(model["sample_rate"])))
    mix(struct.pack("<q", int(model["phase_reference_sample"])))
    return h


def _config_hash(cfg):
    h = _FNV_OFFSET

    def mix(b):
        nonlocal h
        for byte in b:
            h ^= byte
            h = (h * _FNV_PRIME) & 0xFFFFFFFFFFFFFFFF

    def u64(v):
        mix(struct.pack("<Q", int(v) & 0xFFFFFFFFFFFFFFFF))

    mix(struct.pack("<Q", 0x5331434F4E464947))          # "S1CONFIG"
    u64(1 if cfg.get("support_mode", SUPPORT_CONFIGURED_NOMINAL) ==
        SUPPORT_CONFIRMED_LEGACY else 0)
    u64(cfg["comm_code"])
    u64(cfg["sync_reps"])
    u64(cfg["samples_per_symbol"])
    mix(struct.pack("<d", float(cfg["sample_rate"])))
    u64(cfg["k_paths"])
    u64(cfg["fit_reps"])
    u64(cfg["validation_reps"])
    u64(cfg.get("online_validation_reps", 0))
    mix(struct.pack("<q", int(cfg["roi_lo"])))
    mix(struct.pack("<q", int(cfg["roi_hi"])))
    mix(struct.pack("<q", int(cfg.get("observed_end_sample", 0))))
    u64(cfg["coarse_stride"])
    u64(cfg["path_min_spacing"])
    u64(cfg["post_tail"])
    u64(1 if cfg["enable_fractional"] else 0)
    u64(1 if cfg["enable_cfo"] else 0)
    mix(struct.pack("<d", float(cfg["detect_threshold"])))
    mix(struct.pack("<d", float(cfg.get("fit_residual_max", 0.8))))
    return h


def _template_hash(sync, preamble):
    h = _FNV_OFFSET

    def mix(b):
        nonlocal h
        for byte in b:
            h ^= byte
            h = (h * _FNV_PRIME) & 0xFFFFFFFFFFFFFFFF

    mix(struct.pack("<Q", 0x533154504C415445))          # "S1TPLATE"
    mix(struct.pack("<Q", len(sync)))
    mix(np.asarray(sync, dtype=np.complex64).tobytes())
    mix(struct.pack("<Q", len(preamble)))
    mix(np.asarray(preamble, dtype=np.complex64).tobytes())
    return h


def _reconstruct_region(model, lo, m, preamble):
    """i_hat over the ABSOLUTE window [lo, lo+m) from a FROZEN model."""
    if m <= 0:
        return np.zeros(0, dtype=np.complex128)
    acc = np.zeros(m, dtype=np.complex128)
    pre = np.asarray(preamble, dtype=np.complex128)
    for tau, g in zip(model["taus"][:model["n_paths"]],
                      model["gains"][:model["n_paths"]]):
        # dst[i] += (g * preamble)(i - (tau - lo))
        _delay_into(acc, pre * complex(g), float(tau) - float(lo), "core")
    w = 2.0 * np.pi * float(model["cfo_hz"]) / float(model["sample_rate"])
    p = np.arange(lo, lo + m, dtype=np.float64) - float(model["phase_reference_sample"])
    return acc * np.exp(1j * w * p)


def _estimate_model(y, sync, preamble, cfg):
    """S1 estimation stage: reads ONLY y[0, visible_end).  No truth argument."""
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
    fit_res_max = float(cfg.get("fit_residual_max", 0.8))
    support_mode = cfg.get("support_mode", SUPPORT_CONFIGURED_NOMINAL)
    online_reps = int(cfg.get("online_validation_reps", 0))
    legacy = (support_mode == SUPPORT_CONFIRMED_LEGACY)

    result = _blank_result()
    model = _blank_model()
    times = result["timings"]
    model["nominal_sync_reps"] = int(cfg["sync_reps"])
    model["samples_per_symbol"] = sps
    model["sample_rate"] = fs
    model["template_hash"] = _template_hash(sync, preamble)
    model["config_hash"] = _config_hash(cfg)

    def terminal(state, status, reason):
        model["state"] = state
        model["status"] = status
        model["reason"] = reason
        model["valid"] = (state == STATE_COMMITTED)
        model["model_id"] = _model_id(model)
        result["state"] = state
        result["committed"] = (state == STATE_COMMITTED)
        result["ambiguous"] = (state == STATE_AMBIGUOUS)
        result["detected_start"] = model["origin"]
        result["support_lo"] = model["predicted_support_lo"]
        result["support_hi"] = model["predicted_support_hi"]
        result["support_source"] = model["support_source"]
        result["model_id"] = model["model_id"]
        result["fit_residual"] = model["fit_residual"]
        result["training_reps_confirmed"] = model["training_reps_confirmed"]
        result["bypass_reason"] = reason
        result["reject_reason"] = reason
        times["core_total"] = _us(t_core)

    if support_mode not in (SUPPORT_CONFIGURED_NOMINAL, SUPPORT_CONFIRMED_LEGACY):
        terminal(STATE_FAILED, "failed", "invalid_support_mode")
        return model, result
    if int(cfg.get("observed_end_sample", 0)) < 0:
        terminal(STATE_FAILED, "failed", "invalid_observed_end")
        return model, result
    if not (0.0 < fit_res_max <= 1.0):
        terminal(STATE_FAILED, "failed", "invalid_fit_residual_max")
        return model, result
    if online_reps > val_reps:
        terminal(STATE_FAILED, "failed", "online_validation_reps_exceeds_prepared")
        return model, result

    vis = _resolve_visible_end(N, int(cfg.get("observed_end_sample", 0)))
    model["observed_end_sample"] = vis
    result["observed_end_sample"] = vis

    confirm_reps = fit_reps + online_reps if legacy else fit_reps

    roi_lo = int(cfg["roi_lo"])
    roi_hi = int(cfg["roi_hi"])
    if roi_hi <= roi_lo:
        roi_lo, roi_hi = 0, vis
    roi_hi = min(roi_hi, vis - Ls + 1)
    roi_lo = max(roi_lo, 0)
    result["roi_samples"] = max(0, roi_hi - roi_lo)
    stride = max(1, int(cfg["coarse_stride"]))
    sync_norm = float(np.linalg.norm(sync))

    def norm_at(d):
        # STRICT PREFIX: the whole row must be visible.
        if d < 0 or d + Ls > vis:
            return 0j, 0.0
        seg = y[d:d + Ls]
        c = np.vdot(sync, seg)
        e = float(np.linalg.norm(seg))
        nrm = abs(c) / (e * sync_norm) if e > 0.0 else 0.0
        return c, nrm

    t = time.perf_counter()
    coarse_best = (-1, 0.0)
    for d in range(roi_lo, roi_hi, stride):
        _, nrm = norm_at(d)
        result["coarse_candidates"] += 1
        result["correlations"] += 1
        if nrm > coarse_best[1]:
            coarse_best = (d, nrm)
    times["detect_coarse"] = _us(t)

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
    t = time.perf_counter()
    if detected_start >= 0:
        origin = int(detected_start)
        while origin - sps >= max(roi_lo, 0):
            d = origin - sps
            nrm = norm_at(d)[1]
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
        terminal(STATE_BYPASSED, "bypassed", "below_detect_threshold")
        return model, result
    result["detected"] = True
    result["detected_start"] = int(detected_start)
    model["origin"] = int(detected_start)
    model["phase_reference_sample"] = int(detected_start)

    # repetition confirmation, training span only (rows fully inside the prefix)
    rep_norms, rep_corr = [], []
    confirmed = 0
    training_confirmed = 0
    for k in range(confirm_reps):
        d = detected_start + k * sps
        if d + Ls <= vis:
            c, nrm = norm_at(d)
        else:
            c, nrm = 0j, 0.0
        rep_norms.append(nrm)
        rep_corr.append(c)
        result["correlations"] += 1
        if nrm >= detect_thr:
            confirmed += 1
            if k < fit_reps:
                training_confirmed += 1
    model["training_reps_confirmed"] = training_confirmed
    result["training_reps_confirmed"] = training_confirmed
    if confirmed < 2:
        terminal(STATE_AMBIGUOUS, "ambiguous", "repetition_not_confirmed")
        return model, result
    if training_confirmed < fit_reps:
        result["partial"] = True

    t = time.perf_counter()
    f_est = 0.0
    if cfg["enable_cfo"]:
        phis = []
        k_lo = 1 if fit_reps >= 3 else 0
        for k in range(k_lo, fit_reps - 1):
            if rep_norms[k] >= detect_thr and rep_norms[k + 1] >= detect_thr:
                phis.append(np.angle(rep_corr[k + 1] * np.conj(rep_corr[k])))
        if phis:
            z = np.mean(np.exp(1j * np.asarray(phis)))
            f_est = float(np.angle(z)) / (2.0 * np.pi * sps / fs)
    times["cfo"] = _us(t)
    result["cfo_hz"] = float(f_est)
    model["cfo_hz"] = float(f_est)
    w = 2.0 * np.pi * f_est / fs

    fit_lo = int(detected_start)
    fit_end_ideal = fit_lo + fit_reps * sps
    fit_end = min(fit_end_ideal, vis)
    Mfit = max(0, fit_end - fit_lo)
    model["training_lo"] = fit_lo
    model["training_hi"] = fit_end
    model["estimation_ready_sample"] = fit_end

    t = time.perf_counter()
    dmin, dmax = -4, post_tail + 4
    cir = np.zeros(dmax - dmin, dtype=np.complex128)
    for idx, d in enumerate(range(dmin, dmax)):
        pos = detected_start + d
        # STRICT: a tap may not read past the training span or the prefix.
        if pos >= 0 and pos + Ls <= fit_end:
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
        terminal(STATE_FAILED, "failed", "no_paths")
        return model, result

    # support: predicted nominal vs confirmed legacy
    if legacy:
        model["predicted_support_lo"] = max(0, detected_start)
        model["predicted_support_hi"] = min(
            N, detected_start + confirmed * sps + post_tail)
        model["support_source"] = "confirmed_repetition_legacy"
        if confirmed < confirm_reps:
            result["partial"] = True
    else:
        model["predicted_support_lo"] = max(0, detected_start)
        model["predicted_support_hi"] = min(
            N, detected_start + int(cfg["sync_reps"]) * sps + post_tail)
        model["support_source"] = "configured_nominal_prediction"
    result["support_lo"] = model["predicted_support_lo"]
    result["support_hi"] = model["predicted_support_hi"]

    # permitted cancellation region + sub_roi overlap gate
    sub_roi_lo = int(cfg.get("sub_roi_lo", 0))
    sub_roi_hi = int(cfg.get("sub_roi_hi", 0))
    if sub_roi_hi > sub_roi_lo:
        lo = max(model["predicted_support_lo"], sub_roi_lo)
        hi = min(model["predicted_support_hi"], sub_roi_hi)
        overlap = max(0, hi - lo)
        result["sub_overlap"] = overlap
        if overlap < max(1, int(cfg.get("sub_min_overlap", 0))):
            terminal(STATE_BYPASSED, "bypassed", "outside_sensing_roi")
            return model, result
        sub_lo, sub_hi = lo, hi
    else:
        result["sub_overlap"] = -1
        roi_set = int(cfg["roi_hi"]) > int(cfg["roi_lo"])
        if legacy and roi_set:
            mask_hi = roi_hi  # S0 clipped search ROI
        elif roi_set:
            mask_hi = min(int(cfg["roi_hi"]), N)
        else:
            mask_hi = N
        sub_lo = max(0, model["predicted_support_lo"], int(cfg["roi_lo"]))
        sub_hi = min(mask_hi, model["predicted_support_hi"])
    model["sub_lo"] = sub_lo
    model["sub_hi"] = max(sub_lo, sub_hi)

    if Mfit < len(delays):
        terminal(STATE_FAILED, "failed", "training_window_unavailable")
        return model, result

    def shifted(offset):
        # dst[i] = preamble(i - offset) for i in [0, Mfit): the exact column the
        # C++ builds over the training rows.
        out = np.zeros(Mfit, dtype=np.complex128)
        _delay_into(out, preamble, offset, "core")
        return out

    Wf = np.arange(fit_lo, fit_lo + Mfit)
    ramp = np.exp(1j * w * (Wf - model["phase_reference_sample"]))

    def col_at(tau):
        return shifted(float(tau) - fit_lo) * ramp

    t = time.perf_counter()
    taus = [float(detected_start) + d for d in delays]
    if cfg["enable_fractional"]:
        A0 = np.stack([col_at(tau) for tau in taus], axis=1)
        Q, R = np.linalg.qr(A0, mode="reduced")
        h = np.linalg.solve(R, Q.conj().T @ y[Wf])
        offsets = np.arange(-0.5, 0.5 + 1e-9, 0.05)
        for k in range(len(taus)):
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

    t = time.perf_counter()
    A = np.stack([col_at(tau) for tau in taus], axis=1)
    times["matrix_build"] = _us(t)
    t = time.perf_counter()
    Q, R = np.linalg.qr(A, mode="reduced")
    h = np.linalg.solve(R, Q.conj().T @ y[Wf])
    cond = float(np.linalg.cond(A))
    times["gain_fit"] = _us(t)

    model["n_paths"] = len(taus)
    model["taus"] = [float(x) for x in taus]
    model["gains"] = [complex(np.complex64(x)) for x in h]
    model["gain_condition"] = cond
    result["gain_condition"] = cond
    result["n_paths"] = len(taus)
    result["taus"] = [float(x) for x in taus]
    result["gains"] = [complex(x) for x in h]

    t = time.perf_counter()
    ih = _reconstruct_region(model, fit_lo, Mfit, preamble)
    e_num = float(np.sum(np.abs(y[fit_lo:fit_lo + Mfit] - ih) ** 2))
    e_den = float(np.sum(np.abs(y[fit_lo:fit_lo + Mfit]) ** 2))
    model["fit_residual"] = (e_num / e_den) if e_den > 0.0 else 0.0
    result["fit_residual"] = model["fit_residual"]
    times["model_prepare"] = _us(t)

    t = time.perf_counter()
    commit = True
    reject = None
    finite_ok = (np.isfinite(cond) and np.isfinite(f_est) and
                 np.isfinite(model["fit_residual"]) and
                 all(np.isfinite(x) for x in h))
    if not finite_ok:
        commit, reject = False, "non_finite_model"
    elif not cond < 1e6:
        commit, reject = False, "gain_condition_too_high"
    elif (not legacy) and not (model["fit_residual"] <= fit_res_max):
        commit, reject = False, "fit_residual_high"
    if commit and online_reps > 0:
        hstart = detected_start + fit_reps * sps
        hend = hstart + online_reps * sps
        if hend > vis:
            result["residual_corr"] = float("nan")
            commit, reject = False, "validation_window_unavailable"
        else:
            ih = _reconstruct_region(model, hstart, hend - hstart, preamble)
            r = y[hstart:hend] - ih
            p_col = _reconstruct_region(
                {**model, "n_paths": 1, "taus": [float(detected_start)],
                 "gains": [1.0 + 0j]}, hstart, hend - hstart, preamble)
            den = float(np.linalg.norm(r) * np.linalg.norm(p_col))
            rc = abs(np.vdot(p_col, r)) / den if den > 0 else 1.0
            result["residual_corr"] = float(rc)
            if not rc < validate_thr:
                commit, reject = False, "holdout_residual_high"
    elif online_reps == 0:
        result["residual_corr"] = float("nan")
    times["validate_commit"] = _us(t)

    if not commit:
        terminal(STATE_BYPASSED, "bypassed", reject or "not_committed")
        return model, result
    terminal(STATE_COMMITTED, "model_ready", None)
    return model, result


def _apply_model(model, y, offset, preamble, cfg):
    """Apply a FROZEN model to the chunk y (absolute start `offset`)."""
    y = np.asarray(y)
    out = y.astype(np.complex64).copy()
    ihat = np.zeros(len(y), dtype=np.complex64)
    if not model["valid"] or model["n_paths"] == 0:
        return out, ihat
    plen = len(preamble)
    taus = model["taus"][:model["n_paths"]]
    union_lo = int(math.floor(min(taus)))
    union_hi = max(int(math.ceil(t)) + plen for t in taus)
    g_lo = min(union_lo, model["predicted_support_lo"])
    g_hi = max(union_hi, model["predicted_support_hi"])
    chunk_lo = int(offset)
    chunk_hi = chunk_lo + len(y)
    lo = max(g_lo, chunk_lo)
    hi = min(g_hi, chunk_hi)
    if hi <= lo:
        return out, ihat
    ih = _reconstruct_region(model, lo, hi - lo, preamble)
    ihat[lo - chunk_lo:hi - chunk_lo] = ih.astype(np.complex64)
    slo = max(lo, int(model["sub_lo"]))
    shi = min(hi, int(model["sub_hi"]))
    if shi > slo:
        out[slo - chunk_lo:shi - chunk_lo] = (
            y[slo - chunk_lo:shi - chunk_lo] -
            ih[slo - lo:shi - lo]).astype(np.complex64)
    return out, ihat


def estimate_sic(y, sync, preamble, cfg):
    """Mirror of the C++ ``sic_one`` for SicMode::Estimate (S1 semantics).

    Estimation reads ONLY ``y[0, observed_end_sample)`` (0 => whole window);
    the frozen model is then applied to the WHOLE window in two passes (backfill
    of the observed prefix, then forward), exactly like the C++ core.
    """
    t_core = time.perf_counter()
    y = np.asarray(y, dtype=np.complex128)
    N = len(y)
    model, result = _estimate_model(y, sync, preamble, cfg)
    if not model["valid"]:
        # Every non-committed path is a bit-exact bypass.
        result["i_hat"] = np.zeros(N, dtype=np.complex128)
        result["out"] = np.asarray(y, dtype=np.complex64).copy()
        result["timings"]["core_total"] = _us(t_core)
        return result
    vis = int(model["observed_end_sample"])
    out = np.asarray(y, dtype=np.complex64).copy()
    ihat = np.zeros(N, dtype=np.complex128)
    for off, ln in ((0, vis), (vis, N - vis)):
        if ln <= 0:
            continue
        o, ih = _apply_model(model, y[off:off + ln], off, preamble, cfg)
        out[off:off + ln] = o
        ihat[off:off + ln] = ih
    result["out"] = out
    result["i_hat"] = ihat
    result["model"] = model
    result["timings"]["core_total"] = _us(t_core)
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
        # S1: the CFO comes from the 8 TRAINING symbols only (6 phase pairs).  At
        # sps=127 / SNR=30 dB the measured 8-symbol CFO standard deviation is
        # ~11 kHz (C++ and reference agree to 0.1 Hz), so the S0 2 kHz tolerance
        # no longer applies to this reduced-length vector.  The long-symbol work
        # grid (sps=1016) has the same phase noise on a 8x longer baseline.
        pass_criteria=dict(oracle_nmse_max=1e-6, estimate_nmse_max=0.05,
                           suppression_min_db=12.0, detected_start_tol=2,
                           cfo_hz_tol=15000.0, require_commit=True),
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
                           cfo_hz_tol=15000.0, require_commit=True),
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

    # S1: strict-prefix estimate, frozen model, full-nominal apply.
    _s1_self_test(failures)

    # committed golden vectors must reproduce (schema + hashes + outputs)
    if golden_dir and os.path.isdir(golden_dir):
        _check_golden(golden_dir, failures)
    else:
        print("%-5s %-12s (golden dir absent)" % ("SKIP", "golden"))

    print("== %s ==" % ("ALL PASS" if not failures
                        else "FAILURES: " + " | ".join(failures)))
    return 1 if failures else 0


def _models_equal(a, b):
    """Compare EVERY frozen-model field (not just n_paths/valid)."""
    keys = ("valid", "state", "status", "reason", "model_id", "origin",
            "cfo_hz", "n_paths", "gain_condition", "fit_residual",
            "training_lo", "training_hi", "observed_end_sample",
            "estimation_ready_sample", "predicted_support_lo",
            "predicted_support_hi", "support_source", "sub_lo", "sub_hi",
            "training_reps_confirmed", "nominal_sync_reps",
            "samples_per_symbol", "sample_rate", "phase_reference_sample")
    for k in keys:
        if a[k] != b[k]:
            return False, k
    if len(a["taus"]) != len(b["taus"]):
        return False, "taus"
    if any(x != y for x, y in zip(a["taus"], b["taus"])):
        return False, "taus"
    if any(complex(x) != complex(y) for x, y in zip(a["gains"], b["gains"])):
        return False, "gains"
    return True, ""


def _s1_self_test(failures):
    """S1: strict-prefix 8-SYNC estimate + frozen model + full-nominal apply."""
    sps = 127
    reps = 64

    def run(isr_db, snr_db):
        spec = {
            "name": "s1_64",
            "description": "64-SYNC full-nominal cancellation from an 8-SYNC prefix",
            "paths": [(0.0, 1.0 + 0j)],
            "params": {"sps": sps, "reps": reps, "start": 64,
                       "N": reps * sps + 64 + 4 * sps, "isr_db": float(isr_db),
                       "snr_db": float(snr_db), "code": 10},
            "fit_reps": 8, "validation_reps": 8, "post_tail": 64,
            "detect_threshold": 0.40, "validate_threshold": 0.25,
        }
        built = build_case(spec)
        y, sync, pre = built["y"], built["sync"], built["preamble"]
        start = int(built["truth"]["start"])
        oe = start + 8 * sps
        cfg = dict(built["cfg"])
        cfg["observed_end_sample"] = oe
        est = estimate_sic(y, sync, pre, cfg)
        i = np.asarray(built["i"])
        out = np.asarray(est["out"])

        def supp(lo, hi):
            ei = float(np.sum(np.abs(i[lo:hi]) ** 2))
            er = float(np.sum(np.abs(out[lo:hi] -
                                     (y[lo:hi] - i[lo:hi])) ** 2))
            return 10.0 * math.log10(ei / max(er, 1e-30))

        return built, cfg, est, y, sync, pre, start, oe, supp

    # Mechanism check on a clean estimation environment: the frozen model must
    # reach the full-nominal tail (>20 dB), which proves the support is not
    # truncated to the confirmed/validated repetitions.
    #
    # Reality check (measured, printed, not asserted as a target): at ISR=10 dB
    # and SNR=30 dB the SAME 8 training symbols carry a CFO phase-noise floor
    # from the co-channel sensing signal; over the remaining 56 symbols that
    # phase error accumulates and caps the far-tail depth.  That is a property of
    # 8-symbol estimation, not of the support length -- which is why it is
    # reported next to it.
    built_hi, cfg_hi, est_hi, y_hi, _, _, start_hi, oe_hi, supp_hi = run(40.0, 60.0)
    built, cfg, est, y, sync, pre, start, oe, supp = run(10.0, 30.0)
    tail4_hi, ind_hi = supp_hi(start_hi + 60 * sps, start_hi + 64 * sps), \
        supp_hi(oe_hi, start_hi + reps * sps)
    tail4, ind = supp(start + 60 * sps, start + 64 * sps), \
        supp(oe, start + reps * sps)
    ok_commit = est["committed"] and est_hi["committed"]

    # 1. prefix isolation: zero / randomise the tail -> identical frozen model.
    y2 = np.array(y, copy=True)
    y2[oe:] = 0
    est2 = estimate_sic(y2, sync, pre, cfg)
    same_zero, field_zero = _models_equal(est["model"], est2["model"])
    y3 = np.array(y, copy=True)
    rng = np.random.default_rng(5)
    y3[oe:] = (rng.standard_normal(len(y) - oe) +
               1j * rng.standard_normal(len(y) - oe)).astype(np.complex64)
    est3 = estimate_sic(y3, sync, pre, cfg)
    same_rnd, field_rnd = _models_equal(est["model"], est3["model"])
    if not (same_zero and same_rnd):
        failures.append("s1 prefix isolation (%s/%s)" % (field_zero, field_rnd))

    # 2. full-nominal support, not 8/16/60 symbols.
    sup_len = est["support_hi"] - est["support_lo"]
    want = reps * sps + 64
    ok_sup = (est["support_source"] == "configured_nominal_prediction"
              and sup_len == want)
    if not ok_sup:
        failures.append("s1 support len=%d want=%d src=%s"
                        % (sup_len, want, est["support_source"]))

    # 3. chunk invariance: whole window vs random chunks, BIT-identical.
    model = est["model"]
    out_w = np.asarray(est["out"]).copy()
    ih_w = np.asarray(est["i_hat"]).copy()
    out_c = np.zeros(len(y), dtype=np.complex64)
    ih_c = np.zeros(len(y), dtype=np.complex128)
    rng = np.random.default_rng(7)
    pos = 0
    while pos < len(y):
        ln = int(min(len(y) - pos, 1 + int(rng.integers(0, 900))))
        o, ih = _apply_model(model, np.asarray(y[pos:pos + ln]), pos, pre, cfg)
        out_c[pos:pos + ln] = o
        ih_c[pos:pos + ln] = ih
        pos += ln
    chunk_ok = (np.array_equal(out_c, out_w) and np.array_equal(ih_c, ih_w))
    if not chunk_ok:
        failures.append("s1 chunk invariance")

    # 4. bypass: an untrusted (invalid) frozen model must not touch the output.
    bad = _blank_model()
    o, ih = _apply_model(bad, np.asarray(y), 0, pre, cfg)
    bypass_ok = np.array_equal(o, np.asarray(y).astype(np.complex64)) and not ih.any()
    if not bypass_ok:
        failures.append("s1 untrusted-model bypass")

    if not (tail4_hi > 20.0 and ind_hi > 20.0):
        failures.append("s1 high-SNR tail4=%.2f indep=%.2f dB"
                        % (tail4_hi, ind_hi))
    if not (tail4 > 0.0 and ind > 0.0):
        failures.append("s1 snr30 tail4=%.2f indep=%.2f dB" % (tail4, ind))

    ok = (ok_commit and same_zero and same_rnd and ok_sup and
          tail4_hi > 20.0 and ind_hi > 20.0 and tail4 > 0.0 and ind > 0.0 and
          chunk_ok and bypass_ok)
    print("%-5s %-12s support=%d src=%s fit_res=%.4f | ISR40/SNR60 tail4=%.2f "
          "indep=%.2f dB | ISR10/SNR30 tail4=%.2f indep=%.2f dB cfo=%.0f Hz "
          "(8-symbol CFO limited) | prefix_iso=%s chunk=%s bypass=%s"
          % ("PASS" if ok else "FAIL", "s1", sup_len,
             est["support_source"], est["fit_residual"], tail4_hi, ind_hi,
             tail4, ind, est["model"]["cfo_hz"], same_zero and same_rnd,
             chunk_ok, bypass_ok))
    return ok


def _check_golden(golden_dir, failures):
    """Verify committed golden vectors load, hash, and reproduce.

    S0-era vectors (no ``support_mode`` key) are replayed in the EXPLICIT legacy
    mode they were generated with, and the i_hat/out comparison uses the widened
    NMSE tolerance documented in the C++ QA: their stored arrays predate the
    strict 8-SYMBOL CFO estimator (the only stage S1 changed inside the estimate).
    Everything else -- hash, array set, detected mask and byte-exactness outside
    the mask -- is still compared strictly.
    """
    index = json.load(open(os.path.join(golden_dir, "index.json")))
    bad = []
    for entry in index["cases"]:
        io_path = os.path.join(golden_dir, entry["path"])
        meta = json.load(open(os.path.join(
            golden_dir, entry["name"], "meta.json")))
        z = np.load(io_path)
        h = _sha256_file(io_path)
        runtime_cfg = dict(meta["runtime_cfg"])
        legacy_replay = "support_mode" not in runtime_cfg
        if legacy_replay:
            runtime_cfg["support_mode"] = SUPPORT_CONFIRMED_LEGACY
            runtime_cfg["online_validation_reps"] = int(
                runtime_cfg["validation_reps"])
        est = estimate_sic(z["y"], z["sync"], z["preamble"], runtime_cfg)
        tol_ih = 5e-3 if legacy_replay else 1e-5
        tol_out = 2e-2 if legacy_replay else 1e-5
        nm_ih = nmse(z["i_hat_expected"], est["i_hat"])
        nm_out = nmse(z["out_expected"], est["out"])
        y = np.asarray(z["y"], dtype=np.complex64)
        outside_bad = int(np.sum((z["out_expected"] == y) &
                                 (est["out"] != y)))
        ok = (h == entry["sha256"]
              and sorted(z.files) == sorted(["y", "sync", "preamble", "i",
                                             "i_hat_expected", "out_expected"])
              and nm_ih <= tol_ih
              and nm_out <= tol_out
              and outside_bad == 0)
        if not ok:
            bad.append(entry["name"])
            failures.append("golden %s: hash/output mismatch" % entry["name"])
        print("      golden %-12s legacy=%d nmse(i_hat)=%.3e nmse(out)=%.3e "
              "outside-mask-bad=%d" % (entry["name"], legacy_replay, nm_ih,
                                       nm_out, outside_bad))
    print("%-5s %-12s %d cases, hash+arrays+tolerance verified"
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
