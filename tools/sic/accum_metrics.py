#!/usr/bin/env python3
"""S2-A fixed-operator accumulation metrics (NumPy reference + QA).

Scope
-----
This module implements ONLY the *offline* fixed-operator evaluation layer of
S2-A (see ``docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`` section 3.1):

    H_x[r, t] = sum_m  y_x[origin + r*SPS + t + m] * conj(code9[m]) / E_code

for the three paired branches ``x in {clean, raw, sic}`` on the 998.4 MS/s work
grid, and the accumulation / residual / coherence / target metrics derived from
it.  It is deliberately:

* **linear and input-only**: one fixed template, one fixed time origin, one
  fixed tap grid, one fixed gain normalisation and NO phase rotation, so that
  ``H_x`` is a linear functional of the input IQ.  Nothing is picked per branch.
* **truth-free**: the origin is the public nominal sensing origin (``pre``),
  the template is the public code-9 operator; the injected comm start / CFO /
  paths / interferer IQ never enter.
* **not a GNU Radio receiver**: the receiver layer is a separate result
  (``receiver_N_sweep`` is a deliberately unimplemented/limited sweep here).

It contains no GNU Radio or pybind dependency, so the independent verifier can
import it without a build tree.  The symmetric independent re-derivation used by
``tools/sic/verify_s2a.py`` re-implements the small amount of algebra it needs
rather than trusting this module's helpers.

Conventions (frozen in ``testdata/sic_accum/operator.json``)
-----------------------------------------------------------
* ``average`` convention: ``H_x,N = (1/N) sum_{r in R_N} H_x[r]``.
  The ``sum`` counterpart is a plain factor ``N`` and is recorded, not
  re-declared: ``H_sum = N * H_avg``.
* residual: ``E_raw,N = H_raw,N - H_clean,N``, ``E_sic,N = H_sic,N - H_clean,N``.
* CIR SIC gain: ``C_CIR(N) = 10log10(||E_raw,N||^2 / ||E_sic,N||^2)``.
* accumulation gain (NOT an SIC gain): ``G(N) = 10log10(P(1)/P(N))`` on the
  nested same-start repetition sets.
* zero numerator / denominator are reported as ``+Inf`` / ``-Inf`` / ``N/A``
  (never replaced by an ``eps`` floor and never clipped to a "good" value).
  ``*_clipped`` mirrors exist for plotting only, with the clip bounds recorded.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Frozen geometry / domain constants (mirror gr-uwb/apps/sic_preamble_sim.py)
# ---------------------------------------------------------------------------
WORK_HZ = 998400000.0
SPS = 1016                 # work samples per Ipatov SYNC symbol
SYNC_REPS = 64
SENSING_CODE = 9
COMM_CODE = 10
SFD_MODE = "4z2"
#: Public nominal sensing origin (``llround(2.0 us * 998.4 MHz)``), identical to
#: every generator instance's ``pre_guard``.  Kept here so pure-NumPy QA can use
#: the frozen geometry without a build tree; the app asserts it matches.
PRE_GUARD_REF = 1997
PREAMBLE_CODE_LEN = 127
SPREADING_FACTOR = 4
SAMPLES_PER_PULSE = 2
CHIP_SAMPLE_STRIDE = SPREADING_FACTOR * SAMPLES_PER_PULSE   # 8

#: Fixed-operator tap grid, in work samples relative to ``origin``.
TAP_LO = -32
TAP_HI = 128

#: ROIs in the fixed-operator tap domain (work samples relative to ``origin``).
#: The weak sensing path is at delay 37 (gain 0.25) on top of the delay-0 unit
#: path.  The public pulse shape is causal with a centre tap at +2 samples, so
#: in this operator (and in the receiver's tap domain) the direct peak lands on
#: offset 2 and the weak-target peak on offset 39.  The R04 frozen receiver ROI
#: [51,60) / target tap 55 is exactly offset [35,44) / tap 39 shifted by
#: ``cir_pre = 16``: receiver_tap = fixed_offset + 16.  The ROI is re-verified on
#: calibration-only seeds at run start (and is NOT re-derived from any SIC
#: result).
TARGET_LO = 35
TARGET_HI = 44
TARGET_OFFSET = 39
BG_LO = 64
BG_HI = 100
DIRECT_LO = -8
DIRECT_HI = 8

#: Receiver-layer (``uwb.radar_cir_estimator``) tap-domain ROIs.  These are the
#: R04 frozen values from ``testdata/sic_preamble/cases/target_roi.json``
#: (target ROI [51,60), target tap 55, main peak 18).  With ``cir_pre = 16`` the
#: receiver tap domain is the fixed-operator offset domain shifted by exactly
#: ``cir_pre``, so the same physical peak is offset 39 here and tap 55 there.
RX_TARGET_ROI = (51, 60)
RX_TARGET_TAP = 55
RX_BACKGROUND_TAPS = (80, 116)     # [64,100) offsets + cir_pre 16
RX_TAP_OFFSET = 16

#: Repetition sets.
FULL_N = (1, 2, 4, 8, 16, 32, 64)
INDEP_START = 8
INDEP_N = (1, 2, 4, 8, 16, 32, 56)

#: Plot clip bounds (documentation only; the numeric values are never clipped).
PLOT_FLOOR_DB = -80.0
PLOT_CEIL_DB = 140.0
#: Iteration cap for the per-window target-ROI calibration search.
CAL_SEEDS = (11000000, 11000001, 11000002, 11000003, 11000004)


# ---------------------------------------------------------------------------
# Reference operator construction (pure NumPy; shared with the app)
# ---------------------------------------------------------------------------
def sampled_code_from_chips(chips: np.ndarray,
                            sps: int = SPS) -> np.ndarray:
    """Build the bare (unshaped) HRP sampled code from the ternary chips.

    Mirrors ``UWB_demodulation/+uwbdecoder/buildUwbReference.m``:

        spread_code[0::SPREADING_FACTOR] = chips
        sampled_code[0::SAMPLES_PER_PULSE] = spread_code

    so chip ``j`` lands on work sample ``8*j`` and the vector length is ``sps``.
    """
    chips = np.asarray(chips, dtype=np.float64).ravel()
    spread = np.zeros(int(chips.size) * SPREADING_FACTOR, dtype=np.float64)
    spread[0::SPREADING_FACTOR] = chips
    sampled = np.zeros(int(spread.size) * SAMPLES_PER_PULSE, dtype=np.float64)
    sampled[0::SAMPLES_PER_PULSE] = spread
    if sampled.size != int(sps):
        raise ValueError("sampled code length %d != sps %d"
                         % (sampled.size, sps))
    return sampled.astype(np.complex128)


def code_energy(code: np.ndarray) -> float:
    """Fixed correlation normaliser ``sum |code|^2`` (== 64 for a full code)."""
    c = np.asarray(code, dtype=np.complex128)
    return float(np.real(np.vdot(c, c)))


def recovery_design_matrix(taps: np.ndarray, *, samples_per_symbol: int = SPS,
                           code_len: int = PREAMBLE_CODE_LEN) -> np.ndarray:
    """M x 127 convolution design matrix for one shaped SYNC symbol.

    Independent re-implementation of
    ``gr-uwb/apps/sic_preamble_sim.py::recovery_design_matrix`` (used so the
    verifier does not import app code).
    """
    A = np.zeros((int(samples_per_symbol), int(code_len)), dtype=np.float64)
    for c in range(int(code_len)):
        off = c * CHIP_SAMPLE_STRIDE
        k = min(int(taps.size), int(samples_per_symbol) - off)
        if k > 0:
            A[off:off + k, c] = taps[:k]
    return A


def recover_chips(template: np.ndarray, taps: np.ndarray, *,
                  samples_per_symbol: int = SPS,
                  code_len: int = PREAMBLE_CODE_LEN) -> Tuple[np.ndarray, Dict]:
    """Recover the ternary HRP chips from one shaped SYNC symbol (public data).

    Pure-NumPy least squares; returns ``(chips, diag)`` with ``diag`` carrying
    the rank / residual / integer-deviation evidence so a caller can fail closed
    on an inexact recovery.
    """
    A = recovery_design_matrix(taps, samples_per_symbol=samples_per_symbol,
                              code_len=code_len)
    y = np.asarray(template[:samples_per_symbol], dtype=np.complex128)
    if y.size < samples_per_symbol:
        raise ValueError("template shorter than one SYNC symbol")
    y = y.real.astype(np.float64)
    coef, _res, rank, _sv = np.linalg.lstsq(A, y, rcond=None)
    resid = A @ coef - y
    resid_rel = float(np.linalg.norm(resid)) / max(
        1e-30, float(np.linalg.norm(y)))
    scale = float(np.max(np.abs(coef)))
    if scale <= 0.0:
        raise ValueError("degenerate SYNC symbol (zero peak)")
    norm = coef / scale
    chips = np.rint(norm).astype(np.int8)
    dev = float(np.max(np.abs(norm - chips)))
    diag = {
        "rank": int(rank), "expected_rank": int(code_len),
        "resid_rel": resid_rel, "max_int_dev": dev,
        "sync_symbol_scale": scale,
        "nonzero_chips": int(np.count_nonzero(chips)),
        "chip_energy": int(np.sum(chips.astype(np.int64) ** 2)),
        "ternary_values": sorted(int(v) for v in np.unique(chips)),
    }
    return chips, diag


# ---------------------------------------------------------------------------
# Fixed operator
# ---------------------------------------------------------------------------
@dataclass
class Operator:
    """The frozen linear CIR operator (identical for clean/raw/SIC)."""

    origin: int
    code: np.ndarray                    # complex128 [SPS]
    energy: float
    tap_lo: int = TAP_LO
    tap_hi: int = TAP_HI
    sync_reps: int = SYNC_REPS
    sps: int = SPS
    sample_rate: float = WORK_HZ
    phase_rotation: str = "identity"
    name: str = "code9_sampled_code_average"

    @property
    def ntaps(self) -> int:
        return int(self.tap_hi - self.tap_lo)

    def taps(self) -> np.ndarray:
        return np.arange(self.tap_lo, self.tap_hi, dtype=np.int64)

    def to_json(self) -> Dict:
        return {
            "name": self.name, "origin": int(self.origin),
            "sync_reps": int(self.sync_reps), "sps": int(self.sps),
            "sample_rate": float(self.sample_rate),
            "tap_lo": int(self.tap_lo), "tap_hi": int(self.tap_hi),
            "tap_count": int(self.ntaps),
            "code_energy": float(self.energy),
            "code_len": int(self.code.size),
            "phase_rotation": self.phase_rotation,
            "convention": "average",
            "sum_conversion": "H_sum = N * H_avg",
            "code_sha256": sha256_f32(self.code),
        }


def sha256_f32(a: np.ndarray) -> str:
    import hashlib
    return hashlib.sha256(
        np.ascontiguousarray(np.asarray(a), dtype=np.complex64)
        .tobytes()).hexdigest()


def fixed_operator_cir(iq: np.ndarray, op: Operator) -> np.ndarray:
    """Per-repetition complex CIR ``H[r, t]`` for one input window.

    Returns ``float64``-indexed ``H`` of shape ``(sync_reps, ntaps)`` as
    ``complex128``.  Samples outside the correlation range come back as
    ``NaN`` (never silently zero-padded).
    """
    y = np.asarray(iq, dtype=np.complex128)
    n = int(y.size)
    code = np.asarray(op.code, dtype=np.complex128)
    ncode = int(code.size)
    if n < ncode:
        raise ValueError("window %d shorter than code %d" % (n, ncode))
    from scipy.signal import fftconvolve
    # mf[k] = sum_m y[k+m] * conj(code[m]) for k in [0, n-ncode]
    mf = fftconvolve(y, np.conj(code[::-1]), mode="valid") / float(op.energy)
    H = np.full((int(op.sync_reps), op.ntaps), np.nan, dtype=np.complex128)
    idx = (int(op.origin) + np.arange(int(op.sync_reps))[:, None] * int(op.sps)
           + op.taps()[None, :])
    valid = (idx >= 0) & (idx <= n - ncode)
    H[valid] = mf[idx[valid]]
    return H


def rotate_by_cfo(H: np.ndarray, cfo_hz: float, *, sps: int = SPS,
                  sample_rate: float = WORK_HZ) -> np.ndarray:
    """Diagnostic-only common derotation of a per-repetition CIR tensor.

    Applied with the SAME scalar (the online SIC model's published ``cfo_hz``)
    to all three branches; it is never estimated per branch and never used for
    the default metrics.
    """
    H = np.asarray(H, dtype=np.complex128)
    r = np.arange(H.shape[0], dtype=np.float64)
    w = 2.0 * math.pi * float(cfo_hz) / float(sample_rate)
    rot = np.exp(-1j * w * (r * float(sps)))
    return H * rot[:, None]


# ---------------------------------------------------------------------------
# Accumulation and residual metrics
# ---------------------------------------------------------------------------
def rep_indices(full: bool, n: int) -> np.ndarray:
    """Repetition indices of a set: ``[0,N)`` or ``[INDEP_START, INDEP_START+N)``."""
    start = 0 if full else int(INDEP_START)
    return np.arange(start, start + int(n), dtype=np.int64)


def accumulate(H: np.ndarray, idx: Sequence[int]) -> np.ndarray:
    """``H_N = (1/N) sum_{r in idx} H[r]`` (average convention)."""
    H = np.asarray(H, dtype=np.complex128)
    sel = H[np.asarray(idx, dtype=np.int64)]
    return sel.mean(axis=0)


def energy(H: np.ndarray, lo: int = 0, hi: Optional[int] = None) -> float:
    """``sum_t |H[t]|^2`` over the absolute index range ``[lo, hi)``."""
    H = np.asarray(H)
    hi = H.shape[-1] if hi is None else int(hi)
    seg = H[..., int(lo):int(hi)]
    return float(np.sum(np.abs(seg) ** 2))


def _ratio_db(num: float, den: float) -> Tuple[Optional[float], str]:
    """``10log10(num/den)`` with exact zero handling (no eps floor)."""
    if num <= 0.0 and den <= 0.0:
        return None, "na"
    if den <= 0.0:
        return float("inf"), "pos_inf"
    if num <= 0.0:
        return float("-inf"), "neg_inf"
    return 10.0 * math.log10(num / den), "numeric"


def clip_db(v: Optional[float]) -> Optional[float]:
    """Plot-only clip; ``None`` stays ``None`` and the bounds are recorded."""
    if v is None:
        return None
    if v == float("inf"):
        return float(PLOT_CEIL_DB)
    if v == float("-inf"):
        return float(PLOT_FLOOR_DB)
    return float(min(max(v, PLOT_FLOOR_DB), PLOT_CEIL_DB))


def nmse(H_x: np.ndarray, H_clean: np.ndarray,
         lo: int = 0, hi: Optional[int] = None) -> Optional[float]:
    """``||H_x - H_clean||^2 / ||H_clean||^2`` over ``[lo, hi)`` (None if 0/0)."""
    d = energy(np.asarray(H_x) - np.asarray(H_clean), lo, hi)
    c = energy(H_clean, lo, hi)
    if c <= 0.0:
        return None
    return float(d / c)


def residual_table(H_clean: np.ndarray, H_raw: np.ndarray, H_sic: np.ndarray,
                   op: Operator) -> Dict:
    """Per-N accumulation metrics on both repetition sets.

    Returns a JSON-ready dict::

        {"full":   {"1": {...}, "2": {...}, ...},
         "indep":  {...},
         "positions": {"tap_grid": [lo,hi], "roi_target": [...], ...}}
    """
    grid_lo, grid_hi = 0, op.ntaps
    rois = {
        "full": (grid_lo, grid_hi),
        "target": (int(TARGET_LO - op.tap_lo), int(TARGET_HI - op.tap_lo)),
        "background": (int(BG_LO - op.tap_lo), int(BG_HI - op.tap_lo)),
        "direct": (int(DIRECT_LO - op.tap_lo), int(DIRECT_HI - op.tap_lo)),
    }
    out: Dict = {"full": {}, "indep": {}, "rois": rois,
                 "tap_domain": {"tap_lo": int(op.tap_lo),
                                "tap_hi": int(op.tap_hi),
                                "origin": int(op.origin),
                                "roi_target_offsets": [int(TARGET_LO),
                                                       int(TARGET_HI)],
                                "roi_target_tap": int(TARGET_OFFSET),
                                "roi_background_offsets": [int(BG_LO),
                                                           int(BG_HI)],
                                "receiver_tap_offset": RX_TAP_OFFSET}}
    for set_name, n_list in (("full", FULL_N), ("indep", INDEP_N)):
        for n in n_list:
            idx = rep_indices(set_name == "full", n)
            hc = accumulate(H_clean, idx)
            hr = accumulate(H_raw, idx)
            hs = accumulate(H_sic, idx)
            rec: Dict = {"N": int(n), "start": int(idx[0]), "stop": int(idx[-1]) + 1}
            for rname, (lo, hi) in rois.items():
                e_raw = energy(hr - hc, lo, hi)
                e_sic = energy(hs - hc, lo, hi)
                c, flag = _ratio_db(e_raw, e_sic)
                rec.setdefault("e_raw", {})[rname] = e_raw
                rec.setdefault("e_sic", {})[rname] = e_sic
                rec.setdefault("c_cir_db", {})[rname] = c
                rec.setdefault("c_cir_flag", {})[rname] = flag
                rec.setdefault("c_cir_db_clipped", {})[rname] = clip_db(c)
                rec.setdefault("nmse_raw", {})[rname] = nmse(hr, hc, lo, hi)
                rec.setdefault("nmse_sic", {})[rname] = nmse(hs, hc, lo, hi)
                e_clean = energy(hc, lo, hi)
                rr, rf = _ratio_db(e_raw, e_clean)
                sr, sf = _ratio_db(e_sic, e_clean)
                rec.setdefault("residual_raw_db", {})[rname] = rr
                rec.setdefault("residual_raw_flag", {})[rname] = rf
                rec.setdefault("residual_sic_db", {})[rname] = sr
                rec.setdefault("residual_sic_flag", {})[rname] = sf
            # Accumulation gain vs the same set's N=1 (nested, same start) is
            # filled in the second pass below (needs the N=1 entry).
            out[set_name][str(n)] = rec
        # second pass for the accumulation gain (needs the N=1 entry)
        e1r = out[set_name]["1"]["e_raw"]["full"]
        e1s = out[set_name]["1"]["e_sic"]["full"]
        for n in n_list:
            rec = out[set_name][str(n)]
            gr, fr = _ratio_db(e1r, rec["e_raw"]["full"])
            gs, fs = _ratio_db(e1s, rec["e_sic"]["full"])
            rec["g_raw_db"] = gr
            rec["g_raw_flag"] = fr
            rec["g_sic_db"] = gs
            rec["g_sic_flag"] = fs
            # accumulation gain is NOT an SIC gain; recorded explicitly.
            rec["g_is_sic_gain"] = False
            rec["sum_conversion"] = int(n)
    return out


# ---------------------------------------------------------------------------
# Coherence of the per-symbol residual
# ---------------------------------------------------------------------------
def coherence_stats(E: np.ndarray, op: Operator,
                    roi="target") -> Dict:
    """Single-window coherence of a per-symbol residual tensor ``E[r, tap]``.

    * ``eta = ||sum_r E_r||^2 / (N sum_r ||E_r||^2)`` (1 for perfectly coherent,
      ~1/N for an i.i.d. zero-mean ensemble, N/A on zero energy),
    * uncentered and centered lag correlations (a finite-sample de-mean removes
      the coherent part, so both are reported),
    * mean vs random energy split, and the dominant-tap phase trajectory /
      linear phase slope (the CFO-ramp indicator).
    """
    E = np.asarray(E, dtype=np.complex128)
    if isinstance(roi, str):
        roi = {"target": (TARGET_LO, TARGET_HI),
               "background": (BG_LO, BG_HI),
               "all": (op.tap_lo, op.tap_hi)}[roi]
    lo, hi = int(roi[0] - op.tap_lo), int(roi[1] - op.tap_lo)
    seg = E[:, lo:hi]
    N = int(seg.shape[0])
    total = float(np.sum(np.abs(seg) ** 2))
    if total <= 0.0:
        return {"N": N, "roi": list(roi), "energy": 0.0, "eta": None,
                "eta_flag": "na", "lag": {}, "centered_lag": {},
                "mean_energy": 0.0, "random_energy": 0.0,
                "mean_fraction": None, "phase_slope_rad_per_symbol": None,
                "phase_slope_hz": None, "dominant_tap": None}
    s = seg.sum(axis=0)
    eta = float(np.sum(np.abs(s) ** 2) / (N * total))
    mean = seg.mean(axis=0)
    mean_e = float(np.sum(np.abs(mean) ** 2))
    rand = seg - mean
    rand_e = float(np.sum(np.abs(rand) ** 2))
    lags: Dict[str, Optional[float]] = {}
    clags: Dict[str, Optional[float]] = {}
    for lag in range(1, min(N - 1, 8) + 1):
        a = seg[lag:]
        b = seg[:-lag]
        num = np.vdot(a, b)
        den = math.sqrt(float(np.sum(np.abs(a) ** 2))
                        * float(np.sum(np.abs(b) ** 2)))
        lags[str(lag)] = None if den <= 0 else float(abs(num) / den)
        ac = a - a.mean(axis=0)
        bc = b - b.mean(axis=0)
        numc = np.vdot(ac, bc)
        denc = math.sqrt(float(np.sum(np.abs(ac) ** 2))
                         * float(np.sum(np.abs(bc) ** 2)))
        clags[str(lag)] = None if denc <= 0 else float(abs(numc) / denc)
    # Dominant-tap phase trajectory.
    mag = np.abs(seg).mean(axis=0)
    dt = int(np.argmax(mag))
    phase = np.unwrap(np.angle(seg[:, dt]))
    slope = float(np.polyfit(np.arange(N, dtype=np.float64), phase, 1)[0]) \
        if N >= 2 else 0.0
    slope_hz = slope / (2.0 * math.pi * (SPS / WORK_HZ))
    return {
        "N": N, "roi": list(roi), "energy": total, "eta": eta,
        "eta_flag": "numeric", "eta_random_reference": 1.0 / N,
        "lag": lags, "centered_lag": clags,
        "mean_energy": mean_e, "random_energy": rand_e,
        "mean_fraction": mean_e / total if total > 0 else None,
        "phase_slope_rad_per_symbol": slope,
        "phase_slope_hz": slope_hz,
        "dominant_tap": int(dt + lo + op.tap_lo),
    }


# ---------------------------------------------------------------------------
# Target protection (weak path delay 37, gain 0.25)
# ---------------------------------------------------------------------------
def target_metrics(H_N: np.ndarray, H_clean_N: np.ndarray,
                   op: Operator) -> Dict:
    """Weak-target amplitude / phase error, local peak shift, background proxy."""
    H = np.asarray(H_N, dtype=np.complex128)
    C = np.asarray(H_clean_N, dtype=np.complex128)
    tlo, thi = TARGET_LO - op.tap_lo, TARGET_HI - op.tap_lo
    blo, bhi = BG_LO - op.tap_lo, BG_HI - op.tap_lo
    tref = int(TARGET_OFFSET - op.tap_lo)
    a_h = abs(H[tref])
    a_c = abs(C[tref])
    amp_err_db = (20.0 * math.log10(a_h / a_c)
                  if a_h > 0.0 and a_c > 0.0 else None)
    ph = float(np.angle(H[tref] * np.conj(C[tref]))) if (a_h > 0 and a_c > 0) \
        else None
    peak_h = int(np.argmax(np.abs(H[tlo:thi]))) + tlo
    peak_c = int(np.argmax(np.abs(C[tlo:thi]))) + tlo
    bg = float(np.max(np.abs(H[blo:bhi]))) if bhi > blo else 0.0
    bg_mean = float(np.mean(np.abs(H[blo:bhi]) ** 2)) if bhi > blo else 0.0
    return {
        "target_tap": int(TARGET_OFFSET),
        "amp_h": a_h, "amp_clean": a_c,
        "amp_err_db": amp_err_db,
        "phase_err_deg": None if ph is None else math.degrees(ph),
        "peak_tap": peak_h, "peak_tap_clean": peak_c,
        "peak_shift": peak_h - peak_c,
        "peak_in_roi_taps": [int(tlo + op.tap_lo), int(thi + op.tap_lo)],
        "bg_false_peak_ratio": (bg / a_h) if a_h > 0 else None,
        "target_to_background_proxy": ((a_h ** 2) / bg_mean) if bg_mean > 0
        else None,
        "target_to_background_definition":
            "|H[target_tap]|^2 / mean_{bg ROI}|H[tap]|^2 (weak-target proxy; "
            "NOT a detection SINR)",
    }


# ---------------------------------------------------------------------------
# IQ-level actual cancellation (kept alongside the CIR metrics)
# ---------------------------------------------------------------------------
def iq_suppression_db(raw: np.ndarray, clean: np.ndarray, sic: np.ndarray,
                      lo: int, hi: int) -> Dict:
    """Actual-output suppression on raw IQ: ``||raw-clean||^2 / ||sic-clean||^2``.

    This is the S0 ``actual_suppression_db`` quantity, computed on the real
    output, not on the reconstruction.
    """
    r = np.asarray(raw, dtype=np.complex128)[lo:hi]
    c = np.asarray(clean, dtype=np.complex128)[lo:hi]
    s = np.asarray(sic, dtype=np.complex128)[lo:hi]
    num = float(np.sum(np.abs(r - c) ** 2))
    den = float(np.sum(np.abs(s - c) ** 2))
    val, flag = _ratio_db(num, den)
    return {"lo": int(lo), "hi": int(hi), "raw_residual_energy": num,
            "sic_residual_energy": den, "suppression_db": val, "flag": flag}


# ---------------------------------------------------------------------------
# Aggregation helpers
# ---------------------------------------------------------------------------
def percentile(values: Iterable[float], p: float) -> Optional[float]:
    vals = np.asarray([v for v in values if v is not None and np.isfinite(v)],
                      dtype=np.float64)
    if vals.size == 0:
        return None
    return float(np.percentile(vals, p))


def bootstrap_ci(values: Sequence[float], *, n_boot: int = 2000,
                 alpha: float = 0.05, seed: int = 20261017,
                 grouped: Optional[Sequence] = None) -> Dict:
    """95% bootstrap interval of the mean.

    ``grouped`` (optional) gives one group id per value; resampling is done by
    GROUP (window/seed), so the same window's different-N entries are never
    treated as independent samples.
    """
    vals = np.asarray(values, dtype=np.float64)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return {"n": 0, "mean": None, "lo": None, "hi": None}
    rng = np.random.default_rng(int(seed))
    if grouped is None:
        idx = rng.integers(0, vals.size, size=(int(n_boot), vals.size))
        means = vals[idx].mean(axis=1)
    else:
        groups = np.asarray(grouped)
        groups = groups[np.isfinite(np.asarray(values, dtype=np.float64))]
        uniq = np.unique(groups)
        by_group = {g: vals[groups == g] for g in uniq}
        means = np.empty(int(n_boot), dtype=np.float64)
        for i in range(int(n_boot)):
            pick = rng.integers(0, uniq.size, size=uniq.size)
            means[i] = np.concatenate([by_group[uniq[j]] for j in pick]).mean()
    return {"n": int(vals.size), "mean": float(vals.mean()),
            "lo": float(np.percentile(means, 100 * alpha / 2)),
            "hi": float(np.percentile(means, 100 * (1 - alpha / 2))),
            "n_boot": int(n_boot), "grouped": grouped is not None}


def to_csv_rows(records: Sequence[Dict], columns: Sequence[str]) -> str:
    import csv
    import io
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(columns)
    for r in records:
        w.writerow([_csv_cell(r.get(c)) for c in columns])
    return buf.getvalue()


def _csv_cell(v):
    if v is None:
        return "na"
    if isinstance(v, float):
        if v == float("inf"):
            return "inf"
        if v == float("-inf"):
            return "-inf"
        return repr(v)
    return v


#: Public alias (the app/verifier formatting helper).
csv_cell = _csv_cell


# ---------------------------------------------------------------------------
# QA self-test (S2-A instruction section 5, cases 1-6)
# ---------------------------------------------------------------------------
def _qa_operator(**kw) -> Operator:
    chips = np.zeros(PREAMBLE_CODE_LEN, dtype=np.int8)
    chips[::2] = 1
    chips[1::4] = -1
    code = sampled_code_from_chips(chips)
    return Operator(origin=0, code=code, energy=code_energy(code), **kw)


def _synthetic_H(N=64, ntaps=64, seed=1, kind="independent"):
    """Synthetic per-symbol residual tensor with a known structure."""
    rng = np.random.default_rng(seed)
    if kind == "independent":
        return (rng.standard_normal((N, ntaps))
                + 1j * rng.standard_normal((N, ntaps))) * 0.1
    if kind == "identical":
        one = (rng.standard_normal(ntaps) + 1j * rng.standard_normal(ntaps))
        return np.tile(one * 0.1, (N, 1))
    if kind == "ramp":
        r = np.arange(N, dtype=np.float64)[:, None]
        amp = (rng.standard_normal(ntaps) + 1j * rng.standard_normal(ntaps))
        return 0.1 * np.exp(1j * 0.3 * r) * np.tile(amp, (N, 1))
    raise ValueError(kind)


def self_test(verbose: bool = True) -> int:
    """Run the six S2-A fixed-operator QA cases; return the failure count."""
    fails: List[str] = []

    def check(name, cond, detail=""):
        okk = bool(cond)
        if verbose:
            print("%-58s %s %s" % (name, "PASS" if okk else "FAIL", detail))
        if not okk:
            fails.append(name)

    op = _qa_operator()

    # --- 1. perfect subtraction: out == clean -> zero residual, no 0/0 -------
    hc = _synthetic_H(SYNC_REPS, op.ntaps, seed=2, kind="independent") + 1.0
    rt = residual_table(hc, hc, hc, op)
    e = rt["full"]["8"]["e_sic"]["full"]
    check("qa1 perfect subtraction: zero residual", e == 0.0, "e=%.3g" % e)
    check("qa1 C_CIR 0/0 reported N/A (not fabricated)",
          rt["full"]["8"]["c_cir_flag"]["full"] == "na",
          rt["full"]["8"]["c_cir_flag"]["full"])
    check("qa1 NMSE of a perfect subtraction is exactly 0 (no 0/0 artifacts)",
          rt["full"]["8"]["nmse_sic"]["full"] == 0.0
          and rt["full"]["8"]["residual_sic_flag"]["full"] == "neg_inf",
          "%r" % rt["full"]["8"]["nmse_sic"]["full"])
    coh = coherence_stats(np.zeros((SYNC_REPS, op.ntaps),
                                   dtype=np.complex128), op)
    check("qa1 coherence N/A on zero energy", coh["eta"] is None
          and coh["eta_flag"] == "na")

    # --- 2. no SIC: raw == sic -> C_CIR = 0 dB ------------------------------
    hclean = _synthetic_H(SYNC_REPS, op.ntaps, seed=3) + 2.0
    hraw = hclean + _synthetic_H(SYNC_REPS, op.ntaps, seed=4)
    rt = residual_table(hclean, hraw, hraw, op)
    rec = rt["full"]["8"]
    check("qa2 raw==sic -> C_CIR = 0 dB",
          rec["c_cir_flag"]["full"] == "numeric"
          and abs(rec["c_cir_db"]["full"]) < 1e-9,
          "%.3g" % rec["c_cir_db"]["full"])
    check("qa2 raw==sic -> G_raw == G_sic",
          rec["g_raw_db"] == rec["g_sic_db"])

    # --- 3. independent zero-mean residual: mean power decays ~1/N ----------
    amps = {}
    for N in (1, 4, 16, 64):
        accs = []
        for s in range(80):
            E = _synthetic_H(N, op.ntaps, seed=1000 + 7 * s + N)
            accs.append(float(np.sum(np.abs(E.mean(axis=0)) ** 2)))
        amps[N] = float(np.mean(accs))
    ratios = [amps[1] / amps[4], amps[4] / amps[16], amps[16] / amps[64]]
    # theoretical ratio == 4; statistical tolerance, not an exact-3dB assert.
    check("qa3 independent residual decays ~1/N (ratio 4 +-35%)",
          all(4.0 * 0.65 <= r <= 4.0 * 1.35 for r in ratios),
          ",".join("%.2f" % r for r in ratios))

    # --- 4. identical per-symbol residual: average power does NOT decay -----
    amps = {}
    for N in (1, 4, 16, 64):
        E = _synthetic_H(N, op.ntaps, seed=5, kind="identical")
        amps[N] = float(np.sum(np.abs(E.mean(axis=0)) ** 2))
    rng_ratio = max(amps.values()) / min(amps.values())
    check("qa4 identical residual: no 1/N decay",
          abs(rng_ratio - 1.0) < 1e-9, "ratio=%.12g" % rng_ratio)

    # --- 5. fixed phase ramp: coherent, but non-monotonic accumulation ------
    E = _synthetic_H(64, op.ntaps, seed=6, kind="ramp")
    per_sym = np.sum(np.abs(E) ** 2, axis=1)
    ev = np.array([float(np.sum(np.abs(E[:4 * (k + 1)].mean(axis=0)) ** 2))
                   for k in range(16)])
    coh = coherence_stats(E, op, roi="all")
    coh_lag1 = coh["centered_lag"].get("1")
    check("qa5 phase ramp: constant per-symbol energy (not decaying)",
          per_sym.max() / per_sym.min() < 1.001,
          "ratio=%.6f" % (per_sym.max() / per_sym.min()))
    check("qa5 phase ramp: centered lag-1 correlation stays ~1 (NOT random)",
          coh_lag1 is not None and coh_lag1 > 0.99, "rho1=%s" % (coh_lag1,))
    check("qa5 phase ramp: low eta alone must not be read as random "
          "(eta=%.4f < 1/N=%.4f)" % (coh["eta"], 1.0 / 64),
          coh["eta"] is not None and coh["eta"] < 5.0 / 64)
    check("qa5 phase ramp: dominant-tap slope recovers 0.3 rad/symbol",
          abs(coh["phase_slope_rad_per_symbol"] - 0.3) < 0.05,
          "%.4f" % coh["phase_slope_rad_per_symbol"])
    check("qa5 scalar-coherence is NOT proof of independence",
          len(set(np.round(ev, 12))) > 4, "accumulation not monotone")

    # --- 6. additivity / sum-vs-average / region boundary -------------------
    d1 = _synthetic_H(SYNC_REPS, op.ntaps, seed=7)
    d2 = _synthetic_H(SYNC_REPS, op.ntaps, seed=8)
    s1 = accumulate(d1 + d2, rep_indices(True, 8))
    s2 = accumulate(d1, rep_indices(True, 8)) \
        + accumulate(d2, rep_indices(True, 8))
    check("qa6 accumulation is additive (fixed linear operator)",
          np.allclose(s1, s2, atol=1e-12))
    hsum = np.sum(d1[rep_indices(True, 4)], axis=0)
    check("qa6 sum vs average conversion H_sum = N*H_avg",
          np.allclose(hsum, 4 * accumulate(d1, rep_indices(True, 4)),
                      atol=1e-12))
    check("qa6 full and independent set boundaries",
          list(rep_indices(True, 8)) == list(range(8))
          and list(rep_indices(False, 8)) == list(range(8, 16)))
    # paired same-operator check with a known residual scale ratio (x2 -> 6.02 dB)
    hclean = np.ones((SYNC_REPS, op.ntaps), dtype=np.complex128) * 3.0
    dd = _synthetic_H(SYNC_REPS, op.ntaps, seed=9)
    rt = residual_table(hclean, hclean + dd, hclean + 0.5 * dd, op)
    rec = rt["full"]["8"]
    check("qa6 known scale ratio -> C_CIR = 6.0206 dB",
          abs(rec["c_cir_db"]["full"] - 20.0 * math.log10(2.0)) < 1e-9,
          "%.6f" % rec["c_cir_db"]["full"])
    check("qa6 residual vs clean NMSE matches by hand",
          abs(rec["nmse_raw"]["full"]
              - energy(dd[:8].mean(axis=0)) / energy(hclean[:8].mean(axis=0)))
          < 1e-12,
          "%.6g" % rec["nmse_raw"]["full"])

    if verbose:
        print("self-test: %d case(s) failed" % len(fails))
    return len(fails)


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true",
                    help="run the six fixed-operator QA cases")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(verbose=not args.quiet)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
