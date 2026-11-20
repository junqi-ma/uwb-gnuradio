#!/usr/bin/env python3
"""S2-A fixed-operator accumulation metrics (NumPy reference + QA), schema 2.

Scope
-----
Offline fixed-operator evaluation layer of S2-A
(``docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`` section 3.1) plus the
rectification items A-R01 (support-disjoint independent region) and A-R02
(coherent/centered energy decomposition) of
``docs/sic/S2A_整改意见_2026-10-18.md``:

    H_x[r, t] = sum_m  y_x[origin + r*SPS + t + m] * conj(code9[m]) / E_code

for the three paired branches ``x in {clean, raw, sic}`` on the 998.4 MS/s work
grid, and the accumulation / residual / coherence / target metrics derived from
it.  It is deliberately:

* **linear and input-only**: one fixed template, one fixed time origin, one
  fixed tap grid, one fixed gain normalisation and NO phase rotation, so that
  ``H_x`` is a linear functional of the input IQ.
* **truth-free**: the origin is the public nominal sensing origin (``pre``),
  the template is the public code-9 operator; the injected comm start / CFO /
  paths / interferer IQ never enter.
* **support-aware**: the independent-region repetition set is derived from the
  ACTUAL read support of the correlation operator (nonzero template indices),
  the public visibility limit and the actual applied subtraction region -- not
  from a repetition index (A-R01).

It contains no GNU Radio or pybind dependency, so the independent verifier can
import it without a build tree.  The verifier re-implements the algebra it needs
rather than trusting these helpers.

Conventions (frozen in ``testdata/sic_accum/operator.json``, schema 2)
---------------------------------------------------------------------
* ``average``: ``H_x,N = (1/N) sum_{r in R_N} H_x[r]``; ``H_sum = N * H_avg``.
* residual: ``E_raw,N = H_raw,N - H_clean,N``, ``E_sic,N = H_sic,N - H_clean,N``.
* CIR SIC gain: ``C_CIR(N) = 10log10(||E_raw,N||^2 / ||E_sic,N||^2)``.
* accumulation gain (NOT an SIC gain): ``G(N) = 10log10(P(1)/P(N))`` on nested
  same-start sets.
* coherence on one window (A-R02, same-dimension energies)::

      mu             = (1/N) sum_r E_r
      E_coherent     = N * ||mu||^2
      E_centered     = sum_r ||E_r - mu||^2
      E_total        = E_coherent + E_centered
      coherent_fraction = E_coherent / E_total = eta

  ``E_centered`` is **not** "random noise": a deterministic phase ramp also lands
  there.
* zero numerator / denominator are reported as ``+Inf`` / ``-Inf`` / ``N/A``
  (never an ``eps`` floor, never clipped to a "good" value).  ``*_clipped``
  mirrors exist for plotting only.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Frozen geometry / domain constants (mirror apps/sic_preamble_sim.py)
# ---------------------------------------------------------------------------
SCHEMA_VERSION = 2

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

#: Receiver-layer (``uwb.radar_cir_estimator``) tap-domain ROIs (R04 values).
RX_TARGET_ROI = (51, 60)
RX_TARGET_TAP = 55
RX_BACKGROUND_TAPS = (80, 116)     # [64,100) offsets + cir_pre 16
RX_TAP_OFFSET = 16

#: Repetition-set nominal N lists (truncated honestly when fewer reps are legal).
FULL_N = (1, 2, 4, 8, 16, 32, 64)
INDEP_N_NOMINAL = (1, 2, 4, 8, 16, 32, 56)
#: A-R01: the old ``[8,8+N)`` label is kept only as a legacy diagnostic.
LEGACY_INDEP_START = 8

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
    ``apps/sic_preamble_sim.py::recovery_design_matrix`` (used so the
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
# A-R01: exact read support of the correlation operator
# ---------------------------------------------------------------------------
def code_support(code: np.ndarray) -> Dict:
    """Nonzero template support of the correlation operator.

    The matched filter multiplies by ``conj(code[m])``; zero taps contribute
    exactly zero, so the samples an output actually depends on are
    ``origin + r*sps + t + m`` for the NONZERO template indices ``m``.  Both the
    exact support and the conservative full-window support are reported.
    """
    c = np.asarray(code)
    idx = np.flatnonzero(np.abs(c) > 0.0)
    if idx.size == 0:
        raise ValueError("degenerate (all-zero) template")
    return {"nz_count": int(idx.size),
            "nz_min": int(idx[0]),
            "nz_max": int(idx[-1]),
            "nz_indices": idx,
            "conservative_len": int(c.size)}


def read_support(op: "Operator", r: int, tap_lo: int, tap_hi: int,
                 nz: Optional[Dict] = None) -> Tuple[int, int]:
    """Half-open absolute sample span read for repetition ``r`` over a tap ROI.

    Exact (nonzero template indices) by default; ``nz=None`` uses the
    conservative full template window.
    """
    if nz is None:
        nz = code_support(op.code)
    base = int(op.origin) + int(r) * int(op.sps)
    lo = base + int(tap_lo) + int(nz["nz_min"])
    hi = base + int(tap_hi) - 1 + int(nz["nz_max"]) + 1
    return int(lo), int(hi)


def truncate_n_list(available: int,
                    nominal: Sequence[int] = INDEP_N_NOMINAL) -> Tuple[int, ...]:
    """Honest N list: a prefix of ``nominal`` plus the real tail if it is short.

    A-R01 §5: "数量不足56就如实缩短，不补零、不借用SFD、不悄悄换N标签".  The
    appended tail IS the actual available count (e.g. 54), not a relabelled 56.
    """
    available = int(available)
    out = [int(n) for n in nominal if int(n) <= available]
    if available >= 32 and available not in out:
        out.append(available)
    return tuple(out)


def derive_rep_set(op: "Operator", *, visible_end: int, sub_lo: int,
                   sub_hi: int, n: int, tap_lo: int, tap_hi: int,
                   name: str, kind: str,
                   nominal: Sequence[int] = INDEP_N_NOMINAL,
                   strict_holdout: bool = True) -> Dict:
    """Derive a repetition set from the operator's ACTUAL sample support.

    A repetition ``r`` is legal iff every sample the correlation reads for that
    repetition over ``[tap_lo, tap_hi)`` is:

    * at or after ``visible_end`` (the public estimation visibility limit, which
      already includes the extra visibility guard) -- strict holdout never reads
      training/guard samples;
    * inside the ACTUAL applied subtraction region ``[sub_lo, sub_hi)`` -- the
      independent region must not score un-subtracted tail samples;
    * inside the window ``[0, n)``.

    Returns a JSON-ready dict (half-open intervals throughout) with the legal
    repetitions, the honest N list, the excluded counts/reasons and the bounds.
    """
    nz = code_support(op.code)
    reps: List[int] = []
    excluded: List[Dict] = []
    for r in range(int(op.sync_reps)):
        lo, hi = read_support(op, r, tap_lo, tap_hi, nz)
        reasons = []
        if lo < 0 or hi > int(n):
            reasons.append("outside_window")
        if strict_holdout and lo < int(visible_end):
            reasons.append("reads_training_or_guard")
        if lo < int(sub_lo) or hi > int(sub_hi):
            reasons.append("outside_applied_subtraction")
        if reasons:
            excluded.append({"r": int(r), "read_lo": lo, "read_hi": hi,
                             "reasons": reasons})
        else:
            reps.append(int(r))
    # keep the longest contiguous run (the operator is period-sampled, so a
    # non-contiguous legal set is a geometry/anomaly signal, recorded as such).
    runs: List[List[int]] = []
    for r in reps:
        if runs and r == runs[-1][-1] + 1:
            runs[-1].append(r)
        else:
            runs.append([r])
    run = max(runs, key=len) if runs else []
    dropped_noncontig = sorted(set(reps) - set(run))
    first = int(run[0]) if run else -1
    last = int(run[-1]) if run else -1
    lo_read = int(read_support(op, first, tap_lo, tap_hi, nz)[0]) if run else -1
    hi_read = int(read_support(op, last, tap_lo, tap_hi, nz)[1]) if run else -1
    return {
        "name": name, "kind": kind,
        "strict_holdout": bool(strict_holdout),
        "tap_lo": int(tap_lo), "tap_hi": int(tap_hi),
        "available": len(run),
        "first": first, "last": last,
        "reps": [int(r) for r in run],
        "n_list": list(truncate_n_list(len(run), nominal)),
        "n_nominal": [int(x) for x in nominal],
        "n_shortened": bool(len(run) < max(int(x) for x in nominal)),
        "read_lo": lo_read, "read_hi": hi_read,
        "support_source": "nonzero_template_indices",
        "nz_support": {"nz_min": int(nz["nz_min"]), "nz_max": int(nz["nz_max"]),
                       "nz_count": int(nz["nz_count"]),
                       "conservative_len": int(nz["conservative_len"])},
        "bounds": {"visible_end": int(visible_end), "sub_lo": int(sub_lo),
                   "sub_hi": int(sub_hi), "n": int(n),
                   "origin": int(op.origin), "sps": int(op.sps)},
        "excluded_before": sum(1 for e in excluded if e["r"] < first
                               and "outside_applied_subtraction" not in
                               e["reasons"]) if run else len(excluded),
        "excluded_after": sum(1 for e in excluded if run and e["r"] > last)
        if run else 0,
        "excluded": excluded,
        "dropped_noncontiguous": dropped_noncontig,
    }


def legacy_nominal_index_set(op: "Operator",
                             nominal: Sequence[int] = INDEP_N_NOMINAL) -> Dict:
    """The OLD ``[8,8+N)`` definition, kept as an explicit legacy diagnostic.

    A-R01 §4: it is NOT a support-disjoint set (its correlation windows still
    read training/guard samples and, at the far end, un-subtracted tail
    samples).  It is reported separately and never used as the main result.
    """
    reps = list(range(int(LEGACY_INDEP_START),
                      int(LEGACY_INDEP_START) + int(max(nominal))))
    return {
        "name": "legacy_nominal_index", "kind": "legacy_nominal_index",
        "strict_holdout": False, "tap_lo": int(TAP_LO), "tap_hi": int(TAP_HI),
        "available": len(reps), "first": int(reps[0]), "last": int(reps[-1]),
        "reps": reps, "n_list": [int(x) for x in nominal],
        "n_nominal": [int(x) for x in nominal], "n_shortened": False,
        "read_lo": int(op.origin) + reps[0] * int(op.sps) + int(TAP_LO),
        "read_hi": int(op.origin) + reps[-1] * int(op.sps) + int(TAP_HI) - 1
        + 1008 + 1,
        "support_source": "repetition_index_only",
        "nz_support": None, "bounds": None, "excluded_before": 0,
        "excluded_after": 0, "excluded": [], "dropped_noncontiguous": [],
        "superseded_by": "support_disjoint",
        "note": "reads training/guard samples and un-subtracted tail samples; "
                "diagnostic only",
    }


def full_preamble_set(op: "Operator") -> Dict:
    """``[0,64)`` full-preamble set (kept; head/tail finite-length effects)."""
    reps = list(range(int(op.sync_reps)))
    return {
        "name": "full_preamble", "kind": "full_preamble",
        "strict_holdout": False, "tap_lo": int(TAP_LO), "tap_hi": int(TAP_HI),
        "available": len(reps), "first": 0, "last": int(reps[-1]),
        "reps": reps, "n_list": [int(x) for x in FULL_N],
        "n_nominal": [int(x) for x in FULL_N], "n_shortened": False,
        "read_lo": int(op.origin) + int(TAP_LO),
        "read_hi": int(op.origin) + (len(reps) - 1) * int(op.sps)
        + int(TAP_HI) - 1 + 1008 + 1,
        "support_source": "full_preamble", "nz_support": None, "bounds": None,
        "excluded_before": 0, "excluded_after": 0, "excluded": [],
        "dropped_noncontiguous": [],
        "note": "includes the first/last repetition finite-length effects and "
                "reads training/guard samples; not a holdout",
    }


def coverage_report(op: "Operator", rs: Dict, *, truth_span: Optional[Tuple],
                    n: int) -> Dict:
    """Coverage ratios for CIR support vs applied subtraction vs truth support.

    A-R01 §6: correlation support, interference truth support and the actual
    subtraction support are separate quantities; they are intersected and
    reported as ratios instead of scoring only inside the subtracted region.
    ``truth_span`` is an OFFLINE diagnostic input and never feeds a decision.
    """
    if not rs.get("reps"):
        return {"cir_support": None}
    lo, hi = int(rs["read_lo"]), int(rs["read_hi"])
    span = max(1, hi - lo)
    out = {"cir_support": [lo, hi], "cir_support_len": span,
           "window": [0, int(n)]}
    b = rs.get("bounds") or {}
    if b:
        slo, shi = int(b["sub_lo"]), int(b["sub_hi"])
        inter = max(0, min(hi, shi) - max(lo, slo))
        out["applied_sub_support"] = [slo, shi]
        out["cir_overlap_sub_ratio"] = inter / span
        out["cir_outside_sub_samples"] = span - inter
    if truth_span is not None:
        tlo, thi = int(truth_span[0]), int(truth_span[1])
        inter = max(0, min(hi, thi) - max(lo, tlo))
        out["truth_support_diagnostic"] = [tlo, thi]
        out["cir_overlap_truth_ratio"] = inter / span
        out["truth_overlap_cir_ratio"] = inter / max(1, thi - tlo)
    return out


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
            "support_source": "nonzero_template_indices",
        }


def sha256_f32(a: np.ndarray) -> str:
    import hashlib
    return hashlib.sha256(
        np.ascontiguousarray(np.asarray(a), dtype=np.complex64)
        .tobytes()).hexdigest()


def matched_filter(iq: np.ndarray, code: np.ndarray,
                   energy: float) -> np.ndarray:
    """Exact sparse matched filter ``mf[k] = sum_m y[k+m] conj(code[m]) / E``.

    Uses the SPARSE nonzero template indices so the read support is exactly the
    declared one (A-R01) and the result is bit-exact under changes outside that
    support.  Falls back to a dense FFT convolution only for a dense template.
    """
    y = np.asarray(iq, dtype=np.complex128)
    c = np.asarray(code, dtype=np.complex128)
    out_len = int(y.size) - int(c.size) + 1
    if out_len <= 0:
        raise ValueError("window %d shorter than code %d" % (y.size, c.size))
    nz = np.flatnonzero(np.abs(c) > 0.0)
    if nz.size * 8 < int(c.size):     # sparse: exact, reads only nz indices
        acc = np.zeros(out_len, dtype=np.complex128)
        for m in nz:
            acc += np.conj(c[m]) * y[m:m + out_len]
        return acc / float(energy)
    from scipy.signal import fftconvolve
    return fftconvolve(y, np.conj(c[::-1]), mode="valid") / float(energy)


def fixed_operator_cir(iq: np.ndarray, op: Operator) -> np.ndarray:
    """Per-repetition complex CIR ``H[r, t]`` for one input window.

    Returns ``complex128`` of shape ``(sync_reps, ntaps)``.  Samples outside the
    correlation range come back as ``NaN`` (never silently zero-padded).
    """
    y = np.asarray(iq, dtype=np.complex128)
    n = int(y.size)
    code = np.asarray(op.code, dtype=np.complex128)
    mf = matched_filter(y, code, op.energy)
    H = np.full((int(op.sync_reps), op.ntaps), np.nan, dtype=np.complex128)
    idx = (int(op.origin) + np.arange(int(op.sync_reps))[:, None] * int(op.sps)
           + op.taps()[None, :])
    valid = (idx >= 0) & (idx <= mf.size - 1)
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
    """Legacy helper: ``[0,N)`` (full) or ``[8,8+N)`` (old independent label).

    Kept for the golden/replay helpers and the legacy diagnostic; the
    support-disjoint path uses ``derive_rep_set`` instead.
    """
    start = 0 if full else int(LEGACY_INDEP_START)
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


def _roi_map(op: "Operator") -> Dict[str, Tuple[int, int]]:
    return {
        "full": (0, int(op.ntaps)),
        "target": (int(TARGET_LO - op.tap_lo), int(TARGET_HI - op.tap_lo)),
        "background": (int(BG_LO - op.tap_lo), int(BG_HI - op.tap_lo)),
        "direct": (int(DIRECT_LO - op.tap_lo), int(DIRECT_HI - op.tap_lo)),
    }


def residual_table(H_clean: np.ndarray, H_raw: np.ndarray, H_sic: np.ndarray,
                   op: Operator, sets: Optional[Dict[str, Dict]] = None,
                   roi_reps: Optional[Dict[str, Sequence[int]]] = None) -> Dict:
    """Per-N accumulation metrics for each repetition set and each ROI.

    ``sets`` maps a set name to a ``derive_rep_set``/``legacy_nominal_index_set``
    /``full_preamble_set`` dict.  Default (compat): full preamble + the legacy
    ``[8,8+N)`` diagnostic.

    ``roi_reps`` (appendix only) supplies a per-ROI repetition list; when a ROI
    list is shorter than N the entry is computed with the shorter prefix and
    flagged ``roi_shortened`` (no empty padding).
    """
    rois = _roi_map(op)
    if sets is None:
        sets = {"full": full_preamble_set(op),
                "indep": legacy_nominal_index_set(op)}
    out: Dict = {"sets": {}, "rois": rois,
                 "tap_domain": {"tap_lo": int(op.tap_lo),
                                "tap_hi": int(op.tap_hi),
                                "origin": int(op.origin),
                                "roi_target_offsets": [int(TARGET_LO),
                                                       int(TARGET_HI)],
                                "roi_target_tap": int(TARGET_OFFSET),
                                "roi_background_offsets": [int(BG_LO),
                                                           int(BG_HI)],
                                "receiver_tap_offset": RX_TAP_OFFSET,
                                "schema_version": SCHEMA_VERSION}}
    for set_name, rs in sets.items():
        reps = np.asarray(rs["reps"], dtype=np.int64)
        out["sets"][set_name] = {
            "kind": rs.get("kind"), "first": rs.get("first"),
            "last": rs.get("last"), "available": rs.get("available"),
            "n_list": list(rs.get("n_list") or []),
            "n_nominal": list(rs.get("n_nominal") or []),
            "n_shortened": rs.get("n_shortened"),
            "strict_holdout": rs.get("strict_holdout"),
            "read_lo": rs.get("read_lo"), "read_hi": rs.get("read_hi"),
            "support_source": rs.get("support_source"),
            "bounds": rs.get("bounds"),
            "nz_support": rs.get("nz_support"),
            "excluded": rs.get("excluded"),
            "dropped_noncontiguous": rs.get("dropped_noncontiguous"),
        }
        if reps.size == 0:
            out[set_name] = {}
            continue
        e1: Dict[str, float] = {}
        for n in (rs.get("n_list") or []):
            n = int(n)
            rec: Dict = {"N": n, "start": int(reps[0]),
                         "stop": int(reps[min(n, reps.size) - 1]) + 1,
                         "n_actual": int(min(n, reps.size))}
            for rname, (lo, hi) in rois.items():
                if roi_reps is not None and rname in roi_reps:
                    rreps = np.asarray(roi_reps[rname], dtype=np.int64)
                    avail = min(n, rreps.size)
                    shortened = avail < n
                    idx = rreps[:avail] if avail > 0 else rreps[:0]
                else:
                    avail = min(n, reps.size)
                    shortened = False
                    idx = reps[:avail]
                if idx.size == 0:
                    rec.setdefault("c_cir_db", {})[rname] = None
                    rec.setdefault("c_cir_flag", {})[rname] = "na"
                    continue
                hc = accumulate(H_clean, idx)
                hr = accumulate(H_raw, idx)
                hs = accumulate(H_sic, idx)
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
                rec.setdefault("roi_shortened", {})[rname] = bool(shortened)
            if n == 1:
                e1["raw"] = rec["e_raw"]["full"]
                e1["sic"] = rec["e_sic"]["full"]
            gr, fr = _ratio_db(e1.get("raw", 0.0), rec["e_raw"]["full"])
            gs, fs = _ratio_db(e1.get("sic", 0.0), rec["e_sic"]["full"])
            rec["g_raw_db"] = gr
            rec["g_raw_flag"] = fr
            rec["g_sic_db"] = gs
            rec["g_sic_flag"] = fs
            rec["g_is_sic_gain"] = False
            rec["sum_conversion"] = int(rec["n_actual"])
            out.setdefault(set_name, {})[str(n)] = rec
    return out


# ---------------------------------------------------------------------------
# A-R02: coherence with same-dimension energy decomposition
# ---------------------------------------------------------------------------
def coherence_stats(E: np.ndarray, op: Operator,
                    roi="target") -> Dict:
    """Single-window coherence of a per-symbol residual tensor ``E[r, tap]``.

    Same-dimension decomposition (A-R02)::

        mu = (1/N) sum_r E_r
        E_coherent = N * ||mu||^2
        E_centered = sum_r ||E_r - mu||^2
        E_total    = E_coherent + E_centered     (== sum_r ||E_r||^2)
        coherent_fraction = E_coherent / E_total = eta

    ``mu`` is the mean, so ``||mu||^2`` is a per-symbol-average energy and is
    NOT additive with ``E_centered``; it is kept as ``mu_energy`` (and, for
    schema compatibility, as ``mean_energy``) with that unit note.  The old
    ``mean_fraction = ||mu||^2 / E_total`` was dimensionally wrong (it mixed a
    single-symbol average with the N-symbol total) and is now ``None`` with a
    ``superseded`` note.  ``E_centered`` is deterministic-structure energy, not
    proven random noise: a phase ramp lands there too.
    """
    E = np.asarray(E, dtype=np.complex128)
    if isinstance(roi, str):
        roi = {"target": (TARGET_LO, TARGET_HI),
               "background": (BG_LO, BG_HI),
               "all": (op.tap_lo, op.tap_hi)}[roi]
    lo, hi = int(roi[0] - op.tap_lo), int(roi[1] - op.tap_lo)
    seg = E[:, lo:hi]
    N = int(seg.shape[0])
    raw_total = float(np.sum(np.abs(seg) ** 2))
    if raw_total <= 0.0:
        return {"N": N, "roi": list(roi), "energy": 0.0, "e_total": 0.0,
                "e_coherent": 0.0, "e_centered": 0.0, "eta": None,
                "coherent_fraction": None, "eta_flag": "na",
                "mu_energy": 0.0, "mean_energy": 0.0,
                "mean_energy_note": "||mu||^2, per-symbol-average energy; not "
                                    "additive with e_centered",
                "mean_fraction": None,
                "mean_fraction_superseded": "superseded: schema1 "
                                            "mean_fraction was ||mu||^2/"
                                            "E_total (dimensionally wrong); "
                                            "use coherent_fraction",
                "lag": {}, "centered_lag": {},
                "phase_slope_rad_per_symbol": None,
                "phase_slope_hz": None, "dominant_tap": None,
                "energy_conservation_rel": None}
    mu = seg.mean(axis=0)
    mu_energy = float(np.sum(np.abs(mu) ** 2))
    e_coherent = float(N * mu_energy)
    e_centered = float(np.sum(np.abs(seg - mu) ** 2))
    e_total = e_coherent + e_centered
    eta = e_coherent / e_total if e_total > 0 else None
    lag: Dict[str, Optional[float]] = {}
    clag: Dict[str, Optional[float]] = {}
    for lg in range(1, min(N - 1, 8) + 1):
        a = seg[lg:]
        b = seg[:-lg]
        num = np.vdot(a, b)
        den = math.sqrt(float(np.sum(np.abs(a) ** 2))
                        * float(np.sum(np.abs(b) ** 2)))
        lag[str(lg)] = None if den <= 0 else float(abs(num) / den)
        ac = a - a.mean(axis=0)
        bc = b - b.mean(axis=0)
        numc = np.vdot(ac, bc)
        denc = math.sqrt(float(np.sum(np.abs(ac) ** 2))
                         * float(np.sum(np.abs(bc) ** 2)))
        clag[str(lg)] = None if denc <= 0 else float(abs(numc) / denc)
    mag = np.abs(seg).mean(axis=0)
    dt = int(np.argmax(mag))
    phase = np.unwrap(np.angle(seg[:, dt]))
    slope = float(np.polyfit(np.arange(N, dtype=np.float64), phase, 1)[0]) \
        if N >= 2 else 0.0
    slope_hz = slope / (2.0 * math.pi * (SPS / WORK_HZ))
    return {
        "N": N, "roi": list(roi), "energy": raw_total, "e_total": e_total,
        "e_coherent": e_coherent, "e_centered": e_centered,
        "eta": float(eta) if eta is not None else None,
        "coherent_fraction": float(eta) if eta is not None else None,
        "eta_flag": "numeric", "eta_random_reference": 1.0 / N,
        "mu_energy": mu_energy, "mean_energy": mu_energy,
        "mean_energy_note": "||mu||^2, per-symbol-average energy; not additive "
                            "with e_centered",
        "mean_fraction": None,
        "mean_fraction_superseded": "superseded: schema1 mean_fraction was "
                                    "||mu||^2/E_total (dimensionally wrong); "
                                    "use coherent_fraction",
        "energy_conservation_rel": (abs(e_total - raw_total) / raw_total
                                    if raw_total > 0 else None),
        "lag": lag, "centered_lag": clag,
        "phase_slope_rad_per_symbol": slope, "phase_slope_hz": slope_hz,
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
# QA self-test (S2-A instruction section 5, cases 1-6 + A-R01/A-R02 cases)
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
    """Run the fixed-operator QA cases; return the failure count."""
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
    check("qa6 full and legacy-index set boundaries",
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
          < 1e-12, "%.6g" % rec["nmse_raw"]["full"])

    # --- 7. A-R01: support-disjoint set derivation --------------------------
    nz = code_support(op.code)
    check("ar01 template support is the nonzero code indices",
          nz["nz_count"] == int(np.count_nonzero(op.code))
          and nz["nz_min"] == 0 and nz["nz_max"] == int(
              np.flatnonzero(np.abs(op.code) > 0)[-1]))
    # operator on the real geometry: origin 1997, visible_end 10126, sub 67022
    op_real = Operator(origin=PRE_GUARD_REF,
                       code=sampled_code_from_chips(_qa_chips()),
                       energy=64.0)
    rs = derive_rep_set(op_real, visible_end=10126, sub_lo=1997, sub_hi=67022,
                        n=157376, tap_lo=TAP_LO, tap_hi=TAP_HI,
                        name="sd", kind="support_disjoint")
    check("ar01 support-disjoint set = [9,62] (54 reps) on the real geometry",
          rs["first"] == 9 and rs["last"] == 62 and rs["available"] == 54,
          "[%d,%d] n=%d" % (rs["first"], rs["last"], rs["available"]))
    check("ar01 honest N list shortens 56 -> 54",
          rs["n_list"] == [1, 2, 4, 8, 16, 32, 54]
          and rs["n_shortened"] is True, str(rs["n_list"]))
    check("ar01 excluded r=8 reads below visible_end and r=63 beyond sub_hi",
          any(e["r"] == 8 and "reads_training_or_guard" in e["reasons"]
              for e in rs["excluded"])
          and any(e["r"] == 63
                  and "outside_applied_subtraction" in e["reasons"]
                  for e in rs["excluded"]))
    check("ar01 legacy [8,8+56) is NOT support-disjoint",
          legacy_nominal_index_set(op_real)["strict_holdout"] is False)
    # per-ROI sets are supersets of the full-grid set (narrower tap span)
    rs_t = derive_rep_set(op_real, visible_end=10126, sub_lo=1997,
                          sub_hi=67022, n=157376, tap_lo=TARGET_LO,
                          tap_hi=TARGET_HI, name="sd_t", kind="roi")
    rs_d = derive_rep_set(op_real, visible_end=10126, sub_lo=1997,
                          sub_hi=67022, n=157376, tap_lo=DIRECT_LO,
                          tap_hi=DIRECT_HI, name="sd_d", kind="roi")
    check("ar01 target/direct ROI sets cover the full-grid set",
          rs_t["first"] <= rs["first"] and rs_t["last"] >= rs["last"]
          and rs_d["first"] <= rs["first"] and rs_d["last"] >= rs["last"],
          "t=[%d,%d] d=[%d,%d] full=[%d,%d]"
          % (rs_t["first"], rs_t["last"], rs_d["first"], rs_d["last"],
             rs["first"], rs["last"]))
    # insufficient-symbol case must shorten, never pad
    rs_short = derive_rep_set(op_real, visible_end=10126, sub_lo=1997,
                              sub_hi=1997 + 20 * SPS, n=157376,
                              tap_lo=TAP_LO, tap_hi=TAP_HI, name="s",
                              kind="support_disjoint")
    check("ar01 insufficient symbols shorten the N list (no padding)",
          rs_short["available"] == rs_short["last"] - rs_short["first"] + 1
          and all(n <= rs_short["available"] for n in rs_short["n_list"]),
          str(rs_short["n_list"]))

    # boundary-contamination counterexample: mutate ONLY training/guard samples
    rng = np.random.default_rng(11)
    n = op_real.origin + op_real.sync_reps * op_real.sps + 4000
    y = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * 0.1
    y2 = y.copy()
    vlo, vhi = op_real.origin, 10126
    y2[vlo:vhi] += (rng.standard_normal(vhi - vlo)
                    + 1j * rng.standard_normal(vhi - vlo)) * 1.0
    H1 = fixed_operator_cir(y, op_real)
    H2 = fixed_operator_cir(y2, op_real)
    legal = np.asarray(rs["reps"])
    diff_legal = np.max(np.abs(H1[legal] - H2[legal])) if legal.size else 0.0
    first_illegal = rs["first"] - 1
    diff_illegal = float(np.max(np.abs(H1[first_illegal]
                                       - H2[first_illegal])))
    check("ar01 training/guard mutation leaves every support-disjoint row "
          "bit-identical", diff_legal == 0.0, "max|dH|=%.3g" % diff_legal)
    check("ar01 the old r=8 label DOES read training/guard samples",
          diff_illegal > 0.0, "max|dH|=%.3g" % diff_illegal)
    # negative tap / max positive tap / window clipping
    lo_r = read_support(op_real, rs["first"], TAP_LO, TAP_HI)
    hi_r = read_support(op_real, rs["last"], TAP_LO, TAP_HI)
    check("ar01 read support is half-open and ordered",
          lo_r[0] == rs["read_lo"] and hi_r[1] == rs["read_hi"]
          and lo_r[1] > lo_r[0] and hi_r[1] > lo_r[0])
    rs_clip = derive_rep_set(op_real, visible_end=10126, sub_lo=1997,
                             sub_hi=67022, n=rs["read_hi"], tap_lo=TAP_LO,
                             tap_hi=TAP_HI, name="c", kind="support_disjoint")
    check("ar01 window clipping only removes reps, never invents them",
          rs_clip["available"] <= rs["available"]
          and rs_clip["last"] <= rs["last"])

    # --- 8. A-R02: same-dimension energy decomposition ----------------------
    const = np.ones((64, 4), dtype=np.complex128)
    c1 = coherence_stats(const, op, roi="all")
    check("ar02 coherent_fraction == 1 for 64 identical residuals",
          abs(c1["coherent_fraction"] - 1.0) < 1e-12
          and c1["e_centered"] < 1e-24,
          "%.12f" % c1["coherent_fraction"])
    check("ar02 e_coherent = N * ||mu||^2 (not ||mu||^2)",
          abs(c1["e_coherent"] - 64 * c1["mu_energy"]) < 1e-12,
          "%.6g vs %.6g" % (c1["e_coherent"], c1["mu_energy"]))
    check("ar02 energy conservation e_total == sum||E||^2",
          abs(c1["e_total"] - c1["energy"]) <= 1e-9 * c1["energy"],
          "%.3g" % c1["energy_conservation_rel"])
    check("ar02 old mean_fraction is retired but its supersession is recorded",
          c1["mean_fraction"] is None
          and "superseded" in str(c1["mean_fraction_superseded"]))
    zero_mean = _synthetic_H(64, 4, seed=21)
    c2 = coherence_stats(zero_mean, op, roi=("all"))
    check("ar02 independent residual: coherent_fraction ~ 1/N",
          c2["coherent_fraction"] < 6.0 / 64,
          "%.4f" % c2["coherent_fraction"])
    ramp = np.exp(1j * 0.4 * np.arange(64))[:, None] * np.ones((1, 4))
    c3 = coherence_stats(ramp, op, roi="all")
    check("ar02 phase ramp: conservation holds while eta is low",
          abs(c3["e_total"] - c3["energy"]) <= 1e-9 * c3["energy"]
          and c3["coherent_fraction"] < 0.05
          and c3["e_centered"] > 0.9 * c3["e_total"],
          "eta=%.4f" % c3["coherent_fraction"])

    if verbose:
        print("self-test: %d case(s) failed" % len(fails))
    return len(fails)


def _qa_chips() -> np.ndarray:
    """Public HRP code 9 chips (as recovered from the simulator modulator)."""
    s = ("+..+...-.--..--+.+.+..-+-++.+....++-...+..-..--.-+.+.--.-+++.++...+-"
         ".+..-.++-.+++..-+..+.+.-.++-+--+......+.....-+....-.-...--+")
    return np.array([1 if c == "+" else (-1 if c == "-" else 0) for c in s],
                    dtype=np.int8)


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true",
                    help="run the fixed-operator QA cases")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(verbose=not args.quiet)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
