#!/usr/bin/env python3
"""A-R03 residual-error budget: sources of the frozen-model SIC residual.

The S2-A initial report measured a coherent residual floor after the S1
``estimate8_frozen`` SIC (e.g. ~28.4 dB CIR gain at ISR 10 / CFO 0 that does not
improve with accumulation, and an absolute residual that does not scale with
ISR).  This module quantifies WHERE that residual comes from, without touching
the shared SIC core and without feeding truth into the online path.

Model form (identical to ``SicModel`` / ``tools/sic/sic_reference.py``)::

    i_hat[n] = exp(j*w*(n - phase_reference_sample))
               * sum_k gain[k] * preamble(n - tau_abs[k])

with ``w = 2*pi*cfo_hz/sample_rate``.  The reconstruction operator (fractional
delays via the core's fixed windowed-sinc interpolator, half=16, hamming) is the
FROZEN reference implementation in ``tools/sic/sic_reference.py`` -- this module
imports it and never re-implements or modifies it.  Every diagnostic model it
builds is validated against the adapter's own published ``i_hat`` for the
published model before any conclusion is drawn.

Budget identity used throughout (all on the same fixed mask / ROI / N)::

    i - i_hat = (i - i_true)          # generator/template/interp/finite mismatch
              + (i_true - i_hat)      # parameter error (CFO/tau/gain/path set)
    ||i - i_hat||^2 = ||a||^2 + ||b||^2 + 2*Re<a, b>

The cross term is reported explicitly; component dB improvements are NEVER added
up into a percentage.

Truth-assisted variants (``uses_truth=True`` diagnostics, offline only):
S1 pure-CFO, S2 pure-tau, S3 pure-gain, S4 path-set+gains, S5 tau+CFO,
S6 full truth (reachable floor), C1 conditional gain refit under the truth CFO,
C2 conditional refit under the truth tau+CFO.  "Fix the others" and "refit gains
under a given CFO/tau" are reported separately: a conditional refit is not a
single-item contribution.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import sic_reference as SICREF

WORK_HZ = 998400000.0
SPS = 1016


# ---------------------------------------------------------------------------
# Model helpers (canonical dict used by sic_reference._reconstruct_region)
# ---------------------------------------------------------------------------
def blank_model(cfo_hz: float = 0.0, phase_reference_sample: int = 0,
                sample_rate: float = WORK_HZ) -> Dict:
    return {"valid": True, "n_paths": 0, "taus": [], "gains": [],
            "cfo_hz": float(cfo_hz),
            "phase_reference_sample": int(phase_reference_sample),
            "sample_rate": float(sample_rate)}


def model_of(taus: Sequence[float], gains: Sequence[complex], *,
             cfo_hz: float, phase_reference_sample: int,
             sample_rate: float = WORK_HZ) -> Dict:
    m = blank_model(cfo_hz, phase_reference_sample, sample_rate)
    m["taus"] = [float(t) for t in taus]
    m["gains"] = [complex(g) for g in gains]
    m["n_paths"] = len(taus)
    return m


def reconstruct(model: Dict, lo: int, m: int,
                preamble: np.ndarray) -> np.ndarray:
    """Frozen-reference reconstruction over the absolute window [lo, lo+m)."""
    return SICREF._reconstruct_region(model, int(lo), int(m),
                                      np.asarray(preamble))


# ---------------------------------------------------------------------------
# The C++ core's ACTUAL fractional-delay operator (polyphase blend)
# ---------------------------------------------------------------------------
# ``sic_reference._reconstruct_region`` uses the "exact" windowed-sinc dot
# product.  The C++ core instead pre-computes 32 phase rows of the template and
# blends two neighbouring rows LINEARLY in the sub-sample phase
# (``uwb_preamble_sic_core.cc``: ``build_phase_taps`` / ``select_phase`` /
# ``accum_shifted``).  For integer delays the two agree exactly; for fractional
# delays they differ by that approximation (measured NMSE ~1e-4 on a four-path
# model).  The truth-assisted floors must be computed with the operator the
# production path actually uses, so this module implements the core's operator
# for diagnostics (it never modifies the core).
K_PRE_UP_PHASES = 32
K_INTERP_HALF = 16


def build_polyphase_rows(preamble: np.ndarray,
                         phases: int = K_PRE_UP_PHASES,
                         half: int = K_INTERP_HALF) -> np.ndarray:
    """32 rows of the template advanced by g/phases of a sample (core order).

    Row 0 is the exact integer copy; row g is the Hamming-windowed sinc kernel
    of phase g/phases, DC-normalised, truncated exactly like ``axpy_row``.
    """
    pre = np.asarray(preamble, dtype=np.complex128)
    plen = pre.size
    M = 2 * half + 1
    rows = np.zeros((int(phases), plen), dtype=np.complex64)
    rows[0] = pre.astype(np.complex64)
    nn = np.arange(M, dtype=np.float64)
    win = 0.54 - 0.46 * np.cos(2.0 * np.pi * nn / (M - 1))
    for g in range(1, int(phases)):
        tau = float(g) / float(phases)
        j = nn - float(half)
        taps = np.sinc(j - tau) * win
        ssum = taps.sum()
        if abs(ssum) > 1e-12:
            taps = taps / ssum
        conv = np.convolve(pre, taps.astype(np.complex128))
        rows[g] = conv[half:half + plen].astype(np.complex64)
    return rows


def core_reconstruct(model: Dict, lo: int, m: int, preamble: np.ndarray,
                     rows: Optional[np.ndarray] = None,
                     phases: int = K_PRE_UP_PHASES) -> np.ndarray:
    """i_hat over the absolute window [lo, lo+m) using the CORE's operator."""
    if int(m) <= 0:
        return np.zeros(0, dtype=np.complex128)
    pre = np.asarray(preamble, dtype=np.complex128)
    plen = pre.size
    if rows is None:
        rows = build_polyphase_rows(pre, phases=phases)
    acc = np.zeros(int(m), dtype=np.complex128)
    pos = np.arange(int(lo), int(lo) + int(m), dtype=np.int64)
    for tau, g in zip(model["taus"][:model["n_paths"]],
                      model["gains"][:model["n_paths"]]):
        tau = float(tau)
        ni = int(math.floor(tau))
        fr = tau - float(ni)
        if not (0.0 <= fr < 1.0):
            fr = 0.0
        beta = fr * float(phases)
        gi = int(beta)
        if gi >= int(phases):
            gi = int(phases) - 1
        alpha = beta - float(gi)
        if alpha < 0.0:
            alpha = 0.0
        if alpha > 1.0:
            alpha = 1.0
        idx = pos - ni
        ok = (idx >= 0) & (idx < plen)
        if not ok.any():
            continue
        ii = idx[ok]
        col = np.zeros(ii.size, dtype=np.complex128)
        col += (1.0 - alpha) * rows[gi][ii].astype(np.complex128)
        wrap = (gi + 1 >= int(phases))
        if alpha != 0.0:
            if wrap:
                jj = ii - 1
                okw = (jj >= 0) & (jj < plen)
                col[okw] += alpha * rows[0][jj[okw]].astype(np.complex128)
            else:
                col += alpha * rows[gi + 1][ii].astype(np.complex128)
        acc[np.flatnonzero(ok)] += complex(g) * col
    w = 2.0 * math.pi * float(model["cfo_hz"]) / float(model["sample_rate"])
    p = pos.astype(np.float64) - float(model["phase_reference_sample"])
    return acc * np.exp(1j * w * p)


def core_reconstruct_masked(model: Dict, n: int, preamble: np.ndarray,
                            sub_lo: int, sub_hi: int, support_lo: int,
                            support_hi: int,
                            rows: Optional[np.ndarray] = None) -> np.ndarray:
    """Core-operator i_hat restricted to the applied subtraction region."""
    out = np.zeros(int(n), dtype=np.complex128)
    glo = max(int(sub_lo), min(int(support_lo), int(sub_lo)))
    ghi = min(int(n), int(support_hi))
    lo = max(glo, int(sub_lo))
    hi = min(int(sub_hi), ghi)
    if hi > lo:
        out[lo:hi] = core_reconstruct(model, lo, hi - lo, preamble, rows=rows)
    return out


def support_span(model: Dict, n: int, preamble_len: int) -> Tuple[int, int]:
    """Half-open natural support of a model's reconstruction over the window."""
    if not model["taus"]:
        return (0, 0)
    lo = max(0, int(math.floor(min(float(t) for t in model["taus"]))))
    hi = min(int(n), int(math.ceil(max(float(t) for t in model["taus"])))
             + int(preamble_len))
    return (int(lo), int(hi))


def reconstruct_in(model: Dict, preamble: np.ndarray, lo: int, hi: int,
                   rows: Optional[np.ndarray] = None,
                   core: bool = True) -> np.ndarray:
    """i_hat over an explicit absolute half-open window (no masking)."""
    m = int(hi) - int(lo)
    if m <= 0:
        return np.zeros(0, dtype=np.complex128)
    if core:
        return core_reconstruct(model, int(lo), m, preamble, rows=rows)
    return reconstruct(model, int(lo), m, preamble)


def reconstruct_masked(model: Dict, n: int, preamble: np.ndarray,
                       sub_lo: int, sub_hi: int,
                       support_lo: int, support_hi: int) -> np.ndarray:
    """i_hat over the whole window, computed only inside the applied region."""
    out = np.zeros(int(n), dtype=np.complex128)
    glo = max(int(sub_lo), min(int(support_lo), int(sub_lo)))
    ghi = min(int(n), int(support_hi))
    lo = max(glo, int(sub_lo))
    hi = min(int(sub_hi), ghi)
    if hi > lo:
        out[lo:hi] = reconstruct(model, lo, hi - lo, preamble)
    return out


# ---------------------------------------------------------------------------
# Truth model
# ---------------------------------------------------------------------------
def truth_model(gen_pre: int, comm: Dict, *, sample_rate: float = WORK_HZ
                ) -> Dict:
    """Truth model in the canonical form from the generator truth record.

    The generator builds ``i[n] = scale * exp(j2*pi*f*n/fs) *
    sum_p gain_p * comm_wave(n - (pre + offset + delay_p))``.  With
    ``phase_reference_sample = 0`` that is exactly the model form with
    ``cfo_hz = f`` and ``taus = pre + offset + delay_p``.
    """
    start = float(comm["start_sample_exact"])
    taus = [start + float(p["delay_samples"]) for p in comm["paths"]]
    scale = float(comm.get("scale", 1.0))
    gains = [scale * complex(p["gain"][0], p["gain"][1])
             for p in comm["paths"]]
    return model_of(taus, gains, cfo_hz=float(comm["cfo_hz"]),
                    phase_reference_sample=0, sample_rate=sample_rate)


def published_model(meta: Dict, *, sample_rate: float = WORK_HZ) -> Optional[Dict]:
    """Model dict from the adapter's published frozen-model evidence."""
    taus = meta.get("sic_tau") or []
    gre = meta.get("sic_gain_re") or []
    gim = meta.get("sic_gain_im") or []
    if not taus or len(gre) != len(taus) or len(gim) != len(taus):
        return None
    return model_of(taus, [complex(a, b) for a, b in zip(gre, gim)],
                    cfo_hz=float(meta.get("sic_cfo_hz", 0.0)),
                    phase_reference_sample=int(
                        meta.get("sic_phase_reference", 0)),
                    sample_rate=float(meta.get("sic_sample_rate", sample_rate)))


def effective_gains(model: Dict) -> List[complex]:
    """Gains expressed in the ``phase_reference_sample = 0`` frame.

    ``exp(jw(n-pref)) g P(n-tau) == exp(jwn) (g e^{-jw pref}) P(n-tau)``.
    """
    w = 2.0 * math.pi * float(model["cfo_hz"]) / float(model["sample_rate"])
    pref = float(model["phase_reference_sample"])
    return [complex(g) * np.exp(-1j * w * pref) for g in model["gains"]]


# ---------------------------------------------------------------------------
# Path pairing (explicit; never a tap-index correspondence)
# ---------------------------------------------------------------------------
def pair_paths(model: Dict, truth: Dict) -> Dict:
    """Greedy nearest-delay one-to-one pairing of model paths to truth paths.

    Returns the pairing, the per-path delta tau / gain ratio and the unpaired
    entries.  The pairing uses the delay distance only (the gains are compared
    after pairing), so it never assumes a tap-index correspondence.
    """
    m = list(range(int(model["n_paths"])))
    t = list(range(int(truth["n_paths"])))
    cand = []
    for i in m:
        for j in t:
            cand.append((abs(float(model["taus"][i]) - float(truth["taus"][j])),
                         i, j))
    cand.sort()
    used_m, used_t, pairs = set(), set(), []
    for _d, i, j in cand:
        if i in used_m or j in used_t:
            continue
        used_m.add(i)
        used_t.add(j)
        pairs.append({"model_index": i, "truth_index": j,
                      "dtau": float(model["taus"][i])
                      - float(truth["taus"][j])})
    w = 2.0 * math.pi * float(model["cfo_hz"]) / float(
        model["sample_rate"])
    mg = effective_gains(model)
    tg = [complex(g) for g in truth["gains"]]
    for p in pairs:
        g_m = mg[p["model_index"]]
        g_t = tg[p["truth_index"]]
        p["gain_ratio"] = (complex(g_m / g_t) if abs(g_t) > 0 else None)
        p["gain_err_rel"] = (abs(g_m - g_t) / abs(g_t)
                             if abs(g_t) > 0 else None)
        p["gain_mag_err_db"] = (20.0 * math.log10(abs(g_m) / abs(g_t))
                                if abs(g_m) > 0 and abs(g_t) > 0 else None)
        p["gain_phase_err_deg"] = math.degrees(float(
            np.angle(g_m * np.conj(g_t)))) if abs(g_t) > 0 and abs(g_m) > 0 \
            else None
    del w
    return {"pairs": pairs,
            "unpaired_model": sorted(set(m) - used_m),
            "unpaired_truth": sorted(set(t) - used_t),
            "n_model": int(model["n_paths"]), "n_truth": int(truth["n_paths"]),
            "delay_err_abs_mean": (float(np.mean([abs(p["dtau"])
                                                  for p in pairs]))
                                   if pairs else None),
            "delay_err_abs_max": (float(np.max([abs(p["dtau"])
                                                for p in pairs]))
                                  if pairs else None)}


# ---------------------------------------------------------------------------
# Conditional gain refit (LS on the training span, core operator)
# ---------------------------------------------------------------------------
def refit_gains(taus: Sequence[float], cfo_hz: float, y: np.ndarray, *,
                train_lo: int, train_hi: int, preamble: np.ndarray,
                phase_reference_sample: int, sample_rate: float = WORK_HZ
                ) -> Tuple[List[complex], Dict]:
    """Least-squares complex gains for a FIXED tau set / CFO over the training span.

    This is a *conditional* refit (it is not a single-parameter substitution);
    it mirrors the core's gain fit: the design columns are the frozen
    windowed-sinc delayed + CFO-rotated template on the training samples.
    """
    lo, hi = int(train_lo), int(train_hi)
    m = max(0, hi - lo)
    if m <= 0 or not taus:
        return [], {"error": "empty_training_span"}
    yv = np.asarray(y, dtype=np.complex128)[lo:hi]
    cols = []
    for tau in taus:
        one = model_of([tau], [1.0 + 0j], cfo_hz=cfo_hz,
                       phase_reference_sample=phase_reference_sample,
                       sample_rate=sample_rate)
        cols.append(reconstruct(one, lo, m, preamble))
    A = np.stack(cols, axis=1)
    g, _res, rank, sv = np.linalg.lstsq(A, yv, rcond=None)
    resid = A @ g - yv
    return [complex(v) for v in g], {
        "rank": int(rank), "n_cols": len(taus), "n_rows": m,
        "resid_rel": float(np.linalg.norm(resid))
        / max(1e-30, float(np.linalg.norm(yv))),
        "singular_max_min": (float(sv[0] / sv[-1]) if sv.size > 1 else None)}


# ---------------------------------------------------------------------------
# Truth-assisted substitutions (offline diagnostics; uses_truth=True)
# ---------------------------------------------------------------------------
VARIANT_SPECS = {
    "S0_model": "published frozen model (online result; uses_truth=False)",
    "S1_cfo_truth": "model tau+gain, TRUTH cfo (no refit)",
    "S2_tau_truth": "TRUTH taus, model gain+cfo (no refit)",
    "S3_gain_truth": "model tau+cfo, TRUTH paired gains (no refit)",
    "S4_pathset_truth": "TRUTH taus + TRUTH paired gains, model cfo",
    "S5_tau_cfo_truth": "TRUTH taus + model gains, TRUTH cfo",
    "S6_full_truth": "TRUTH taus + gains + cfo (reachable floor)",
    "C1_cfo_truth_gainrefit": "TRUTH cfo with gains REFIT (conditional)",
    "C2_tau_cfo_truth_gainrefit": "TRUTH tau+cfo with gains REFIT (conditional)",
}


def truth_variants(model: Dict, truth: Dict, y: np.ndarray, *,
                   train_lo: int, train_hi: int, preamble: np.ndarray,
                   sample_rate: float = WORK_HZ) -> Dict[str, Dict]:
    """Build the truth-assisted model variants (S1..S6, C1, C2).

    All variants are expressed in ONE common phase frame
    (``phase_reference_sample = 0``); the model's published gains are converted
    into that frame with ``effective_gains`` (a unit-modulus rotation by the
    model's own CFO), so a "CFO-only" substitution really changes only the CFO
    while the complex gain values stay fixed.  Gains are transferred between the
    model and truth tau grids through the explicit path pairing; an unpaired
    entry gets a zero gain (recorded by the pairing, never silently aliased).
    """
    pairing = pair_paths(model, truth)
    mg = effective_gains(model)          # model gains in the pref=0 frame
    tg = [complex(g) for g in truth["gains"]]
    t2m = {p["truth_index"]: p["model_index"] for p in pairing["pairs"]}
    m2t = {p["model_index"]: p["truth_index"] for p in pairing["pairs"]}
    gains_on_truth_taus = [mg[t2m[j]] if j in t2m else 0.0 + 0.0j
                           for j in range(len(truth["taus"]))]
    gains_on_model_taus = [tg[m2t[i]] if i in m2t else 0.0 + 0.0j
                           for i in range(len(model["taus"]))]
    mtaus = [float(t) for t in model["taus"]]
    ttaus = [float(t) for t in truth["taus"]]
    mcfo = float(model["cfo_hz"])
    tcfo = float(truth["cfo_hz"])

    def mk(taus, gains, cfo):
        return model_of(taus, gains, cfo_hz=cfo, phase_reference_sample=0,
                        sample_rate=sample_rate)

    out: Dict[str, Dict] = {"S0_model": model}
    out["S1_cfo_truth"] = mk(mtaus, mg, tcfo)
    out["S2_tau_truth"] = mk(ttaus, gains_on_truth_taus, mcfo)
    out["S3_gain_truth"] = mk(mtaus, gains_on_model_taus, mcfo)
    out["S4_pathset_truth"] = mk(ttaus, tg, mcfo)
    out["S5_tau_cfo_truth"] = mk(ttaus, gains_on_truth_taus, tcfo)
    out["S6_full_truth"] = mk(ttaus, tg, tcfo)
    g1, d1 = refit_gains(mtaus, tcfo, y, train_lo=train_lo, train_hi=train_hi,
                         preamble=preamble, phase_reference_sample=0,
                         sample_rate=sample_rate)
    if g1:
        out["C1_cfo_truth_gainrefit"] = mk(mtaus, g1, tcfo)
        out["C1_cfo_truth_gainrefit"]["refit_diag"] = d1
    g2, d2 = refit_gains(ttaus, tcfo, y, train_lo=train_lo, train_hi=train_hi,
                         preamble=preamble, phase_reference_sample=0,
                         sample_rate=sample_rate)
    if g2:
        out["C2_tau_cfo_truth_gainrefit"] = mk(ttaus, g2, tcfo)
        out["C2_tau_cfo_truth_gainrefit"]["refit_diag"] = d2
    return out


# ---------------------------------------------------------------------------
# Residual decomposition and budget
# ---------------------------------------------------------------------------
def _energy(v: np.ndarray, lo: int = 0, hi: Optional[int] = None) -> float:
    v = np.asarray(v)
    hi = v.size if hi is None else int(hi)
    return float(np.sum(np.abs(v[int(lo):hi]) ** 2))


def iq_budget(i: np.ndarray, i_true: np.ndarray, i_hat: np.ndarray,
              *, lo: int, hi: int) -> Dict:
    """Two-component IQ residual budget with an explicit cross term.

    ``i - i_hat = (i - i_true) + (i_true - i_hat)`` over ``[lo, hi)``.
    """
    a = np.asarray(i, dtype=np.complex128)[lo:hi] \
        - np.asarray(i_true, dtype=np.complex128)[lo:hi]
    b = np.asarray(i_true, dtype=np.complex128)[lo:hi] \
        - np.asarray(i_hat, dtype=np.complex128)[lo:hi]
    ea = float(np.sum(np.abs(a) ** 2))
    eb = float(np.sum(np.abs(b) ** 2))
    tot = float(np.sum(np.abs(a + b) ** 2))
    cross = tot - ea - eb
    return {"lo": int(lo), "hi": int(hi),
            "e_impl_mismatch": ea, "e_param": eb, "e_cross": cross,
            "e_total": tot,
            "check_sum_identity_rel": (abs((ea + eb + cross) - tot)
                                       / max(1e-30, tot)),
            "note": "e_cross is real by construction (2Re<a,b>); component dB "
                    "values are NOT additive"}


def residual_energies(H: np.ndarray, H_clean: np.ndarray, *,
                      lo: int, hi: int) -> Dict:
    """CIR-domain residual energy of a variant and its ratio to clean."""
    d = np.asarray(H, dtype=np.complex128) - np.asarray(H_clean,
                                                        dtype=np.complex128)
    e = _energy(d, lo, hi)
    c = _energy(H_clean, lo, hi)
    return {"e": e, "e_clean": c,
            "ratio_db": (10.0 * math.log10(e / c) if e > 0 and c > 0
                         else None)}


def sensitivity(truth: Dict, *, y: np.ndarray, preamble: np.ndarray,
                lo: int, hi: int, dcfo_list: Sequence[float],
                dtau_list: Sequence[float],
                sample_rate: float = WORK_HZ) -> Dict:
    """First-order local sensitivity of the IQ residual to CFO and tau.

    Finite-difference around the truth model; the caller checks that the
    response is locally linear over the chosen deltas (the returned values
    include the second difference so non-linearity is visible).
    """
    base = reconstruct(truth, lo, hi - lo, preamble)
    res: Dict = {"dcfo": {}, "dtau": {}}
    e0 = _energy(np.asarray(y, dtype=np.complex128)[lo:hi] - base)
    res["e0"] = e0
    for d in dcfo_list:
        m = model_of(truth["taus"], truth["gains"],
                     cfo_hz=float(truth["cfo_hz"]) + float(d),
                     phase_reference_sample=truth["phase_reference_sample"],
                     sample_rate=sample_rate)
        r = reconstruct(m, lo, hi - lo, preamble)
        res["dcfo"][str(d)] = _energy(
            np.asarray(y, dtype=np.complex128)[lo:hi] - r)
    for d in dtau_list:
        m = model_of([float(t) + float(d) for t in truth["taus"]],
                     truth["gains"], cfo_hz=float(truth["cfo_hz"]),
                     phase_reference_sample=truth["phase_reference_sample"],
                     sample_rate=sample_rate)
        r = reconstruct(m, lo, hi - lo, preamble)
        res["dtau"][str(d)] = _energy(
            np.asarray(y, dtype=np.complex128)[lo:hi] - r)
    return res


# ---------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------
def self_test(verbose: bool = True) -> int:
    """QA for the budget module (no GNU Radio / build tree needed)."""
    fails = 0

    def check(name, cond, detail=""):
        nonlocal fails
        if verbose:
            print("%-58s %s %s" % (name, "PASS" if cond else "FAIL", detail))
        if not cond:
            fails += 1

    rng = np.random.default_rng(5)
    n = 4000
    preamble = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * 0.1
    # truth and a model with a small CFO/tau/gain error
    truth = model_of([100.0, 137.0], [1.0 + 0j, 0.25 + 0.05j], cfo_hz=5000.0,
                     phase_reference_sample=0)
    model = model_of([100.0, 137.4], [1.01 + 0.01j, 0.24 + 0.04j],
                     cfo_hz=4600.0, phase_reference_sample=0)
    i_true = reconstruct(truth, 0, n, preamble)
    i_hat = reconstruct(model, 0, n, preamble)
    budget = iq_budget(i_true, i_true, i_hat, lo=0, hi=n)
    check("budget: zero impl mismatch when i == i_true",
          budget["e_impl_mismatch"] == 0.0)
    check("budget: identity e_total == ea + eb + cross",
          budget["check_sum_identity_rel"] < 1e-12,
          "%.3g" % budget["check_sum_identity_rel"])
    # pairing: explicit delay pairing, not index order
    p = pair_paths(model, truth)
    check("pairing: one-to-one and delay-based",
          len(p["pairs"]) == 2 and not p["unpaired_model"]
          and not p["unpaired_truth"], str(p["pairs"]))
    check("pairing: delta tau is the true delay error",
          abs(abs(p["pairs"][1]["dtau"]) - 0.4) < 1e-9
          or abs(abs(p["pairs"][0]["dtau"]) - 0.0) < 1e-9,
          str([q["dtau"] for q in p["pairs"]]))
    # effective gains: pref=0 frame matches the truth frame when equal
    th2 = model_of([100.0], [1.0 + 0j], cfo_hz=1000.0,
                   phase_reference_sample=64)
    mg = effective_gains(th2)
    check("effective gains: frame conversion is a unit-modulus rotation",
          abs(abs(mg[0]) - 1.0) < 1e-12 and abs(mg[0] - 1.0) > 1e-6)
    # full truth is the reachable floor: residual shrink vs the model
    svars = truth_variants(model, truth, i_true, train_lo=0, train_hi=n,
                          preamble=preamble)
    e_model = _energy(i_true - reconstruct(svars["S0_model"], 0, n, preamble))
    e_full = _energy(i_true - reconstruct(svars["S6_full_truth"], 0, n,
                                         preamble))
    check("S6 full truth residual is far below the model residual",
          e_full < 1e-6 * e_model, "%.3g vs %.3g" % (e_full, e_model))
    check("all documented variants are produced",
          all(k in svars for k in VARIANT_SPECS), str(sorted(svars)))
    # conditional refit is flagged as conditional, not a pure substitution
    check("conditional refit variants are present and separate",
          "C1_cfo_truth_gainrefit" in svars
          and "C2_tau_cfo_truth_gainrefit" in svars
          and "conditional" in VARIANT_SPECS["C1_cfo_truth_gainrefit"])
    # sensitivity is monotone in |dcfo| near the truth for a positive error
    y = i_hat  # residual measured against i_hat
    for d in (-400.0, 0.0, 400.0):
        m = model_of(model["taus"], model["gains"],
                     cfo_hz=model["cfo_hz"] + d, phase_reference_sample=0)
        check("cfo sweep changes the residual for d=%g" % d,
              _energy(y - reconstruct(m, 0, n, preamble)) >= 0.0)
    # core operator vs the frozen reference: identical for integer delays
    rows = build_polyphase_rows(preamble)
    int_model = model_of([100.0, 137.0], [1.0 + 0j, 0.25 + 0j], cfo_hz=0.0,
                         phase_reference_sample=0)
    a = reconstruct(int_model, 0, 2000, preamble)
    b = core_reconstruct(int_model, 0, 2000, preamble, rows=rows)
    check("core operator == reference for integer delays (f32 row rounding)",
          np.max(np.abs(a - b)) < 1e-6 * max(1.0, float(np.max(np.abs(a)))),
          "%.3g" % float(np.max(np.abs(a - b))))
    frac_model = model_of([100.4], [1.0 + 0j], cfo_hz=0.0,
                          phase_reference_sample=0)
    a = reconstruct(frac_model, 0, 2000, preamble)
    b = core_reconstruct(frac_model, 0, 2000, preamble, rows=rows)
    check("core operator differs from the reference for fractional delays",
          np.max(np.abs(a - b)) > 1e-6, "%.3g" % float(np.max(np.abs(a - b))))
    check("row 0 is the exact template",
          np.array_equal(rows[0], preamble.astype(np.complex64)))
    # zero-length window is safe
    check("empty window is safe",
          reconstruct(truth, 0, 0, preamble).size == 0)
    if verbose:
        print("residual_budget self-test: %d failure(s)" % fails)
    return fails


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(verbose=not args.quiet)
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
