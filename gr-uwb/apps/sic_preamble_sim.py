#!/usr/bin/env python3
"""S0 preamble-collision simulation helpers (generator side, truth-bearing).

This module is the reusable signal/geometry core behind
``gr-uwb/apps/uwb_sim_preamble_sic.py``.  It reuses the geometry, HRP source
configuration, PCHIP fractional-delay model and AWGN convention of
``gr-uwb/apps/uwb_sim_cg400_echo_cir.py`` but:

* loads the ``uwb`` pybindings from an explicit ``--build-dir`` instead of the
  hard-coded ``gr-uwb/build`` (the old entry point is left untouched);
* works on the 998.4 MS/s **work grid** directly (S0 input-grid ``work``);
* models ``y = s + i + w`` with an *independent* comm preamble (code 10,
  64 SYNC, preamble-only) through an independent channel/CFO.

The comm signal never carries a PHR/payload/FCS and is never decoded.  Nothing
in this module is handed to the online SIC API; ``truth`` objects are for the
generator and the verifier only.
"""
from __future__ import annotations

import glob
import importlib.util
import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.interpolate import PchipInterpolator

C_LIGHT = 299792458.0
WORK_HZ = 998400000.0
CG400_HZ = 491520000.0
SPS = 1016                       # samples per SYNC symbol on the work grid
SFD_SYMS_4Z2 = 8
SFD_MODE = "4z2"
NATIVE_DECIM = 65
NATIVE_INTERP = 32

# --- HRP BPRF SYNC geometry (mirrors gr-uwb/include/gnuradio/uwb/
#     uwb_phy_profile.h + uwb_hrp_mod_core.h::add_sync_symbol) --------------
# One SYNC symbol is a length-127 ternary Ipatov code, spread by
# kQm35SpreadingFactor=4 chips and kQm35SamplesPerChip=2 samples per chip,
# giving kQm35ChipsPerSymbol=508 chips and kQm35SamplesPerSymbol=1016 samples.
SAMPLES_PER_CHIP = 2
SPREADING_FACTOR = 4
CHIPS_PER_SYMBOL = 508
PREAMBLE_CODE_LEN = 127
CHIP_SAMPLE_STRIDE = SPREADING_FACTOR * SAMPLES_PER_CHIP          # 8
assert CHIPS_PER_SYMBOL * SAMPLES_PER_CHIP == SPS
assert PREAMBLE_CODE_LEN * CHIP_SAMPLE_STRIDE <= SPS + CHIP_SAMPLE_STRIDE

DEFAULT_SENSING_CODE = 9
DEFAULT_COMM_CODE = 10
DEFAULT_SYNC_REPS = 64
DEFAULT_PSDU_HEX = "47261DF66F4C1BEF45C8F77CE77BD7D8C4D180FB1221"

# The canonical radar golden was produced by MATLAB lrwpanWaveformGenerator.
# `hrp_packet_source(pulse_shape="legacy")` reproduces it to <2e-7 (verified in
# S0-A G0); the CG400 sim's default `linear` external pulse does NOT.
DEFAULT_PULSE_SHAPE = "legacy"


def find_repo_root(start: Optional[str] = None) -> str:
    cur = os.path.abspath(start or os.path.dirname(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(cur, "testdata", "resampler_65_32")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise SystemExit("cannot find repo root from %s" % __file__)


def load_uwb(build_dir: str):
    """Load the OOT pybindings from *build_dir* (never the default build)."""
    bindir = os.path.join(build_dir, "python", "uwb", "bindings")
    matches = sorted(glob.glob(os.path.join(bindir, "uwb_python*.so")))
    if not matches:
        raise SystemExit("missing %s/uwb_python*.so; build gr-uwb python "
                         "bindings into --build-dir" % bindir)
    so = matches[-1]
    spec = importlib.util.spec_from_file_location("uwb_python", so)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def llround(x: float) -> int:
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def ceildiv(a: int, b: int) -> int:
    return -(-a // b)


def hex_to_bytes(s: str) -> List[int]:
    h = "".join(c for c in s if c in "0123456789abcdefABCDEF")
    if len(h) % 2:
        raise ValueError("psdu hex must have even length")
    return list(bytes.fromhex(h))


def work_fullwin_geometry(rate: float, pre_us: float, range_m: float,
                          tail_us: float, pad_us: float,
                          tx_samples: int) -> Dict[str, int]:
    """Work-grid full window geometry, mirroring the CG400 sim's fullwin.

    ``rx_len = pre_guard + tx + pad + range_guard + tail_guard`` rounded up to a
    multiple of 4 work samples.  ``pre_guard`` is the calibrated zero-delay
    origin used by both consumers.
    """
    pre = llround(pre_us * 1e-6 * rate)
    rng = int(math.ceil(2.0 * range_m / C_LIGHT * rate))
    tail = llround(tail_us * 1e-6 * rate)
    pad = llround(pad_us * 1e-6 * rate)
    n = pre + tx_samples + pad + rng + tail
    n = (n + 3) // 4 * 4
    return {"pre_guard": pre, "pad": pad, "range_guard": rng,
            "tail_guard": tail, "rx_len": n}


def apply_path(rx: np.ndarray, tx: np.ndarray, base: int, delay: float,
               gain: complex, frac_model: str = "pchip") -> None:
    """Add ``gain * tx`` delayed by ``delay`` samples starting at ``base``.

    Integer delays are exact integer shifts; fractional delays use PCHIP
    (default) or a windowed-sinc band-limited interpolator.  The generator
    records which model was used; the SIC core must not assume it.
    """
    off = base + delay
    if abs(delay - round(delay)) < 1e-9:
        start = int(round(off))
        k0 = max(0, -start)
        k1 = min(tx.size, rx.size - start)
        if k1 > k0:
            rx[start + k0:start + k1] += gain * tx[k0:k1]
        return

    # Fractional: place the integer part, then resample the shifted support.
    base_start = int(math.floor(off))
    frac = off - base_start
    k0 = max(0, -base_start)
    k1 = min(tx.size, rx.size - base_start)
    if k1 <= k0:
        return
    y = np.zeros(rx.size, dtype=np.complex128)
    y[base_start + k0:base_start + k1] = tx[k0:k1]
    xi = np.arange(rx.size, dtype=np.float64)
    if frac_model == "bandlimited":
        # Windowed-sinc fractional delay over the local support.
        half = 32
        taus = np.arange(-half, half + 1, dtype=np.float64)
        taps = np.sinc(taus - frac) * np.hamming(2 * half + 1)
        taps /= taps.sum()
        lo = max(0, base_start + k0 - half)
        hi = min(rx.size, base_start + k1 + half)
        conv = np.convolve(y[lo:hi], taps, mode="same")
        vals = np.zeros(rx.size, dtype=np.complex128)
        vals[lo:hi] = conv
    else:  # pchip
        xq = xi - frac
        idx = (xq >= 0.0) & (xq <= rx.size - 1)
        vals = np.zeros(rx.size, dtype=np.complex128)
        if np.any(idx):
            re = PchipInterpolator(xi, y.real)(xq[idx])
            im = PchipInterpolator(xi, y.imag)(xq[idx])
            vals[idx] = re + 1j * im
    rx += gain * vals


@dataclass
class PathSpec:
    delay_samples: float
    gain: complex

    @staticmethod
    def from_pairs(pairs: Sequence) -> List["PathSpec"]:
        out = []
        for p in pairs:
            if isinstance(p, dict):
                g = p["gain_complex"]
                out.append(PathSpec(float(p["delay_samples"]),
                                    complex(g[0], g[1])))
            else:
                d, g = p
                out.append(PathSpec(float(d), complex(g[0], g[1])))
        return out


@dataclass
class TemplateInfo:
    role: str
    code_index: int
    sync_reps: int
    pulse_shape: str
    pulse_taps: int
    pulse_center_taps: int
    length: int
    energy: float
    peak: float
    sha256: str
    waveform: np.ndarray = field(repr=False, default=None)
    extra: Dict = field(default_factory=dict, repr=False)


def sha256_array(a: np.ndarray) -> str:
    import hashlib
    return hashlib.sha256(np.ascontiguousarray(a, dtype=np.complex64)
                          .tobytes()).hexdigest()


# ---------------------------------------------------------------------------
# R07: FINITE SYNC code-pulse train (spread code -> chip/pulse train -> the
# same causal pulse-shaping FIR the HRP modulator uses).
#
# The comm "preamble" must be SYNC-only.  Truncating a full packet at
# `sync_reps*1016 + pulse_taps` leaks the SFD that begins immediately after the
# last SYNC chip into the so-called preamble tail.  Instead the finite 64-SYNC
# train is shaped directly, preserving ONLY the natural tail the shaping filter
# itself produces.  The ternary code is recovered from the same modulator by
# inverting its causal FIR on one shaped SYNC symbol (so it can never drift from
# the C++ table), and an independent np.convolve reference must agree at the
# sample level.
# ---------------------------------------------------------------------------
def pulse_taps(uwb, pulse_shape: str, sigma_ns: float = 2.5,
               bw_mhz: float = 200.0) -> Optional[np.ndarray]:
    """The active pulse-shaping FIR from the built pybindings (same source the
    modulator uses).  Returns None for shapes the binding cannot materialize
    (e.g. external taps), which forces the labeled packet-truncation fallback."""
    try:
        taps = uwb.make_pulse_taps(pulse_shape, float(sigma_ns), float(bw_mhz))
    except Exception:
        return None
    taps = np.asarray(taps, dtype=np.float64).ravel()
    return taps if taps.size >= 2 else None


def recovery_design_matrix(taps: np.ndarray, *, samples_per_symbol: int = SPS,
                           code_len: int = PREAMBLE_CODE_LEN) -> np.ndarray:
    """M x 127 convolution design matrix for one SYNC symbol.

    Column c places the causal pulse at chip c (sample offset c*8); the last
    column is clipped at the symbol boundary (its remaining taps are ~1e-7 and
    the clip contributes nothing above the fit tolerance)."""
    A = np.zeros((int(samples_per_symbol), int(code_len)), dtype=np.float64)
    for c in range(int(code_len)):
        off = c * CHIP_SAMPLE_STRIDE
        k = min(taps.size, int(samples_per_symbol) - off)
        if k > 0:
            A[off:off + k, c] = taps[:k]
    return A


def recover_preamble_code(packet_samples: np.ndarray, taps: np.ndarray, *,
                          samples_per_symbol: int = SPS,
                          code_len: int = PREAMBLE_CODE_LEN) -> Tuple[np.ndarray, Dict]:
    """Recover the ternary HRP preamble code from one shaped SYNC symbol.

    The waveform is the modulator's own causal FIR applied to a sparse chip
    train; solving that linear system recovers the chips (rank 127).  The result
    is rounded to {-1,0,+1} and the residual / integer deviation is returned so
    the caller can assert the recovery is exact, not approximate."""
    A = recovery_design_matrix(taps, samples_per_symbol=samples_per_symbol,
                               code_len=code_len)
    y = np.asarray(packet_samples[:samples_per_symbol], dtype=np.complex128)
    if np.max(np.abs(y.imag)) > 1e-6 * max(1.0, float(np.max(np.abs(y.real)))):
        raise ValueError("SYNC symbol is not real; unexpected modulator output")
    y = y.real.astype(np.float64)
    coef, _res, rank, _sv = np.linalg.lstsq(A, y, rcond=None)
    resid = A @ coef - y
    ynorm = float(np.linalg.norm(y))
    resid_rel = float(np.linalg.norm(resid)) / max(1e-30, ynorm)
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


def shape_finite_sync(chips: np.ndarray, sync_reps: int, taps: np.ndarray, *,
                      samples_per_symbol: int = SPS) -> np.ndarray:
    """Spread a code into a finite `sync_reps` SYNC train and apply the causal
    pulse-shaping FIR (exactly mirroring `add_shaped_chip`).

    Sample n receives chip c of repetition r as
    `taps[n - r*sps - c*8]`; the returned length is the natural end
    `(R-1)*sps + (127-1)*8 + len(taps)` so only the SYNC-induced filter tail is
    present (no SFD/PHR/payload sample can be included)."""
    chips = np.asarray(chips, dtype=np.int64).ravel()
    n_last = ((int(sync_reps) - 1) * int(samples_per_symbol)
              + (chips.size - 1) * CHIP_SAMPLE_STRIDE)
    n = n_last + int(taps.size)
    y = np.zeros(n, dtype=np.float64)
    for r in range(int(sync_reps)):
        base = r * int(samples_per_symbol)
        for c in range(chips.size):
            pc = int(chips[c])
            if pc == 0:
                continue
            off = base + c * CHIP_SAMPLE_STRIDE
            k = min(int(taps.size), n - off)
            if k > 0:
                y[off:off + k] += pc * taps[:k]
    return y


def shape_finite_sync_convolve(chips: np.ndarray, sync_reps: int,
                               taps: np.ndarray, *,
                               samples_per_symbol: int = SPS) -> np.ndarray:
    """Independent reference for `shape_finite_sync`: build the dense sparse
    chip train and convolve with np.convolve (no manual shift-add loop)."""
    chips = np.asarray(chips, dtype=np.float64).ravel()
    n_last = ((int(sync_reps) - 1) * int(samples_per_symbol)
              + (chips.size - 1) * CHIP_SAMPLE_STRIDE)
    dense = np.zeros(n_last + 1, dtype=np.float64)
    for r in range(int(sync_reps)):
        base = r * int(samples_per_symbol)
        for c in range(chips.size):
            if chips[c] != 0.0:
                dense[base + c * CHIP_SAMPLE_STRIDE] += chips[c]
    return np.convolve(dense, np.asarray(taps, dtype=np.float64))


def natural_sync_length(sync_reps: int, taps_len: int, *,
                        samples_per_symbol: int = SPS,
                        code_len: int = PREAMBLE_CODE_LEN) -> int:
    return ((int(sync_reps) - 1) * int(samples_per_symbol)
            + (int(code_len) - 1) * CHIP_SAMPLE_STRIDE + int(taps_len))


def build_comm_preamble(full_packet: np.ndarray, *, uwb, comm_code: int,
                        sync_reps: int, taps: Optional[np.ndarray],
                        pulse_shape: str, pulse_taps_file: str,
                        pulse_taps_count: int,
                        peak_amplitude: float
                        ) -> Tuple[Dict, Dict]:
    """Build the comm preamble-only waveform (R07).

    Primary path (finite SYNC): recover the ternary code from one shaped SYNC
    symbol, shape the finite `sync_reps` train directly, and verify it against
    (a) an independent np.convolve reference and (b) the modulator's own
    SFD-free packet prefix.  Fallback (labeled, only when the pulse taps are
    unavailable): the old packet truncation, named and with the leaked SFD range
    declared -- never silently presented as the finite template.

    Returns ``(meta, packet_trunc_meta)``; both carry a ``waveform`` (and the
    finite ``meta`` carries ``chips``) which the caller pops off.
    """
    full = np.ascontiguousarray(full_packet, dtype=np.complex64)
    sfd_lo = int(sync_reps) * SPS
    trunc_len = min(int(sync_reps) * SPS + int(pulse_taps_count), full.size)
    trunc = full[:trunc_len].astype(np.complex64).copy()
    trunc_meta = {
        "mode": "packet_truncation_diagnostic",
        "length": int(trunc.size),
        "sha256": sha256_array(trunc),
        "sfd_region_lo": int(sfd_lo),
        "sfd_region_hi": int(trunc.size),
        "sfd_leak_samples": int(max(0, trunc.size - sfd_lo)),
        "note": ("named diagnostic only; [sfd_region_lo, length) contains the "
                 "SFD that follows SYNC and MUST NOT be used as the online "
                 "comm template (R07)"),
        "waveform": trunc,
    }
    if taps is None:
        meta = {
            "mode": "packet_truncation_fallback",
            "reason": ("pulse taps unavailable for pulse_shape=%r "
                       "pulse_taps_file=%r; cannot reconstruct the finite SYNC "
                       "train independently" % (pulse_shape, pulse_taps_file)),
            "length": int(trunc.size),
            "energy": float(np.sum(np.abs(trunc) ** 2)),
            "peak": float(np.max(np.abs(trunc))),
            "sha256": sha256_array(trunc),
            "chips": None,
            "waveform": trunc,
        }
        return meta, trunc_meta

    chips, diag = recover_preamble_code(full, taps)
    raw = shape_finite_sync(chips, sync_reps, taps)
    raw_ref = shape_finite_sync_convolve(chips, sync_reps, taps)
    conv_diff = (float(np.max(np.abs(raw - raw_ref)))
                 if raw.size == raw_ref.size else float("inf"))
    # Independent reference #2: the modulator's SFD-free packet prefix.
    prefix = full[:sfd_lo].astype(np.complex128)
    rawc = raw.astype(np.complex128)
    m = min(prefix.size, rawc.size)
    denom = float(np.vdot(rawc[:m], rawc[:m]).real)
    ratio = (float(np.vdot(prefix[:m], rawc[:m]).real) / denom) if denom > 0 else 0.0
    resid = prefix[:m] - ratio * rawc[:m]
    prefix_rel_resid = (float(np.linalg.norm(resid))
                        / max(1e-30, float(np.linalg.norm(prefix[:m]))))
    sync_peak = float(np.max(np.abs(raw)))
    scale = (float(peak_amplitude) / sync_peak) if sync_peak > 0 else 0.0
    wave = (raw * scale).astype(np.complex64)
    meta = {
        "mode": "finite_sync_code_pulse",
        "length": int(wave.size),
        "energy": float(np.sum(np.abs(wave) ** 2)),
        "peak": float(np.max(np.abs(wave))),
        "sha256": sha256_array(wave),
        "chip_sha256": sha256_array(chips.astype(np.complex64)),
        "natural_length": int(natural_sync_length(sync_reps, taps.size)),
        "pulse_taps": int(taps.size),
        "pulse_shape": pulse_shape,
        "code_len": int(PREAMBLE_CODE_LEN),
        "chip_stride": int(CHIP_SAMPLE_STRIDE),
        "chips_nonzero": int(np.count_nonzero(chips)),
        "chip_energy": int(np.sum(chips.astype(np.int64) ** 2)),
        "normalization": {
            "method": "sync_train_peak",
            "peak_amplitude": float(peak_amplitude),
            "sync_train_peak": sync_peak,
            "scale": scale,
        },
        "recovery": diag,
        "reference": {
            "convolve_max_abs_diff": conv_diff,
            "packet_prefix_samples": int(prefix.size),
            "packet_prefix_ratio": ratio,
            "packet_prefix_rel_resid": prefix_rel_resid,
        },
        "packet_truncation": {
            "length": int(trunc.size),
            "sha256": sha256_array(trunc),
            "sfd_region_lo": int(sfd_lo),
            "sfd_leak_samples": int(max(0, trunc.size - sfd_lo)),
        },
        "chips": chips,
        "waveform": wave,
    }
    return meta, trunc_meta


def verify_comm_template(full_fn, meta: Dict, chips: np.ndarray,
                         taps: np.ndarray, *, sync_reps: int,
                         samples_per_symbol: int = SPS) -> Dict:
    """R07 closure evidence for one comm template.

    ``full_fn(psdu_hex, sfd_mode)`` must return the modulator's full packet for
    that variant.  Verifies: finite train == convolve reference; finite train ==
    the SFD-free packet prefix up to scale; length == natural length; and that
    changing SFD/PHR/payload does NOT change the finite template while it DOES
    change the packet-truncation diagnostic in the leaked SFD region.
    """
    out: Dict = {}
    raw = shape_finite_sync(chips, sync_reps, taps)
    raw_c = raw.astype(np.complex128)
    raw_ref = shape_finite_sync_convolve(chips, sync_reps, taps)
    out["convolve_ref_max_abs_diff"] = (
        float(np.max(np.abs(raw - raw_ref))) if raw.size == raw_ref.size
        else float("inf"))
    out["natural_length"] = int(natural_sync_length(sync_reps, taps.size))
    out["length_matches_natural"] = bool(
        raw.size == out["natural_length"] == int(meta["length"]))
    out["finite_template_sha256"] = meta.get("sha256")
    out["finite_template_independent_by_construction"] = True

    variants = [
        ("canonical_4z2", DEFAULT_PSDU_HEX, "4z2"),
        ("sfd_ieee", DEFAULT_PSDU_HEX, "ieee"),
        ("empty_psdu", "00", "4z2"),
        ("other_psdu", "DEADBEEF00", "4z2"),
    ]
    prefix_rel = {}
    trunc_hash = {}
    trunc_sfd_peak = {}
    payload_hash = {}
    for name, psdu_hex, sfd in variants:
        full = np.ascontiguousarray(full_fn(psdu_hex, sfd), dtype=np.complex64)
        sfd_lo = int(sync_reps) * samples_per_symbol
        prefix = full[:sfd_lo].astype(np.complex128)
        m = min(prefix.size, raw_c.size)
        den_m = float(np.vdot(raw_c[:m], raw_c[:m]).real)
        ratio = (float(np.vdot(prefix[:m], raw_c[:m]).real) / den_m
                 if den_m > 0 else 0.0)
        resid = prefix[:m] - ratio * raw_c[:m]
        prefix_rel[name] = (float(np.linalg.norm(resid))
                            / max(1e-30, float(np.linalg.norm(prefix[:m]))))
        tlen = min(sfd_lo + int(taps.size), full.size)
        trunc = full[:tlen].astype(np.complex64)
        trunc_hash[name] = sha256_array(trunc)
        tail = trunc[sfd_lo:]
        trunc_sfd_peak[name] = (float(np.max(np.abs(tail)))
                                if tail.size else 0.0)
        # Post-SFD content (PHR/payload): must depend on SFD/PSDU, proving the
        # packet is not SYNC-only and a truncation-based template would leak it.
        payload_hash[name] = sha256_array(
            full[sfd_lo:min(full.size, sfd_lo + 4096)].astype(np.complex64))
    out["packet_prefix_rel_resid"] = prefix_rel
    out["packet_prefix_rel_resid_max"] = (max(prefix_rel.values())
                                          if prefix_rel else None)
    out["packet_prefix_matches_finite"] = bool(
        out["packet_prefix_rel_resid_max"] is not None and
        out["packet_prefix_rel_resid_max"] <= 1e-5)
    out["packet_trunc_sha256_by_variant"] = trunc_hash
    out["packet_trunc_differs_across_variants"] = (
        len(set(trunc_hash.values())) > 1)
    out["packet_trunc_sfd_region_max_abs_by_variant"] = trunc_sfd_peak
    out["post_sfd_hash_by_variant"] = payload_hash
    out["post_sfd_depends_on_sfd_and_psdu"] = len(set(payload_hash.values())) > 1
    return out


# ---------------------------------------------------------------------------
# R08: CG400 native front-end helpers (491.52 MS/s RX / SC16 / 65-32).
#
# The native capture is a SINGLE quantized voltage.  SC16 quantization is a
# non-linear operator, so the per-component values are never quantized
# separately and summed: each *mixture* (clean = s+w, raw = s+i+w) is
# quantized once with one COMMON scale, and the naive ``q(s)+q(i)+q(w)`` is
# kept only as a labelled non-linearity diagnostic.  The 491.52 -> 998.4
# mapping mirrors the PDU 65/32 block contract:
#   Lout = ceil(((N-1)*65 + T)/32)
#   map(p) = round((p*65 + (T-1)/2)/32)
# ---------------------------------------------------------------------------
def work_to_native(x: np.ndarray) -> np.ndarray:
    """Decimate a 998.4 MS/s work waveform to 491.52 MS/s (32/65)."""
    from scipy.signal import resample_poly
    return resample_poly(np.asarray(x, dtype=np.complex128),
                         NATIVE_INTERP, NATIVE_DECIM)


def native_group_delay_input(tap_count: int) -> float:
    """Resampler group delay in NATIVE input samples.

    The PDU block is an UPSAMPLER native(491.52) -> work(998.4) with
    interp=65, decim=32, so the delay in its input units is (T-1)/(2*65)."""
    return (float(tap_count) - 1.0) / (2.0 * NATIVE_DECIM)


def native_group_delay_output(tap_count: int) -> float:
    """Resampler group delay in WORK output samples = (T-1)/(2*32)."""
    return (float(tap_count) - 1.0) / (2.0 * NATIVE_INTERP)


def native_map_input_to_output(p: int, tap_count: int) -> int:
    """``map(p) = round((p*65 + (T-1)/2)/32)`` (the PDU block contract)."""
    return int(round((float(p) * NATIVE_DECIM
                      + (float(tap_count) - 1.0) / 2.0) / NATIVE_INTERP))


def quantize_sc16(x: np.ndarray, scale: float):
    """Quantize one complex array with a GIVEN common scale.

    Returns ``(interleaved_i16, clips_i, clips_q, peak)``; the caller picks one
    scale for the whole window so every component shares it (no per-component
    normalization)."""
    a = np.asarray(x, dtype=np.complex128)
    ri = np.rint(a.real * scale)
    rq = np.rint(a.imag * scale)
    ci = int(np.count_nonzero((ri > 32767.0) | (ri < -32768.0)))
    cq = int(np.count_nonzero((rq > 32767.0) | (rq < -32768.0)))
    ii = np.clip(ri, -32768.0, 32767.0).astype(np.int16)
    qq = np.clip(rq, -32768.0, 32767.0).astype(np.int16)
    out = np.empty(2 * a.size, dtype=np.int16)
    out[0::2] = ii
    out[1::2] = qq
    peak = float(np.max(np.abs(a))) if a.size else 0.0
    return out, ci, cq, peak


def dequantize_sc16(interleaved: np.ndarray, scale: float) -> np.ndarray:
    a = np.asarray(interleaved, dtype=np.float64).astype(np.float64)
    re = a[0::2] / float(scale)
    im = a[1::2] / float(scale)
    return re + 1j * im


def common_sc16_scale(peaks, headroom: float = 1.0) -> float:
    """One scale shared by every component of a window (clip-free by default)."""
    pmax = max([float(p) for p in peaks if p is not None and p > 0.0] or [0.0])
    if pmax <= 0.0:
        return 1.0
    return (32767.0 * float(headroom)) / pmax


def quantize_nonlinearity(s_nat, i_nat, w_nat, scale: float) -> Dict:
    """Quantify ``q(s+i+w) - (q(s)+q(i)+q(w))`` (the SC16 non-linearity)."""
    mix, ci, cq, _pk = quantize_sc16(s_nat + i_nat + w_nat, scale)
    qsum = (dequantize_sc16(quantize_sc16(s_nat, scale)[0], scale)
            + dequantize_sc16(quantize_sc16(i_nat, scale)[0], scale)
            + dequantize_sc16(quantize_sc16(w_nat, scale)[0], scale))
    mixed = dequantize_sc16(mix, scale)
    d = mixed - qsum
    den = float(np.vdot(mixed, mixed).real)
    return {
        "mixture_clip_i": ci, "mixture_clip_q": cq,
        "max_abs_diff": float(np.max(np.abs(d))) if d.size else 0.0,
        "rms_diff": float(np.sqrt(np.mean(np.abs(d) ** 2))) if d.size else 0.0,
        "relative_rms": (float(np.sqrt(np.mean(np.abs(d) ** 2)))
                         / math.sqrt(den / max(1, mixed.size))
                         if den > 0 else None),
        "n_diff_samples": int(np.count_nonzero(d != 0)),
        "n_samples": int(d.size),
        "note": ("SC16 is non-linear: quantizing s/i/w separately and summing is "
                 "NOT the quantized mixture; only this diagnostic uses it."),
    }


def reference_resample_65_32(native_iq: np.ndarray,
                             taps: np.ndarray) -> np.ndarray:
    """Independent scalar reference for the 65/32 polyphase upsampler.

    Uses ``scipy.signal.upfirdn`` with the SAME taps (a different code path
    from the C++ core).  The exact block output is this reference shifted by
    the group delay, which the caller aligns by cross-correlation and records.
    """
    from scipy.signal import upfirdn
    # The PDU block upsamples native(491.52) -> work(998.4): up=65, down=32.
    # The taps already carry the x65 upsampling gain (sum == 65).
    h = np.asarray(taps, dtype=np.float64)
    x = np.asarray(native_iq, dtype=np.complex128)
    return upfirdn(h, x, up=NATIVE_DECIM, down=NATIVE_INTERP)



class SicSignalGenerator:
    """Generates ``y = s + i + w`` windows on the 998.4 MS/s work grid."""

    def __init__(self, uwb, *, rate: float = WORK_HZ,
                 sensing_code: int = DEFAULT_SENSING_CODE,
                 comm_code: int = DEFAULT_COMM_CODE,
                 sync_reps: int = DEFAULT_SYNC_REPS,
                 sfd_mode: str = SFD_MODE,
                 psdu_hex: str = DEFAULT_PSDU_HEX,
                 pulse_shape: str = DEFAULT_PULSE_SHAPE,
                 pulse_taps_file: str = "",
                 peak_amplitude: float = 0.8,
                 insert_sts: bool = False,
                 append_fcs: bool = False,
                 sensing_paths: Optional[Sequence] = None,
                 pre_us: float = 2.0, pad_us: float = 8.0,
                 range_m: float = 15.0, tail_us: float = 24.0,
                 snr_db: float = 30.0, isr_db: Optional[float] = 10.0,
                 comm_present: bool = False,
                 build_comm: Optional[bool] = None,
                 frac_model: str = "pchip",
                 verbose: bool = True):
        self.uwb = uwb
        self.rate = float(rate)
        self.sensing_code = int(sensing_code)
        self.comm_code = int(comm_code)
        self.sync_reps = int(sync_reps)
        self.sfd_mode = sfd_mode
        self.psdu_hex = psdu_hex
        self.pulse_shape = pulse_shape
        self.pulse_taps_file = pulse_taps_file
        self.peak_amplitude = float(peak_amplitude)
        self.comm_present = bool(comm_present)
        self.build_comm = (bool(comm_present) if build_comm is None
                           else bool(build_comm))
        self.snr_db = None if snr_db is None else float(snr_db)
        self.isr_db = None if isr_db is None else float(isr_db)
        self.frac_model = frac_model
        if sensing_paths is None:
            sensing_paths = [[0, [1.0, 0.0]], [37, [0.25, 0.0]]]
        self.sensing_paths = PathSpec.from_pairs(sensing_paths)

        psdu = hex_to_bytes(psdu_hex)
        common = dict(peak_amplitude=peak_amplitude, auto_emit=False,
                      pulse_shape=pulse_shape, pulse_sigma_ns=2.5,
                      pulse_bw_mhz=200.0, pulse_taps_file=pulse_taps_file)
        # Sensing full packet (code 9): decodable by UwbRealtimeDemodulator.
        self.sensing_src = uwb.hrp_packet_source(
            psdu, self.sync_reps, sfd_mode, self.sensing_code,
            0.8, 0.01, False, insert_sts, append_fcs,
            pulse_shape, 2.5, 200.0, pulse_taps_file)
        self.tx = np.array(self.sensing_src.samples(), dtype=np.complex64)
        self.pulse_taps = int(self.sensing_src.pulse_taps())
        self.pulse_center = int(self.sensing_src.pulse_center_taps())
        self.geom = work_fullwin_geometry(self.rate, pre_us, range_m, tail_us,
                                          pad_us, int(self.tx.size))
        self.n = int(self.geom["rx_len"])
        self.pre = int(self.geom["pre_guard"])

        # Code-9 SYNC matched-filter template: first 1016 samples of the packet
        # (matches testdata/uwb_radar/sync_template_998p4.cf32).
        self.template = self.tx[:SPS].astype(np.complex64).copy()

        # Comm preamble-only waveform (code 10), shaped DIRECTLY from the finite
        # `sync_reps` SYNC code-pulse train (R07).  Only the SYNC-induced
        # natural shaping-filter tail is kept; no SFD/PHR/payload sample can
        # enter.  A packet-truncation variant is retained ONLY as a named
        # diagnostic (with the leaked SFD range declared), never as the online
        # template.
        self.comm_src = None
        self.comm_wave = None
        self.comm_sync = None
        self.comm_template_len = 0
        self.comm_code_chips = None
        self.comm_pulse_taps = None
        self.comm_meta = {}
        self.comm_packet_trunc = None
        self.comm_packet_trunc_meta = {}
        if self.build_comm:
            self.comm_src = uwb.hrp_packet_source(
                psdu, self.sync_reps, sfd_mode, self.comm_code,
                0.8, 0.01, False, False, False,
                pulse_shape, 2.5, 200.0, pulse_taps_file)
            full = np.array(self.comm_src.samples(), dtype=np.complex64)
            taps = pulse_taps(uwb, pulse_shape, 2.5, 200.0)
            self.comm_pulse_taps = taps
            self.comm_meta, self.comm_packet_trunc_meta = \
                build_comm_preamble(
                    full, uwb=uwb, comm_code=self.comm_code,
                    sync_reps=self.sync_reps, taps=taps,
                    pulse_shape=pulse_shape, pulse_taps_file=pulse_taps_file,
                    pulse_taps_count=int(self.comm_src.pulse_taps()),
                    peak_amplitude=self.peak_amplitude)
            self.comm_wave = self.comm_meta.pop("waveform")
            self.comm_sync = self.comm_wave[:SPS].astype(np.complex64).copy()
            self.comm_code_chips = self.comm_meta.pop("chips")
            self.comm_template_len = int(self.comm_wave.size)
            if self.comm_packet_trunc_meta:
                self.comm_packet_trunc = \
                    self.comm_packet_trunc_meta.pop("waveform")

        if verbose:
            print("generator: tx=%d pre=%d pad=%d range=%d tail=%d n=%d "
                  "pulse_taps=%d center=%d comm_len=%s comm_mode=%s"
                  % (self.tx.size, self.pre, self.geom["pad"],
                     self.geom["range_guard"], self.geom["tail_guard"],
                     self.n, self.pulse_taps, self.pulse_center,
                     self.comm_wave.size if self.comm_wave is not None else 0,
                     self.comm_meta.get("mode", "-")),
                  flush=True)

    # -- templates ---------------------------------------------------------
    def sensing_template_info(self) -> TemplateInfo:
        return TemplateInfo(
            role="sensing_sync", code_index=self.sensing_code,
            sync_reps=self.sync_reps, pulse_shape=self.pulse_shape,
            pulse_taps=self.pulse_taps, pulse_center_taps=self.pulse_center,
            length=int(self.template.size),
            energy=float(np.sum(np.abs(self.template) ** 2)),
            peak=float(np.max(np.abs(self.template))),
            sha256=sha256_array(self.template), waveform=self.template)

    def comm_template_info(self) -> Optional[TemplateInfo]:
        if self.comm_wave is None:
            return None
        return TemplateInfo(
            role="comm_preamble", code_index=self.comm_code,
            sync_reps=self.sync_reps, pulse_shape=self.pulse_shape,
            pulse_taps=self.pulse_taps, pulse_center_taps=self.pulse_center,
            length=int(self.comm_wave.size),
            energy=float(np.sum(np.abs(self.comm_wave) ** 2)),
            peak=float(np.max(np.abs(self.comm_wave))),
            sha256=sha256_array(self.comm_wave), waveform=self.comm_wave,
            extra=dict(self.comm_meta))

    def comm_sync_info(self) -> Optional[TemplateInfo]:
        if self.comm_sync is None:
            return None
        return TemplateInfo(
            role="comm_sync", code_index=self.comm_code,
            sync_reps=self.sync_reps, pulse_shape=self.pulse_shape,
            pulse_taps=self.pulse_taps, pulse_center_taps=self.pulse_center,
            length=int(self.comm_sync.size),
            energy=float(np.sum(np.abs(self.comm_sync) ** 2)),
            peak=float(np.max(np.abs(self.comm_sync))),
            sha256=sha256_array(self.comm_sync), waveform=self.comm_sync,
            extra=dict(self.comm_meta))

    def comm_template_evidence(self) -> Optional[Dict]:
        """R07 closure evidence (finite code-pulse train vs independent
        references; SFD/PHR/payload independence; edge/tail checks)."""
        if self.comm_wave is None or self.comm_code_chips is None \
                or self.comm_pulse_taps is None:
            return None
        psdu = hex_to_bytes(self.psdu_hex)

        def full_fn(psdu_hex, sfd_mode):
            src = self.uwb.hrp_packet_source(
                hex_to_bytes(psdu_hex), self.sync_reps, sfd_mode,
                self.comm_code, 0.8, 0.01, False, False, False,
                self.pulse_shape, 2.5, 200.0, self.pulse_taps_file)
            return np.array(src.samples(), dtype=np.complex64)

        ev = verify_comm_template(full_fn, self.comm_meta,
                                  self.comm_code_chips, self.comm_pulse_taps,
                                  sync_reps=self.sync_reps)
        # Edge/tail checks on the finite train itself.
        wave = np.asarray(self.comm_wave, dtype=np.complex128)
        rec = self.comm_meta.get("recovery", {})
        ref = self.comm_meta.get("reference", {})
        sfd_lo = self.sync_reps * SPS
        tail_lo = sfd_lo - CHIP_SAMPLE_STRIDE        # last natural-symbol edge
        ev["edge"] = {
            "leading_edge_first8_energy": float(
                np.sum(np.abs(wave[:8]) ** 2)),
            "leading_edge_nonzero": bool(np.any(wave[:8] != 0)),
            "natural_tail_samples": int(self.comm_pulse_taps.size - 1),
            "tail_last_energy": float(
                np.sum(np.abs(wave[sfd_lo:]) ** 2)),
            "tail_present": bool(np.any(wave[sfd_lo:] != 0)),
            "symbol_edge_region": [int(tail_lo), int(sfd_lo)],
        }
        ev["recovery_rank_ok"] = bool(
            rec.get("rank") == rec.get("expected_rank"))
        ev["recovery_resid_rel"] = rec.get("resid_rel")
        ev["recovery_max_int_dev"] = rec.get("max_int_dev")
        ev["chip_nonzero"] = rec.get("nonzero_chips")
        ev["chip_energy"] = rec.get("chip_energy")
        ev["packet_prefix_rel_resid_inline"] = ref.get("packet_prefix_rel_resid")
        ev["pass"] = bool(
            ev["length_matches_natural"]
            and (ev["convolve_ref_max_abs_diff"] <= 1e-9)
            and ev["packet_prefix_matches_finite"]
            and ev["packet_trunc_differs_across_variants"]
            and ev["recovery_rank_ok"]
            and (rec.get("resid_rel", 1.0) <= 1e-6)
            and (rec.get("max_int_dev", 1.0) <= 1e-3)
            and (rec.get("nonzero_chips") == 64)
            and (rec.get("chip_energy") == 64)
            and ev["edge"]["leading_edge_nonzero"]
            and ev["edge"]["tail_present"])
        return ev


    # -- signal ------------------------------------------------------------
    def sensing_overlap_span(self) -> Tuple[int, int]:
        """Nominal clean-sensing SYNC span [lo, hi) on the window grid."""
        lo = self.pre
        hi = self.pre + self.sync_reps * SPS
        return lo, min(hi, self.n)

    def build(self, *, seed: int, comm_paths: Optional[Sequence] = None,
              start_offset_work: float = 0.0, cfo_hz: float = 0.0,
              isr_db: Optional[float] = None,
              snr_db: Optional[float] = None) -> Dict:
        """Build one ``y = s + i + w`` window plus its truth record."""
        n = self.n
        s = np.zeros(n, dtype=np.complex128)
        for p in self.sensing_paths:
            apply_path(s, self.tx, self.pre, p.delay_samples, p.gain,
                       self.frac_model)

        i = np.zeros(n, dtype=np.complex128)
        comm_start = float(self.pre) + float(start_offset_work)
        overlap_lo, overlap_hi = 0, 0
        isr_db_eff = isr_db if isr_db is not None else self.isr_db
        snr_db_eff = snr_db if snr_db is not None else self.snr_db
        scale = 1.0
        isr_realized = None
        comm_len = 0
        comm_used_paths: List[PathSpec] = []
        overlap_e_s = 0.0
        overlap_e_i_prescale = 0.0
        if self.comm_present and self.comm_wave is not None:
            comm_used_paths = (PathSpec.from_pairs(comm_paths)
                               if comm_paths else
                               PathSpec.from_pairs([[0, [1.0, 0.0]]]))
            raw = np.zeros(n, dtype=np.complex128)
            for p in comm_used_paths:
                apply_path(raw, self.comm_wave, self.pre, p.delay_samples,
                           p.gain, self.frac_model)
            # Independent whole-waveform start offset (a fractional offset uses
            # the same interpolation convention as the path delays).
            off = float(start_offset_work)
            if abs(off) > 1e-12:
                shifted = np.zeros(n, dtype=np.complex128)
                if abs(off - round(off)) < 1e-9:
                    sh = int(round(off))
                    k0 = max(0, -sh)
                    k1 = min(n, n - sh)
                    if k1 > k0:
                        shifted[sh + k0:sh + k1] = raw[k0:k1]
                else:
                    base = int(math.floor(off))
                    frac = off - base
                    xi = np.arange(n, dtype=np.float64)
                    xq = xi - frac
                    idx = (xq >= 0.0) & (xq <= n - 1)
                    if np.any(idx):
                        re = PchipInterpolator(xi, raw.real)(xq[idx])
                        im = PchipInterpolator(xi, raw.imag)(xq[idx])
                        shifted[idx] = re + 1j * im
                raw = shifted
            comm_len = int(self.comm_wave.size)
            if cfo_hz:
                t = np.arange(n, dtype=np.float64) / self.rate
                raw *= np.exp(1j * 2.0 * math.pi * cfo_hz * t)

            # Overlap = intersection of comm span and clean sensing SYNC span.
            lo = max(int(math.floor(comm_start)),
                     self.sensing_overlap_span()[0])
            hi = min(int(math.ceil(comm_start)) + comm_len,
                     self.sensing_overlap_span()[1])
            if hi > lo and lo < n:
                overlap_lo, overlap_hi = lo, min(hi, n)
                e_s = float(np.sum(np.abs(s[overlap_lo:overlap_hi]) ** 2))
                e_i = float(np.sum(np.abs(raw[overlap_lo:overlap_hi]) ** 2))
                overlap_e_s = e_s
                overlap_e_i_prescale = e_i
                if e_i > 0.0 and e_s > 0.0 and isr_db_eff is not None:
                    # ISR = interference-to-signal ratio over the overlapping
                    # SYNC interval (matches tools/sic/sic_reference.py:
                    # E_i / E_s = 10^(isr_db/10)).  A larger ISR means a
                    # STRONGER interferer.
                    scale = math.sqrt(
                        (e_s * (10.0 ** (isr_db_eff / 10.0))) / e_i)
                    raw *= scale
                    isr_realized = 10.0 * math.log10(e_i * scale ** 2 / e_s)
            i = raw

        # AWGN.  noise_std is the per-I/Q-component std; total complex noise
        # power is 2*sigma^2.  SNR is the ratio of the clean sensing energy to
        # the total noise energy over the reference interval, so
        #   sigma = sqrt(E_s / (2 * N_interval * 10^(SNR/10))).
        noise_std = 0.0
        ref_lo, ref_hi = self.sensing_overlap_span()
        if overlap_hi > overlap_lo:
            ref_lo, ref_hi = overlap_lo, overlap_hi
        n_ref = max(1, ref_hi - ref_lo)
        if snr_db_eff is not None:
            e_ref = float(np.sum(np.abs(s[ref_lo:ref_hi]) ** 2))
            if e_ref > 0.0:
                noise_std = math.sqrt(
                    e_ref / (2.0 * n_ref * (10.0 ** (snr_db_eff / 10.0))))
        rng = np.random.default_rng(int(seed))
        w = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * noise_std

        # CF32 components and the sum.  With interference off, `i` is exactly
        # zero and `y` is computed with the *same* expression as the clean
        # consumer (`s + w`) so the posted IQ is bit-identical.
        s64 = s.astype(np.complex64)
        i64 = i.astype(np.complex64)
        w64 = w.astype(np.complex64)
        if self.comm_present:
            y = (s64 + i64 + w64).astype(np.complex64)
        else:
            y = s64 + w64
        truth = {
            "seed": int(seed),
            "sensing_code": self.sensing_code,
            "comm_code": self.comm_code,
            "comm_present": self.comm_present,
            "comm": {
                "start_sample_exact": float(comm_start),
                "cfo_hz": float(cfo_hz),
                "paths": [{"delay_samples": p.delay_samples,
                           "gain": [p.gain.real, p.gain.imag]}
                          for p in comm_used_paths],
                "length_samples": comm_len,
                "isr_db": isr_realized,
                "isr_db_target": isr_db_eff,
                "scale": float(scale),
                "overlap_energy_clean": float(overlap_e_s),
                "overlap_energy_comm_prescale": float(overlap_e_i_prescale),
                "real_overlap_lo": int(overlap_lo),
                "real_overlap_hi": int(overlap_hi),
                "overlap_samples": int(overlap_hi - overlap_lo),
                "frac_model": self.frac_model,
            },
            "sensing": {
                "paths": [{"delay_samples": p.delay_samples,
                           "gain": [p.gain.real, p.gain.imag]}
                          for p in self.sensing_paths],
                "sync_origin": self.pre,
                "geometry": dict(self.geom),
            },
            "noise_std": noise_std,
        }
        return {
            "s": s64,
            "i": i64,
            "w": w64,
            "y": y,
            "truth": truth,
            "isr_db": isr_realized,
            "overlap": {"lo": int(overlap_lo), "hi": int(overlap_hi),
                        "samples": int(overlap_hi - overlap_lo)},
            "noise_std": noise_std,
        }
