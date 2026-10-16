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


def sha256_array(a: np.ndarray) -> str:
    import hashlib
    return hashlib.sha256(np.ascontiguousarray(a, dtype=np.complex64)
                          .tobytes()).hexdigest()


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

        # Comm preamble-only waveform (code 10), keeping the start transient and
        # the shaping-filter tail.
        self.comm_src = None
        self.comm_wave = None
        self.comm_sync = None
        self.comm_template_len = 0
        if self.build_comm:
            self.comm_src = uwb.hrp_packet_source(
                psdu, self.sync_reps, sfd_mode, self.comm_code,
                0.8, 0.01, False, False, False,
                pulse_shape, 2.5, 200.0, pulse_taps_file)
            full = np.array(self.comm_src.samples(), dtype=np.complex64)
            self.comm_template_len = self.sync_reps * SPS + int(
                self.comm_src.pulse_taps())
            end = min(self.comm_template_len, full.size)
            self.comm_wave = full[:end].astype(np.complex64).copy()
            self.comm_sync = self.comm_wave[:SPS].astype(np.complex64).copy()

        if verbose:
            print("generator: tx=%d pre=%d pad=%d range=%d tail=%d n=%d "
                  "pulse_taps=%d center=%d comm_len=%s"
                  % (self.tx.size, self.pre, self.geom["pad"],
                     self.geom["range_guard"], self.geom["tail_guard"],
                     self.n, self.pulse_taps, self.pulse_center,
                     self.comm_wave.size if self.comm_wave is not None else 0),
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
            sha256=sha256_array(self.comm_wave), waveform=self.comm_wave)

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
            sha256=sha256_array(self.comm_sync), waveform=self.comm_sync)

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
