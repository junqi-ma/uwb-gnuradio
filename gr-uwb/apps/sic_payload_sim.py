#!/usr/bin/env python3
"""S2-B payload-collision signal generator (generator side, truth-bearing).

Roles are the REVERSE of ``sic_preamble_sim.SicSignalGenerator`` (S0/S1):

* desired  ``s`` : the full communication packet (code 10, 64 SYNC, SFD/PHR/
  random PSDU + FCS) — decoded by the code-10 demod profile;
* interferer ``i`` : a SENSING preamble-only finite SYNC train (code 9, 64 SYNC)
  placed entirely inside the comm payload data region.

``y = s + i + w``; the three-way clean/raw/sic pair shares the payload, channel,
sample grid, noise realization and amplitude scale (raw CF32, no per-path
peak/L2 renormalisation).

This module is a thin wrapper over the reusable, already-frozen helpers in
``sic_preamble_sim.py`` (geometry, ``apply_path`` PCHIP fractional delay, the
R07 finite-SYNC template builder, ``hrp_packet_source``, sha256 of arrays).  It
does **not** copy the SIC math or the demodulator.  Everything in ``truth`` is
generator/verifier only and is never handed to the online SIC or the demod.

FCS contract (matches ``mod::append_ieee_fcs`` / ``demod::crc16_802154``):
the 2-byte IEEE 802.15.4 CRC-16 is appended EXACTLY ONCE by this generator to
the random data bytes; ``append_fcs`` on the packet source stays False so the
reconstruction cannot append it twice.  The wire PSDU therefore includes the
FCS and the demodulator returns it verbatim (it never strips it).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

import sic_preamble_sim as S

WORK_HZ = S.WORK_HZ
SPS = S.SPS
CHIP_SAMPLE_STRIDE = S.CHIP_SAMPLE_STRIDE
PREAMBLE_CODE_LEN = S.PREAMBLE_CODE_LEN

DEFAULT_COMM_CODE = 10
DEFAULT_SENSING_CODE = 9
DEFAULT_SYNC_REPS = 64
DEFAULT_SFD_MODE = "4z2"
DEFAULT_PSDU_BYTES = 96      # wire PSDU length advertised by PHR (incl. 2B FCS)
DEFAULT_FCS_BYTES = 2
DEFAULT_COMM_DATA_BYTES = DEFAULT_PSDU_BYTES - DEFAULT_FCS_BYTES  # 94
DEFAULT_PEAK_AMPLITUDE = 0.8
SFD_SYMBOLS_4Z2 = 8
PHR_SYMBOLS = 21
PHR_CHIPS_PER_SYMBOL = 512
PAYLOAD_CHIPS_PER_SYMBOL = 64
SPS_PER_PHR_SYMBOL = PHR_CHIPS_PER_SYMBOL * S.SAMPLES_PER_CHIP   # 1024
SPS_PER_PAYLOAD_SYMBOL = PAYLOAD_CHIPS_PER_SYMBOL * S.SAMPLES_PER_CHIP  # 128

# Public geometry guard around the interferer inside the payload region.
DEFAULT_GUARD = 2048
# Public interferer start-jitter budget (samples) used to build the coarse ROI.
DEFAULT_START_JITTER = 256


def crc16_802154(data: Sequence[int]) -> int:
    """IEEE 802.15.4 reflected CRC-16, poly 0x8408, init 0 (mod::append_ieee_fcs)."""
    crc = 0
    for b in data:
        crc ^= int(b) & 0xFF
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_fcs(data: Sequence[int]) -> bytes:
    """Return ``data || fcs_le_low || fcs_le_high`` (append exactly once)."""
    d = bytes((int(b) & 0xFF) for b in data)
    crc = crc16_802154(d)
    return d + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def make_random_data(nbytes: int, seed: int,
                     force_markers: bool = True) -> bytes:
    """Deterministic random data that always contains 0x00 and 0xFF.

    The first two bytes are forced to 0x00/0xFF so every packet exercises the
    legal extreme values (the instruction forbids a repeated fixed PSDU); the
    remaining bytes are a deterministic per-seed stream.  ``force_markers``
    only makes the 00/FF guarantee explicit (it is present by construction).
    """
    rng = np.random.default_rng(int(seed))
    data = bytearray(
        int(x) for x in rng.integers(0, 256, size=nbytes, dtype=np.uint8))
    if nbytes >= 1:
        data[0] = 0x00
    if nbytes >= 2:
        data[1] = 0xFF
    return bytes(data)


def payload_bpm_symbols(psdu_bytes: int) -> int:
    if psdu_bytes <= 0:
        return 0
    bits = psdu_bytes * 8
    blocks = (bits + 329) // 330
    return bits + 48 * blocks


def comm_field_offsets(sync_reps: int, n_sfd: int, psdu_bytes: int) -> Dict[str, int]:
    """Absolute comm-packet field offsets in WORK samples."""
    sync_lo = 0
    sync_hi = sync_reps * SPS
    sfd_lo = sync_hi
    sfd_hi = sfd_lo + n_sfd * SPS
    phr_lo = sfd_hi
    phr_hi = phr_lo + PHR_SYMBOLS * SPS_PER_PHR_SYMBOL
    nsym = payload_bpm_symbols(psdu_bytes)
    pay_lo = phr_hi
    pay_hi = pay_lo + nsym * SPS_PER_PAYLOAD_SYMBOL
    return {
        "sync_lo": sync_lo, "sync_hi": sync_hi,
        "sfd_lo": sfd_lo, "sfd_hi": sfd_hi,
        "phr_lo": phr_lo, "phr_hi": phr_hi,
        "payload_lo": pay_lo, "payload_hi": pay_hi,
        "packet_len": pay_hi,
        "payload_bpm_symbols": nsym,
    }


def comm_packet_samples(uwb, psdu_wire: bytes, *, sync_reps: int,
                        sfd_mode: str, comm_code: int, pulse_shape: str,
                        peak_amplitude: float = DEFAULT_PEAK_AMPLITUDE,
                        pulse_taps_file: str = "") -> np.ndarray:
    """Build a real communication packet (code ``comm_code``) via the modulator."""
    src = uwb.hrp_packet_source(
        list(psdu_wire), int(sync_reps), str(sfd_mode), int(comm_code),
        float(peak_amplitude), 0.01, False, False, False,
        str(pulse_shape), 2.5, 200.0, str(pulse_taps_file))
    return np.array(src.samples(), dtype=np.complex64)


def build_finite_sync_template(uwb, *, code_index: int, sync_reps: int,
                               sfd_mode: str, pulse_shape: str,
                               peak_amplitude: float,
                               pulse_taps_file: str = "") -> Dict:
    """Build a finite ``sync_reps``-SYNC code-``code_index`` preamble train (R07).

    Recovers the ternary code from one shaped SYNC symbol of a real modulator
    packet (so it can never drift from the C++ table) and shapes the finite
    train directly, keeping only the SYNC-induced pulse-shaping tail.  Returns a
    dict with ``waveform``/``sync``/``meta`` and an independent
    ``np.convolve`` cross-check against ``shape_finite_sync``.
    """
    probe_data = bytes(range(64))  # arbitrary, payload never enters the SYNC
    full = comm_packet_samples(
        uwb, append_fcs(probe_data)[:max(8, sync_reps)], sync_reps=sync_reps,
        sfd_mode=sfd_mode, comm_code=code_index, pulse_shape=pulse_shape,
        peak_amplitude=peak_amplitude, pulse_taps_file=pulse_taps_file)
    taps = S.pulse_taps(uwb, pulse_shape, 2.5, 200.0)
    if taps is None:
        raise SystemExit("cannot materialize pulse taps for shape %r"
                         % pulse_shape)
    chips, diag = S.recover_preamble_code(full, taps)
    raw = S.shape_finite_sync(chips, sync_reps, taps)
    ref = S.shape_finite_sync_convolve(chips, sync_reps, taps)
    conv_diff = (float(np.max(np.abs(raw - ref))) if raw.size == ref.size
                 else float("inf"))
    peak = float(np.max(np.abs(raw)))
    scale = peak_amplitude / peak if peak > 0 else 0.0
    wave = (raw * scale).astype(np.complex64)
    meta = {
        "code_index": int(code_index), "sync_reps": int(sync_reps),
        "pulse_shape": pulse_shape, "pulse_taps": int(taps.size),
        "natural_length": int(S.natural_sync_length(sync_reps, taps.size)),
        "recovery": diag, "convolve_max_abs_diff": conv_diff,
        "sha256": S.sha256_array(wave),
        "chip_sha256": S.sha256_array(chips.astype(np.complex64)),
        "peak": float(np.max(np.abs(wave))),
        "energy": float(np.sum(np.abs(wave) ** 2)),
    }
    return {
        "waveform": wave,
        "sync": wave[:SPS].astype(np.complex64).copy(),
        "chips": chips,
        "taps": taps,
        "meta": meta,
        "sfd_lo": int(sync_reps) * SPS,
    }


@dataclass
class Placement:
    """Public placement of the interferer inside the comm payload region."""
    position: str          # early | center | late
    guard: int
    nominal_start: int     # public nominal interferer origin (work samples)
    start_jitter: int      # public jitter budget (>=0); injected |jitter|<=budget

    @staticmethod
    def compute(position: str, payload_lo: int, payload_hi: int,
                template_len: int, guard: int) -> "Placement":
        a, b = int(payload_lo), int(payload_hi)
        D = int(template_len)
        if b - a < D + 2 * int(guard):
            raise ValueError(
                "payload region [%d,%d) length %d < template %d + 2*guard %d; "
                "refuse to shrink the interferer" % (a, b, b - a, D, guard))
        if position == "early":
            start = a + int(guard)
        elif position == "late":
            start = b - int(guard) - D
        elif position == "center":
            start = a + (b - a - D) // 2
        else:
            raise ValueError("unknown position %r" % position)
        return Placement(position=position, guard=int(guard),
                         nominal_start=int(start), start_jitter=0)


class SicPayloadSignalGenerator:
    """Builds ``y = s + i + w`` windows for the comm-payload S2-B study."""

    def __init__(self, uwb, *, comm_code: int = DEFAULT_COMM_CODE,
                 sensing_code: int = DEFAULT_SENSING_CODE,
                 sync_reps: int = DEFAULT_SYNC_REPS,
                 sfd_mode: str = DEFAULT_SFD_MODE,
                 psdu_bytes: int = DEFAULT_PSDU_BYTES,
                 pulse_shape: str = "legacy",
                 peak_amplitude: float = DEFAULT_PEAK_AMPLITUDE,
                 comm_paths: Optional[Sequence] = None,
                 pre_us: float = 2.0, pad_us: float = 8.0,
                 range_m: float = 15.0, tail_us: float = 24.0,
                 position: str = "center", guard: int = DEFAULT_GUARD,
                 snr_db: Optional[float] = 30.0,
                 isr_db: Optional[float] = 10.0,
                 interferer_present: bool = True,
                 comm_present: bool = True,
                 frac_model: str = "pchip",
                 verbose: bool = True):
        if psdu_bytes > 127 or psdu_bytes < 4:
            raise SystemExit("psdu_bytes %d outside the profile capability [4,127]"
                             % psdu_bytes)
        self.uwb = uwb
        self.comm_code = int(comm_code)
        self.sensing_code = int(sensing_code)
        self.sync_reps = int(sync_reps)
        self.sfd_mode = str(sfd_mode)
        self.psdu_bytes = int(psdu_bytes)
        self.pulse_shape = str(pulse_shape)
        self.peak_amplitude = float(peak_amplitude)
        self.comm_paths = S.PathSpec.from_pairs(
            comm_paths if comm_paths is not None else [[0, [1.0, 0.0]]])
        self.position = str(position)
        self.guard = int(guard)
        self.snr_db = None if snr_db is None else float(snr_db)
        self.isr_db = None if isr_db is None else float(isr_db)
        self.interferer_present = bool(interferer_present)
        self.comm_present = bool(comm_present)
        self.frac_model = str(frac_model)

        # Reference probe packet (fixed data) for the payload-independent SYNC
        # template + geometry; real packets are built per window in build().
        self.probe_data = make_random_data(
            max(1, self.psdu_bytes - DEFAULT_FCS_BYTES), 0)
        self.probe_wire = append_fcs(self.probe_data)
        if len(self.probe_wire) != self.psdu_bytes:
            raise SystemExit("probe wire length %d != psdu_bytes %d"
                             % (len(self.probe_wire), self.psdu_bytes))
        self.comm_probe = comm_packet_samples(
            uwb, self.probe_wire, sync_reps=self.sync_reps,
            sfd_mode=self.sfd_mode, comm_code=self.comm_code,
            pulse_shape=self.pulse_shape, peak_amplitude=self.peak_amplitude)
        self.n_sfd = {"4z1": 4, "4z2": 8, "4z3": 16, "4z4": 32,
                      "ieee": 8, "decawave": 8}.get(self.sfd_mode)
        if self.n_sfd is None:
            raise SystemExit("unknown sfd_mode %r" % self.sfd_mode)
        self.fields = comm_field_offsets(self.sync_reps, self.n_sfd,
                                         self.psdu_bytes)
        if self.fields["packet_len"] != self.comm_probe.size:
            raise SystemExit(
                "geometry mismatch: computed packet_len=%d, modulator=%d "
                "(pulse tail? update comm_field_offsets)"
                % (self.fields["packet_len"], self.comm_probe.size))

        # Comm SYNC matched-filter template for the demod (code 10).
        self.comm_sync = self.comm_probe[:SPS].astype(np.complex64).copy()

        # Sensing interferer: code-9 finite SYNC train (preamble-only).
        self.interferer = build_finite_sync_template(
            uwb, code_index=self.sensing_code, sync_reps=self.sync_reps,
            sfd_mode=self.sfd_mode, pulse_shape=self.pulse_shape,
            peak_amplitude=self.peak_amplitude)
        self.i_wave = self.interferer["waveform"]
        self.i_sync = self.interferer["sync"]
        self.i_template_len = int(self.i_wave.size)

        # Window geometry: comm packet at pre_guard plus guards.
        self.geom = S.work_fullwin_geometry(
            WORK_HZ, pre_us, range_m, tail_us, pad_us, int(self.comm_probe.size))
        self.n = int(self.geom["rx_len"])
        self.pre = int(self.geom["pre_guard"])
        self.comm_start = int(self.pre)
        self.payload_lo = self.comm_start + self.fields["payload_lo"]
        self.payload_hi = self.comm_start + self.fields["payload_hi"]

        self.placement = Placement.compute(
            self.position, self.payload_lo, self.payload_hi,
            self.i_template_len, self.guard)
        self.i_nominal_start = int(self.placement.nominal_start)

        if verbose:
            print("payload generator: psdu=%dB packet=%d n=%d pre=%d "
                  "payload=[%d,%d) interferer_len=%d i_nominal=%d"
                  % (self.psdu_bytes, self.fields["packet_len"], self.n,
                     self.pre, self.payload_lo, self.payload_hi,
                     self.i_template_len, self.i_nominal_start), flush=True)

    # -- templates ---------------------------------------------------------
    def comm_sync_info(self) -> "S.TemplateInfo":
        return S.TemplateInfo(
            role="comm_sync", code_index=self.comm_code,
            sync_reps=self.sync_reps, pulse_shape=self.pulse_shape,
            pulse_taps=self.interferer["meta"]["pulse_taps"], pulse_center_taps=0,
            length=int(self.comm_sync.size),
            energy=float(np.sum(np.abs(self.comm_sync) ** 2)),
            peak=float(np.max(np.abs(self.comm_sync))),
            sha256=S.sha256_array(self.comm_sync), waveform=self.comm_sync)

    def interferer_template_info(self) -> "S.TemplateInfo":
        return S.TemplateInfo(
            role="sensing_preamble", code_index=self.sensing_code,
            sync_reps=self.sync_reps, pulse_shape=self.pulse_shape,
            pulse_taps=self.interferer["meta"]["pulse_taps"],
            pulse_center_taps=0, length=self.i_template_len,
            energy=float(np.sum(np.abs(self.i_wave) ** 2)),
            peak=float(np.max(np.abs(self.i_wave))),
            sha256=S.sha256_array(self.i_wave), waveform=self.i_wave,
            extra=dict(self.interferer["meta"]))

    def interferer_sync_info(self) -> "S.TemplateInfo":
        return S.TemplateInfo(
            role="sensing_sync", code_index=self.sensing_code,
            sync_reps=self.sync_reps, pulse_shape=self.pulse_shape,
            pulse_taps=self.interferer["meta"]["pulse_taps"],
            pulse_center_taps=0, length=int(self.i_sync.size),
            energy=float(np.sum(np.abs(self.i_sync) ** 2)),
            peak=float(np.max(np.abs(self.i_sync))),
            sha256=S.sha256_array(self.i_sync), waveform=self.i_sync)

    # -- public ROIs (no injection truth) ----------------------------------
    def public_rois(self, start_jitter: int = DEFAULT_START_JITTER,
                    post_tail: int = 100, fit_reps: int = 8
                    ) -> Dict[str, object]:
        """Public coarse ROIs + visibility prefix derived from profile/geometry.

        ``nominal_start`` is the pre-declared placement (early/center/late),
        which is a public configuration choice, not the injected truth.  The
        injected origin may differ by up to ``start_jitter`` samples.
        """
        j = max(0, int(start_jitter))
        nominal = self.i_nominal_start
        det_lo = max(0, nominal - j - 64)
        det_hi = min(self.n, nominal + j + 64 + SPS)
        sub_lo = max(0, nominal - j - 64)
        sub_hi = min(self.n, nominal + j + self.i_template_len + int(post_tail)
                     + 128)
        observed_end = min(self.n, nominal + j + int(fit_reps) * SPS + 64)
        return {
            "detect_roi_lo": int(det_lo), "detect_roi_hi": int(det_hi),
            "sub_roi_lo": int(sub_lo), "sub_roi_hi": int(sub_hi),
            "observed_end": int(observed_end),
            "start_jitter": int(j), "nominal_start": int(nominal),
            "guard": int(self.guard),
        }

    # -- signal ------------------------------------------------------------
    def build(self, *, seed: int, payload_seed: int, cfo_hz: float = 0.0,
              isr_db: Optional[float] = None, snr_db: Optional[float] = None,
              start_offset: int = 0,
              interferer_present: Optional[bool] = None,
              comm_paths: Optional[Sequence] = None,
              i_paths: Optional[Sequence] = None,
              interferer_template: Optional[np.ndarray] = None) -> Dict:
        """Build one ``y = s + i + w`` window plus its truth record.

        ``payload_seed`` supplies the per-packet random payload; ``seed`` the
        noise realization.  The injected interferer origin is
        ``i_nominal_start + start_offset``.
        """
        n = self.n
        present = (self.interferer_present if interferer_present is None
                   else bool(interferer_present))
        isr_eff = isr_db if isr_db is not None else self.isr_db
        snr_eff = snr_db if snr_db is not None else self.snr_db

        # --- desired comm packet (per-packet random payload) ---
        data = make_random_data(self.psdu_bytes - DEFAULT_FCS_BYTES,
                                int(payload_seed))
        psdu_wire = append_fcs(data)
        packet = comm_packet_samples(
            self.uwb, psdu_wire, sync_reps=self.sync_reps,
            sfd_mode=self.sfd_mode, comm_code=self.comm_code,
            pulse_shape=self.pulse_shape, peak_amplitude=self.peak_amplitude)
        if packet.size != self.fields["packet_len"]:
            raise SystemExit("packet length changed between packets")
        used_comm_paths = (S.PathSpec.from_pairs(comm_paths)
                           if comm_paths is not None else self.comm_paths)
        s = np.zeros(n, dtype=np.complex128)
        if self.comm_present:
            for p in used_comm_paths:
                S.apply_path(s, packet, self.comm_start, p.delay_samples,
                             p.gain, self.frac_model)

        # --- sensing interferer (code-9 preamble-only) ---
        iw = self.i_wave if interferer_template is None else interferer_template
        iw = np.asarray(iw, dtype=np.complex64)
        i = np.zeros(n, dtype=np.complex128)
        i_start = int(self.i_nominal_start + int(start_offset))
        used_i_paths: List[S.PathSpec] = []
        i_support = (0, 0)
        if present:
            used_i_paths = (S.PathSpec.from_pairs(i_paths)
                            if i_paths is not None else
                            [S.PathSpec(0.0, complex(1.0, 0.0))])
            for p in used_i_paths:
                S.apply_path(i, iw, i_start, p.delay_samples, p.gain,
                             self.frac_model)
            dmin = min(p.delay_samples for p in used_i_paths)
            dmax = max(p.delay_samples for p in used_i_paths)
            i_support = (max(0, int(math.floor(i_start + dmin))),
                         min(n, int(math.ceil(i_start + dmax + iw.size))))
            if cfo_hz:
                t = np.arange(n, dtype=np.float64) / WORK_HZ
                i *= np.exp(1j * 2.0 * math.pi * cfo_hz * t)

        # --- reference interval: comm payload region ---
        ref_lo = max(0, self.payload_lo)
        ref_hi = min(n, self.payload_hi)
        n_ref = max(1, ref_hi - ref_lo)
        e_s_ref = float(np.sum(np.abs(s[ref_lo:ref_hi]) ** 2))

        # --- ISR scaling over the true overlap of i support and payload ---
        overlap_lo = overlap_hi = 0
        isr_realized = None
        scale = 1.0
        if present:
            lo = max(int(i_support[0]), ref_lo)
            hi = min(int(i_support[1]), ref_hi)
            if hi > lo:
                overlap_lo, overlap_hi = lo, hi
                e_s_ov = float(np.sum(np.abs(s[lo:hi]) ** 2))
                e_i_ov = float(np.sum(np.abs(i[lo:hi]) ** 2))
                if e_i_ov > 0.0 and e_s_ov > 0.0 and isr_eff is not None:
                    scale = math.sqrt((e_s_ov * (10.0 ** (isr_eff / 10.0)))
                                      / e_i_ov)
                    i *= scale
                    isr_realized = 10.0 * math.log10(
                        e_i_ov * scale ** 2 / e_s_ov)

        # --- AWGN (same realization for clean/raw/sic) ---
        noise_std = 0.0
        if snr_eff is not None and e_s_ref > 0.0:
            noise_std = math.sqrt(
                e_s_ref / (2.0 * n_ref * (10.0 ** (snr_eff / 10.0))))
        rng = np.random.default_rng(int(seed))
        w = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * noise_std

        s64 = s.astype(np.complex64)
        i64 = i.astype(np.complex64)
        w64 = w.astype(np.complex64)
        if present:
            y = (s64 + i64 + w64).astype(np.complex64)
        else:
            i64 = np.zeros(n, dtype=np.complex64)
            y = (s64 + w64).astype(np.complex64)
        clean = (s64 + w64).astype(np.complex64)

        truth = {
            "seed": int(seed), "payload_seed": int(payload_seed),
            "comm_code": self.comm_code, "sensing_code": self.sensing_code,
            "interferer_present": present,
            "psdu_data_hex": data.hex().upper(),
            "psdu_wire_hex": psdu_wire.hex().upper(),
            "psdu_wire_len": len(psdu_wire),
            "fcs_appended_once": True,
            "psdu_has_fcs": True,
            "comm": {
                "start_sample": float(self.comm_start),
                "packet_len": int(packet.size),
                "paths": [{"delay_samples": p.delay_samples,
                           "gain": [p.gain.real, p.gain.imag]}
                          for p in used_comm_paths],
                "fields": dict(self.fields),
            },
            "interferer": {
                "start_sample": float(i_start),
                "cfo_hz": float(cfo_hz),
                "paths": [{"delay_samples": p.delay_samples,
                           "gain": [p.gain.real, p.gain.imag]}
                          for p in used_i_paths],
                "template_len": int(iw.size),
                "support": [int(i_support[0]), int(i_support[1])],
                "isr_db": isr_realized, "isr_db_target": isr_eff,
                "scale": float(scale),
                "frac_model": self.frac_model,
            },
            "reference": {"lo": int(ref_lo), "hi": int(ref_hi),
                          "snr_db_target": snr_eff,
                          "e_s": float(e_s_ref)},
            "overlap": {"lo": int(overlap_lo), "hi": int(overlap_hi),
                        "samples": int(overlap_hi - overlap_lo)},
            "noise_std": float(noise_std),
            "geometry": dict(self.geom),
        }
        return {
            "s": s64, "i": i64, "w": w64, "y": y, "clean": clean,
            "truth": truth, "isr_db": isr_realized,
            "overlap": truth["overlap"], "noise_std": noise_std,
            "packet": packet,
        }

    # -- training-region firewall facts ------------------------------------
    def training_span(self, start_offset: int = 0, fit_reps: int = 8
                      ) -> Tuple[int, int]:
        o = int(self.i_nominal_start + int(start_offset))
        return o, o + int(fit_reps) * SPS
