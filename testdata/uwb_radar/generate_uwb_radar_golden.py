#!/usr/bin/env python3
"""Generate UWB monostatic-radar TX/RX/CIR golden files (Python fallback).

MATLAB Communications Toolbox `lrwpanWaveformGenerator` (see
export_uwb_radar_golden.m) is the generator of record.  This script is
what this machine can actually run: it splices a 4z2 SFD onto the
existing IEEE-SFD code-9 packet rather than synthesizing PHR/PSDU.

Native TX is one-shot 48/65 resample of the *entire* packet.  Do not
repeat the 751-sample native one-SYNC detector template — the true
native SYNC period is 1016*48/65 = 750.276923... samples.
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np
from scipy.ndimage import shift as nd_shift
from scipy.signal import resample_poly

HERE = os.path.dirname(os.path.abspath(__file__))
TESTDATA = os.path.dirname(HERE)
DEFAULT_SRC = os.path.join(
    TESTDATA, "uwb_code9_preamble64_payload128_standard_sfd.cfile"
)

FS_998P4 = 998_400_000.0
FS_737P28 = 737_280_000.0
INTERP = 48
DECIM = 65
RESAMPLE_HALF_LENGTH = 10
RESAMPLE_WINDOW = ("kaiser", 5.0)

CODE_INDEX = 9
SYNC_REPETITIONS = 64
SAMPLES_PER_SYMBOL = 1016
SFD_MODE = "4z2"
SFD_SEQUENCE = [-1, -1, -1, 1, -1, -1, 1, -1]
PEAK_AMPLITUDE = 0.8
RNG_SEED = 20260904  # MATLAB-of-record seed (this script does not draw PSDU)
SRC_PACKET_START = 4_992_000  # 0-based; testdata/make_reference_template.py
SRC_RNG_SEED = 20260807

C_M_PER_S = 299_792_458.0
RADAR_MAX_RANGE_M = 15.0
PRE_GUARD_S = 2e-6
TAIL_SAMPLES = 4096
CIR_SKIP = 10
CIR_RADAR_PRE = 16
CIR_DEMOD_PRE = 8
CIR_DEMOD_POST = 30

DELAY_INT_SAMPLES = 37
DELAY_INT_GAIN_MAG = 0.4
DELAY_INT_GAIN_PHASE = 0.7
DELAY_FRAC_SAMPLES = 12.4
DELAY_FRAC_GAIN_MAG = 0.55
DELAY_FRAC_GAIN_PHASE = -1.1
# Cubic spline, same family as MATLAB interp1(..., 'pchip') is not required;
# documented cubic (order-3 spline) so fractional-delay RX is deterministic.
DELAY_FRAC_INTERPOLATION = "cubic_spline_ndimage_order3"

# IEEE 802.15.4a HRP code-9 (Ipatov 127), same as kPreambleCode9.
HRP_CODE9 = np.array(
    [
        1, 0, 0, 1, 0, 0, 0, -1, 0, -1, -1, 0, 0, -1, -1, 1, 0, 1, 0, 1, 0, 0,
        -1, 1, -1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, -1, 0, 0, 0, 1, 0, 0, -1, 0,
        0, -1, -1, 0, -1, 1, 0, 1, 0, -1, -1, 0, -1, 1, 1, 1, 0, 1, 1, 0, 0, 0,
        1, -1, 0, 1, 0, 0, -1, 0, 1, 1, -1, 0, 1, 1, 1, 0, 0, -1, 1, 0, 0, 1,
        0, 1, 0, -1, 0, 1, 1, -1, 1, -1, -1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, -1, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, -1, 1,
    ],
    dtype=np.int8,
)


def write_cf32(path: str, x: np.ndarray) -> None:
    np.asarray(x, dtype=np.complex64).tofile(path)


def resample_group_delay_samples(up: int, down: int, half: int) -> float:
    ntaps = 2 * half * max(up, down) + 1
    return float(ntaps - 1) / 2.0 / float(down)


def build_sampled_code(code: np.ndarray, spreading: int = 4, spp: int = 2) -> np.ndarray:
    """Sparse 1016-sample sampled_code (spread-by-4, then SamplesPerPulse=2)."""
    spread = np.zeros(code.size * spreading, dtype=np.float64)
    spread[::spreading] = code.astype(np.float64)
    sampled = np.zeros(spread.size * spp, dtype=np.float64)
    sampled[::spp] = spread
    return sampled


def peak_normalize(x: np.ndarray, peak: float) -> np.ndarray:
    mag = float(np.max(np.abs(x)))
    if mag <= 0.0 or not np.isfinite(mag):
        raise RuntimeError("TX packet has no finite peak")
    return x * (peak / mag)


def assemble_tx_998p4(src: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Keep 64 original SYNCs, replace IEEE SFD with kron(4z2, first SYNC)."""
    if src.size < SRC_PACKET_START + SAMPLES_PER_SYMBOL:
        raise RuntimeError(f"source cfile too short: {src.size}")
    packet = np.asarray(src[SRC_PACKET_START:], dtype=np.complex128)
    sps = SAMPLES_PER_SYMBOL
    n_sfd = len(SFD_SEQUENCE)
    need = (SYNC_REPETITIONS + n_sfd) * sps
    if packet.size < need:
        raise RuntimeError(
            f"packet too short for 64 SYNC + 8 SFD: {packet.size} < {need}"
        )
    # Natural-amplitude first SYNC; do not L2-normalize the TX template.
    sync = packet[:sps].copy()
    sync_block = packet[: SYNC_REPETITIONS * sps]
    sfd = np.kron(np.asarray(SFD_SEQUENCE, dtype=np.float64), sync)
    phr_psdu = packet[need:]
    tx = np.concatenate([sync_block, sfd, phr_psdu])
    tx = peak_normalize(tx, PEAK_AMPLITUDE)
    return tx, sync


def embed_tx(
    tx: np.ndarray,
    pre_guard: int,
    tail: int,
    delay_samples: float,
    gain: complex,
    interpolation: str,
) -> np.ndarray:
    """Place TX in a fixed window: leading zeros, optional delay, trailing zeros."""
    n = pre_guard + int(tx.size) + tail
    buf = np.zeros(n, dtype=np.complex128)
    buf[pre_guard : pre_guard + int(tx.size)] = tx
    if delay_samples == int(delay_samples):
        d = int(delay_samples)
        out = np.zeros(n, dtype=np.complex128)
        src_lo = 0
        dst_lo = d
        count = n
        if dst_lo < 0:
            src_lo = -dst_lo
            dst_lo = 0
        copy_n = min(n - src_lo, n - dst_lo, count)
        if copy_n > 0:
            out[dst_lo : dst_lo + copy_n] = buf[src_lo : src_lo + copy_n]
        return (out * gain).astype(np.complex64)
    if interpolation != DELAY_FRAC_INTERPOLATION:
        raise RuntimeError(f"unsupported interpolation {interpolation}")
    # Positive shift moves energy toward higher indices (a delay).
    delayed_r = nd_shift(buf.real, shift=float(delay_samples), order=3, mode="constant", cval=0.0)
    delayed_i = nd_shift(buf.imag, shift=float(delay_samples), order=3, mode="constant", cval=0.0)
    return ((delayed_r + 1j * delayed_i) * gain).astype(np.complex64)


def estimate_cir(
    rx: np.ndarray,
    sync_origin: int,
    pre: int,
    post: int,
    sampled_code: np.ndarray,
    skip: int = CIR_SKIP,
    n_sync: int = SYNC_REPETITIONS,
    period: int = SAMPLES_PER_SYMBOL,
) -> dict:
    """Port of estimateCir.m without the final L2 step (raw) plus L2 (norm).

    Windows are aligned to `sync_origin` (predicted / TX-time origin), so a
    channel delay of D samples peaks at tap index pre+D.
    """
    rx = np.asarray(rx)
    code = np.asarray(sampled_code, dtype=np.complex128)
    code_energy = float(np.real(np.vdot(code, code)))
    if code_energy <= 0.0:
        raise RuntimeError("sampled_code energy is zero")
    tap_count = int(pre + post)
    wlen = int(code.size + tap_count - 1)
    acc = np.zeros(wlen, dtype=np.complex128)
    valid = 0
    for k in range(skip, n_sync):
        rs = int(sync_origin + k * period)
        lo = rs - int(pre)
        hi = lo + wlen
        if lo < 0 or hi > rx.size:
            continue
        acc += rx[lo:hi]
        valid += 1
    if valid == 0:
        raise RuntimeError("no complete SYNC windows for CIR")
    avg = acc / float(valid)
    raw = np.empty(tap_count, dtype=np.complex128)
    for n in range(tap_count):
        # conv(avg, flipud(conj(code)), 'valid') / code_energy
        raw[n] = np.vdot(code, avg[n : n + code.size]) / code_energy
    nrm_scale = float(np.linalg.norm(raw))
    normed = raw / (nrm_scale + np.finfo(np.float64).eps)
    peak_tap = int(np.argmax(np.abs(raw)))
    return {
        "raw": raw.astype(np.complex64),
        "norm": normed.astype(np.complex64),
        "valid_repetitions": valid,
        "code_energy": code_energy,
        "peak_tap": peak_tap,
        "peak_raw": complex(raw[peak_tap]),
        "raw_l2_norm": nrm_scale,
        "pre": int(pre),
        "post": int(post),
    }


def generate(src_path: str, out_dir: str) -> dict:
    os.makedirs(out_dir, exist_ok=True)
    src = np.fromfile(src_path, dtype=np.complex64)
    tx, _sync_natural = assemble_tx_998p4(src)
    tx_c64 = tx.astype(np.complex64)

    native = resample_poly(tx, INTERP, DECIM, window=RESAMPLE_WINDOW)
    native = np.asarray(native, dtype=np.complex64)
    gd = resample_group_delay_samples(INTERP, DECIM, RESAMPLE_HALF_LENGTH)

    pre_guard = int(round(PRE_GUARD_S * FS_998P4))
    cir_radar_post = int(np.ceil(2.0 * RADAR_MAX_RANGE_M / C_M_PER_S * FS_998P4))
    gain_int = DELAY_INT_GAIN_MAG * np.exp(1j * DELAY_INT_GAIN_PHASE)
    gain_frac = DELAY_FRAC_GAIN_MAG * np.exp(1j * DELAY_FRAC_GAIN_PHASE)

    rx_clean = embed_tx(tx, pre_guard, TAIL_SAMPLES, 0, 1.0 + 0.0j, DELAY_FRAC_INTERPOLATION)
    rx_delay_int = embed_tx(
        tx, pre_guard, TAIL_SAMPLES, DELAY_INT_SAMPLES, gain_int, DELAY_FRAC_INTERPOLATION
    )
    rx_delay_frac = embed_tx(
        tx, pre_guard, TAIL_SAMPLES, DELAY_FRAC_SAMPLES, gain_frac, DELAY_FRAC_INTERPOLATION
    )

    sampled_code = build_sampled_code(HRP_CODE9)
    # CIR uses the TX-time / predicted SYNC origin, not the delayed origin.
    origin_clean = pre_guard
    cir_clean_radar = estimate_cir(rx_clean, origin_clean, CIR_RADAR_PRE, cir_radar_post, sampled_code)
    cir_clean_830 = estimate_cir(rx_clean, origin_clean, CIR_DEMOD_PRE, CIR_DEMOD_POST, sampled_code)
    cir_int_radar = estimate_cir(rx_delay_int, origin_clean, CIR_RADAR_PRE, cir_radar_post, sampled_code)
    cir_int_830 = estimate_cir(rx_delay_int, origin_clean, CIR_DEMOD_PRE, CIR_DEMOD_POST, sampled_code)

    files = {
        "tx_998p4.cf32": tx_c64,
        "tx_737p28.cf32": native,
        "rx_clean_998p4.cf32": rx_clean,
        "rx_delay_int_998p4.cf32": rx_delay_int,
        "rx_delay_frac_998p4.cf32": rx_delay_frac,
        "cir_raw_clean_radar.cf32": cir_clean_radar["raw"],
        "cir_norm_clean_radar.cf32": cir_clean_radar["norm"],
        "cir_raw_clean_8_30.cf32": cir_clean_830["raw"],
        "cir_norm_clean_8_30.cf32": cir_clean_830["norm"],
        "cir_raw_delay_int_radar.cf32": cir_int_radar["raw"],
        "cir_norm_delay_int_radar.cf32": cir_int_radar["norm"],
        "cir_raw_delay_int_8_30.cf32": cir_int_830["raw"],
        "cir_norm_delay_int_8_30.cf32": cir_int_830["norm"],
    }
    for name, arr in files.items():
        write_cf32(os.path.join(out_dir, name), arr)

    sfd_tx = SYNC_REPETITIONS * SAMPLES_PER_SYMBOL
    native_peak = float(np.max(np.abs(native)))
    native_energy = float(np.sum(np.abs(native.astype(np.complex128)) ** 2))
    tx_peak = float(np.max(np.abs(tx_c64)))
    tx_energy = float(np.sum(np.abs(tx.astype(np.complex128)) ** 2))

    meta = {
        "description": (
            "UWB monostatic-radar golden: IEEE 802.15.4z BPRF / HRP code 9, "
            "64 SYNC, 4z2 SFD, full packet at 998.4 MS/s and one-shot 48/65 "
            "native 737.28 MS/s. Python splices 4z2 onto the existing IEEE-SFD "
            "packet; MATLAB lrwpanWaveformGenerator is the generator of record."
        ),
        "dtype": "complex64",
        "byte_order": "little-endian",
        "layout": "interleaved_iq",
        "sample_index_base": 0,
        "rate_work_hz": FS_998P4,
        "rate_native_hz": FS_737P28,
        "code_index": CODE_INDEX,
        "sync_repetitions": SYNC_REPETITIONS,
        "sfd_mode": SFD_MODE,
        "sfd_sequence": list(SFD_SEQUENCE),
        "samples_per_symbol": SAMPLES_PER_SYMBOL,
        "peak_amplitude": PEAK_AMPLITUDE,
        "rng_seed": RNG_SEED,
        "tx_length_998p4": int(tx_c64.size),
        "tx_length_737p28": int(native.size),
        "tx_998p4_peak": tx_peak,
        "tx_998p4_energy": tx_energy,
        "tx_737p28_peak": native_peak,
        "tx_737p28_energy": native_energy,
        "coordinates_0based": {
            "tx_998p4": {
                "sync_origin": 0,
                "sfd_start": int(sfd_tx),
            },
            "rx_clean_998p4": {
                "sync_origin": int(pre_guard),
                "sfd_start": int(pre_guard + sfd_tx),
            },
            "rx_delay_int_998p4": {
                "sync_origin": int(pre_guard + DELAY_INT_SAMPLES),
                "sfd_start": int(pre_guard + DELAY_INT_SAMPLES + sfd_tx),
            },
        },
        "resample": {
            "interp": INTERP,
            "decim": DECIM,
            "half_length": RESAMPLE_HALF_LENGTH,
            "window": ["kaiser", 5.0],
            "group_delay_samples": gd,
            "group_delay_domain": "native_737p28",
            "ntaps": 2 * RESAMPLE_HALF_LENGTH * max(INTERP, DECIM) + 1,
        },
        "rx_window": {
            "pre_guard_samples": int(pre_guard),
            "pre_guard_s": PRE_GUARD_S,
            "tail_samples": int(TAIL_SAMPLES),
            "length_998p4": int(rx_clean.size),
            "cfo_hz": 0.0,
        },
        "delay_int": {
            "samples": DELAY_INT_SAMPLES,
            "gain_mag": DELAY_INT_GAIN_MAG,
            "gain_phase_rad": DELAY_INT_GAIN_PHASE,
        },
        "delay_frac": {
            "samples": DELAY_FRAC_SAMPLES,
            "gain_mag": DELAY_FRAC_GAIN_MAG,
            "gain_phase_rad": DELAY_FRAC_GAIN_PHASE,
            "interpolation": DELAY_FRAC_INTERPOLATION,
        },
        "cir": {
            "algorithm": "estimateCir.m (skip, coherent average, forward sampled_code, /code_energy)",
            "sampled_code_length": int(sampled_code.size),
            "code_energy": cir_clean_radar["code_energy"],
            "skip_initial_repetitions": CIR_SKIP,
            "repetitions_used": SYNC_REPETITIONS - CIR_SKIP,
            "c_m_per_s": C_M_PER_S,
            "radar_max_range_m": RADAR_MAX_RANGE_M,
            "range_m_per_tap": C_M_PER_S / (2.0 * FS_998P4),
            "radar": {
                "pre": CIR_RADAR_PRE,
                "post": cir_radar_post,
                "tap_count": CIR_RADAR_PRE + cir_radar_post,
                "peak_tap_expected_clean": CIR_RADAR_PRE,
                "peak_tap_expected_delay_int": CIR_RADAR_PRE + DELAY_INT_SAMPLES,
                "peak_tap_measured_clean": cir_clean_radar["peak_tap"],
                "peak_tap_measured_delay_int": cir_int_radar["peak_tap"],
                "pulse_shape_offset_note": (
                    "sampled_code is unshaped; the HRP pulse peak is ~2 samples "
                    "later, so measured clean peak is typically pre+2"
                ),
                "valid_repetitions_clean": cir_clean_radar["valid_repetitions"],
                "valid_repetitions_delay_int": cir_int_radar["valid_repetitions"],
            },
            "demod_8_30": {
                "pre": CIR_DEMOD_PRE,
                "post": CIR_DEMOD_POST,
                "tap_count": CIR_DEMOD_PRE + CIR_DEMOD_POST,
                "peak_tap_expected_clean": CIR_DEMOD_PRE,
                "peak_tap_measured_clean": cir_clean_830["peak_tap"],
                "peak_tap_measured_delay_int": cir_int_830["peak_tap"],
                "note": (
                    "delay_int peak at pre+37 is outside post=30; 8/30 delay_int "
                    "CIR is exported for demod-window comparison only"
                ),
            },
        },
        "files": {name: int(arr.size) for name, arr in files.items()},
        "generator": "generate_uwb_radar_golden.py",
        "matlab_reference": "testdata/uwb_radar/export_uwb_radar_golden.m",
        "python_source_cfile": os.path.relpath(src_path, TESTDATA),
        "python_source_packet_start_0based": SRC_PACKET_START,
        "python_source_rng_seed": SRC_RNG_SEED,
        "note": (
            "MATLAB lrwpanWaveformGenerator is the generator of record when "
            "Communications Toolbox is available. This Python fallback splices "
            "4z2 (kron(sfd, first SYNC)) onto the existing IEEE-SFD packet and "
            "keeps the original PHR+PSDU. Native TX is resample_poly of the "
            "entire packet (48/65, Kaiser 5); do not repeat a 751-sample "
            "single-SYNC template. Native packet is not L2-normalized."
        ),
    }
    meta_path = os.path.join(out_dir, "metadata.json")
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, sort_keys=True)
        f.write("\n")
    return meta


def main() -> int:
    src = DEFAULT_SRC
    if len(sys.argv) > 1:
        src = sys.argv[1]
    out_dir = HERE
    if len(sys.argv) > 2:
        out_dir = sys.argv[2]
    meta = generate(src, out_dir)
    coords = meta["coordinates_0based"]
    cir = meta["cir"]["radar"]
    print("wrote testdata/uwb_radar goldens")
    print(
        "  tx_998p4={n} peak={p:.6f}  tx_737p28={m} peak={q:.6f} gd={gd:.6f}".format(
            n=meta["tx_length_998p4"],
            p=meta["tx_998p4_peak"],
            m=meta["tx_length_737p28"],
            q=meta["tx_737p28_peak"],
            gd=meta["resample"]["group_delay_samples"],
        )
    )
    print(
        "  sync/sfd tx={so}/{sfd}  clean={cso}/{csfd}  delay_int={dso}/{dsfd}".format(
            so=coords["tx_998p4"]["sync_origin"],
            sfd=coords["tx_998p4"]["sfd_start"],
            cso=coords["rx_clean_998p4"]["sync_origin"],
            csfd=coords["rx_clean_998p4"]["sfd_start"],
            dso=coords["rx_delay_int_998p4"]["sync_origin"],
            dsfd=coords["rx_delay_int_998p4"]["sfd_start"],
        )
    )
    print(
        "  CIR radar peak tap clean={c} (expected {ce}) delay_int={d} (expected {de})".format(
            c=cir["peak_tap_measured_clean"],
            ce=cir["peak_tap_expected_clean"],
            d=cir["peak_tap_measured_delay_int"],
            de=cir["peak_tap_expected_delay_int"],
        )
    )
    print("  metadata", os.path.join(out_dir, "metadata.json"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
