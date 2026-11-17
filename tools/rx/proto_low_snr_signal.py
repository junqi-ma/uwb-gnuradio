#!/usr/bin/env python3
"""Low-SNR lock-in study: shared signal generator (Python prototype).

Purpose
-------
Fast feasibility check for the plan
`docs/performance/低SNR进锁阶段优化与快速验证方案_2026-11-17.md`.
It produces the SAME fixtures for every experiment so that the acquisition
(修改 A) and the folded-timing (修改 B) prototypes are directly comparable.

What it builds
--------------
* 256-SYNC code-10 preamble at the 998.4 MS/s work grid, tiled from the
  canonical finite SYNC template (the HRP SYNC symbol is repeated verbatim,
  so tiling is the production waveform, not an approximation).
* 48/65 polyphase resampling to the 737.28 MS/s native grid with the same
  Kaiser (beta=5.0) prototype `resample_poly` uses in the earlier AWGN study
  (`docs/performance/AWGN_SNR仿真与阈值分析_2026-11-17.md`).
* Slots at a fixed period (default 1 ms) with the packet at the slot start.
* Optional AWGN at a specified IN-BAND (±250 MHz) SNR, then int16
  quantisation at a fixed packet RMS (542 LSB, the earlier study's scale).

Only numpy/scipy are required; no C++ build and no GNU Radio.

Usage
-----
    python3 tools/rx/proto_low_snr_signal.py --snr -14 --packets 20 \
        --out /tmp/opencode/m14.cf32

The Python API used by the experiment scripts:

    import proto_low_snr_signal as sig
    x, meta = sig.make_stream(snr_db=-14.0, packets=20, seed=1)
    #   x    : complex64 at 737.28 MS/s, length packets*period_samples
    #   meta : dict(packet_starts=[...], preamble_len=..., period_samples=...)
    tmpl = sig.load_sync_template()          # 1016 samples @ 998.4 MS/s
    pre  = sig.build_preamble_998p4(256)     # 256*1016 samples @ 998.4 MS/s
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TESTDATA = os.path.join(REPO, "testdata")

FS_NATIVE = 737.28e6
FS_WORK = 998.4e6
SAMPLES_PER_SYMBOL_WORK = 1016
SYNC_REPETITIONS = 256
PREAMBLE_LEN_WORK = SYNC_REPETITIONS * SAMPLES_PER_SYMBOL_WORK  # 260,096
# 48/65 resampling of the whole preamble -> 192,071 native samples.
PREAMBLE_LEN_NATIVE = int(np.ceil(PREAMBLE_LEN_WORK * 48.0 / 65.0))

SYNC_TEMPLATE = os.path.join(TESTDATA, "sic_preamble", "golden",
                             "comm_template", "comm_sync_code10_998p4.cf32")
PREAMBLE_TEMPLATE = os.path.join(TESTDATA, "sic_preamble", "golden",
                                 "comm_template",
                                 "comm_preamble_code10_998p4.cf32")

# In-band SNR is defined over the 500 MHz UWB channel inside the 737.28 MHz
# native band (same convention as tools/rx/measure_rx_snr.py).
INBAND_HZ = 500e6


def load_sync_template() -> np.ndarray:
    x = np.fromfile(SYNC_TEMPLATE, dtype=np.complex64)
    if x.size != SAMPLES_PER_SYMBOL_WORK:
        raise RuntimeError(f"unexpected SYNC template length {x.size}")
    return x


def build_preamble_998p4(repetitions: int = SYNC_REPETITIONS) -> np.ndarray:
    """Tile the canonical finite SYNC symbol (the preamble is periodic)."""
    sync = load_sync_template()
    return np.tile(sync, repetitions).astype(np.complex64)


def resample_to_native(x998: np.ndarray) -> np.ndarray:
    """48/65 polyphase resample 998.4 -> 737.28 MS/s (Kaiser beta=5.0)."""
    from scipy.signal import resample_poly
    return resample_poly(x998, 48, 65).astype(np.complex64)


def noise_power_for(snr_inband_db: float, rms_lsb: float = 542.0) -> float:
    """Total (full-band) noise power for an in-band SNR at packet RMS `rms_lsb`.

    The noise is white over the whole native band, so the in-band noise power
    is (500/737.28) of the total noise power.  `rms_lsb` is the PACKET rms,
    not the slot rms: the slot is mostly silence.
    """
    p_sig = float(rms_lsb) ** 2
    return p_sig * (FS_NATIVE / INBAND_HZ) / (10.0 ** (snr_inband_db / 10.0))


def add_awgn(x: np.ndarray, snr_inband_db: float, rms_lsb: float = 542.0,
             seed: int = 0) -> np.ndarray:
    """Add white noise at the requested in-band SNR (packet rms = `rms_lsb`)."""
    p_noise_total = noise_power_for(snr_inband_db, rms_lsb)
    rng = np.random.default_rng(seed)
    n = (rng.standard_normal(x.size) + 1j * rng.standard_normal(x.size))
    n *= np.sqrt(p_noise_total / 2.0)
    return (x + n).astype(np.complex64)


def quantize_sc16(x: np.ndarray) -> np.ndarray:
    """Round to int16 with clipping (no dither)."""
    i = np.clip(np.rint(x.real), -32768, 32767).astype(np.int16)
    q = np.clip(np.rint(x.imag), -32768, 32767).astype(np.int16)
    out = np.empty(2 * x.size, dtype=np.int16)
    out[0::2] = i
    out[1::2] = q
    return out


def make_stream(snr_db: float = None, packets: int = 20, period_ms: float = 1.0,
                seed: int = 1, rms_lsb: float = 542.0,
                preamble_reps: int = SYNC_REPETITIONS):
    """Build a native-rate stream with `packets` preambles at `period_ms`.

    `snr_db=None` produces pure AWGN (no packets) at the same noise power as
    a -14 dB stream, which is what the false-alarm test needs.
    """
    period = int(round(period_ms * 1e-3 * FS_NATIVE))
    pre_native = resample_to_native(build_preamble_998p4(preamble_reps))
    # Normalise the PACKET (not the slot) to the requested rms.
    pre_rms = float(np.sqrt(np.mean(np.abs(pre_native) ** 2)))
    pre_native = (pre_native * (rms_lsb / pre_rms)).astype(np.complex64)
    total = period * packets
    x = np.zeros(total, dtype=np.complex64)
    starts = []
    for k in range(packets):
        s = k * period
        starts.append(s)
        x[s:s + pre_native.size] += pre_native
    if snr_db is None:
        # Same noise power as the reference stream, signal removed.
        p_noise_total = noise_power_for(-14.0, rms_lsb)
        rng = np.random.default_rng(seed)
        n = (rng.standard_normal(x.size) + 1j * rng.standard_normal(x.size))
        n *= np.sqrt(p_noise_total / 2.0)
        x = n.astype(np.complex64)
    else:
        x = add_awgn(x, snr_db, rms_lsb=rms_lsb, seed=seed)
    meta = dict(snr_db=snr_db, packets=packets, period_samples=period,
                period_ms=period_ms, preamble_len_native=int(pre_native.size),
                preamble_len_work=PREAMBLE_LEN_WORK,
                sample_rate=FS_NATIVE, seed=seed, rms_lsb=rms_lsb,
                packet_starts=starts,
                inband_hz=INBAND_HZ)
    return x, meta


def make_stream_work(snr_db: float = None, packets: int = 20,
                     period_ms: float = 1.0, seed: int = 1,
                     rms_lsb: float = 542.0,
                     preamble_reps: int = SYNC_REPETITIONS):
    """Build a stream on the 998.4 MS/s WORK grid (no 48/65 resampling).

    Here the SYNC period is exactly 1016 samples, so folding is exact.  The
    in-band SNR uses the same 500 MHz convention, but the full band is now
    998.4 MHz, so the full-band noise is (998.4/500) of the in-band value.
    """
    period = int(round(period_ms * 1e-3 * FS_WORK))
    pre = build_preamble_998p4(preamble_reps)
    pre_rms = float(np.sqrt(np.mean(np.abs(pre) ** 2)))
    pre = (pre * (rms_lsb / pre_rms)).astype(np.complex64)
    x = np.zeros(period * packets, dtype=np.complex64)
    starts = []
    for k in range(packets):
        s = k * period
        starts.append(s)
        x[s:s + pre.size] += pre
    p_sig = float(rms_lsb) ** 2
    p_noise_total = p_sig * (FS_WORK / INBAND_HZ) / (10.0 ** ((snr_db if snr_db is not None else -14.0) / 10.0))
    rng = np.random.default_rng(seed)
    n = (rng.standard_normal(x.size) + 1j * rng.standard_normal(x.size))
    n *= np.sqrt(p_noise_total / 2.0)
    x = (x + n).astype(np.complex64)
    meta = dict(snr_db=snr_db, packets=packets, period_samples=period,
                period_ms=period_ms, preamble_len_work=int(pre.size),
                sample_rate=FS_WORK, seed=seed, rms_lsb=rms_lsb,
                packet_starts=starts, inband_hz=INBAND_HZ,
                samples_per_symbol=SAMPLES_PER_SYMBOL_WORK)
    return x, meta


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--snr", type=float, default=-14.0,
                    help="in-band SNR in dB (omit for pure noise: use --noise)")
    ap.add_argument("--noise", action="store_true", help="pure AWGN, no packets")
    ap.add_argument("--packets", type=int, default=20)
    ap.add_argument("--period-ms", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--rms", type=float, default=542.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--meta", default=None, help="JSON sidecar (default <out>.json)")
    ap.add_argument("--format", choices=["sc16", "cf32"], default="sc16")
    ap.add_argument("--work", action="store_true",
                    help="generate on the 998.4 MS/s work grid (no 48/65 resample)")
    a = ap.parse_args(argv)

    if a.work:
        x, meta = make_stream_work(None if a.noise else a.snr, a.packets,
                                   a.period_ms, a.seed, a.rms)
    else:
        x, meta = make_stream(None if a.noise else a.snr, a.packets, a.period_ms,
                              a.seed, a.rms)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    if a.format == "sc16":
        quantize_sc16(x).tofile(a.out)
    else:
        x.tofile(a.out)
    meta["format"] = a.format
    meta["path"] = os.path.abspath(a.out)
    side = a.meta or (a.out + ".json")
    with open(side, "w") as f:
        json.dump(meta, f, indent=1)
    print(f"wrote {a.out} ({x.size} samples, {x.size / FS_NATIVE * 1e3:.3f} ms) "
          f"snr={meta['snr_db']} meta={side}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
