#!/usr/bin/env python3
"""Remove a known continuous-wave tone from a complete raw SC16 file."""
from __future__ import annotations

import argparse
import json
import hashlib
import os
from pathlib import Path

import numpy as np


FS_DEFAULT = 737.28e6
CENTER_DEFAULT = 6489.6e6
TONE_RF_DEFAULT = 6256.640e6


def sha256_file(path: Path, chunk_bytes: int = 16 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(chunk_bytes)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def iter_chunks(mm: np.memmap, chunk_complex: int):
    for lo in range(0, mm.size, chunk_complex * 2):
        hi = min(lo + chunk_complex * 2, mm.size)
        values = np.asarray(mm[lo:hi], dtype=np.float64)
        sample_lo = lo // 2
        yield sample_lo, values[0::2] + 1j * values[1::2]


def tone_vector(sample_lo: int, count: int, fs: float, tone_hz: float):
    indices = np.arange(sample_lo, sample_lo + count, dtype=np.float64)
    return np.exp(2j * np.pi * tone_hz * indices / fs)


def remove_tone(input_path: Path,
                output_path: Path,
                fs: float,
                tone_hz: float,
                chunk_complex: int) -> dict:
    input_size = input_path.stat().st_size
    if input_size % 4:
        raise ValueError(f"input is not an integral SC16 stream: {input_path}")

    raw = np.memmap(input_path, dtype=np.int16, mode="r")
    sample_count = raw.size // 2

    # Fit one complex coefficient over the whole continuous capture.
    projection = 0j
    power_before = 0.0
    for sample_lo, x in iter_chunks(raw, chunk_complex):
        tone = tone_vector(sample_lo, x.size, fs, tone_hz)
        projection += np.vdot(tone, x)
        power_before += float(np.vdot(x, x).real)
    coefficient = projection / sample_count

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(output_path.name + ".tmp")
    if temporary.exists():
        temporary.unlink()

    power_after = 0.0
    residual_projection = 0j
    clipped_components = 0
    component_count = 0
    with temporary.open("wb") as stream:
        for sample_lo, x in iter_chunks(raw, chunk_complex):
            tone = tone_vector(sample_lo, x.size, fs, tone_hz)
            y = x - coefficient * tone
            power_after += float(np.vdot(y, y).real)
            residual_projection += np.vdot(tone, y)

            values = np.empty(y.size * 2, dtype=np.float64)
            values[0::2] = y.real
            values[1::2] = y.imag
            clipped = (values > 32767.0) | (values < -32768.0)
            clipped_components += int(np.count_nonzero(clipped))
            component_count += values.size
            values = np.clip(np.rint(values), -32768, 32767).astype(np.int16)
            stream.write(values.tobytes(order="C"))

    os.replace(temporary, output_path)
    del raw

    tone_power = float(abs(coefficient) ** 2)
    residual_coefficient = residual_projection / sample_count
    power_suppression_db = (
        10.0 * np.log10(power_before / power_after)
        if power_after > 0 else float("inf"))
    metadata = {
        "input": str(input_path),
        "output": str(output_path),
        "sample_format": "sc16",
        "sample_count": sample_count,
        "sample_rate_hz": fs,
        "tone_baseband_hz": tone_hz,
        "tone_amplitude": float(abs(coefficient)),
        "tone_power": tone_power,
        "input_power": float(power_before / sample_count),
        "output_power": float(power_after / sample_count),
        "power_suppression_db": float(power_suppression_db),
        "residual_tone_amplitude": float(abs(residual_coefficient)),
        "tone_suppression_db": float(
            20.0 * np.log10((abs(coefficient) + 1e-30) /
                             (abs(residual_coefficient) + 1e-30))),
        "clipped_component_fraction": float(
            clipped_components / component_count),
        "sha256": sha256_file(output_path),
    }
    metadata_path = output_path.with_name(output_path.stem + "_metadata.json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n",
                             encoding="utf-8")
    print(json.dumps(metadata, sort_keys=True))
    return metadata


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--fs", type=float, default=FS_DEFAULT)
    ap.add_argument("--center-hz", type=float, default=CENTER_DEFAULT)
    ap.add_argument("--tone-rf-hz", type=float, default=TONE_RF_DEFAULT)
    ap.add_argument("--tone-hz", type=float, default=None,
                    help="baseband tone frequency; overrides --tone-rf-hz")
    ap.add_argument("--chunk-complex", type=int, default=4_000_000)
    return ap


def main() -> int:
    args = parser().parse_args()
    if not args.input.is_file():
        raise SystemExit(f"missing input: {args.input}")
    if args.chunk_complex <= 0:
        raise SystemExit("--chunk-complex must be positive")
    tone_hz = (args.tone_hz if args.tone_hz is not None
               else args.tone_rf_hz - args.center_hz)
    remove_tone(args.input, args.output, args.fs, tone_hz, args.chunk_complex)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
