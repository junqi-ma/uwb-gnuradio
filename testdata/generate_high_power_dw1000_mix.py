#!/usr/bin/env python3
"""Add a real DW1000 SC16 capture to a clean QM35 SC16 capture.

Both inputs must be raw little-endian interleaved int16 I/Q with identical
sample rate, length, center frequency, and capture alignment.  The output is
also raw SC16.  ``gain`` scales the DW1000 capture amplitude before addition;
the default levels are gain1=0 dB, gain2=-6 dB, and gain3=-12 dB for the
interferer.

Example:
  python3 testdata/generate_high_power_dw1000_mix.py \
    --clean /mnt/f/UWB.../qm35_clean.dat \
    --interference /mnt/f/UWB.../dw1000_interference.dat \
    --output-dir /mnt/f/UWB.../qm35_high_power_mix
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np


DEFAULT_GAINS = (1.0, 10.0 ** (-6.0 / 20.0), 10.0 ** (-12.0 / 20.0))


def sha256_file(path: Path, chunk_bytes: int = 16 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(chunk_bytes)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def generate_one(clean: Path,
                 interference: Path,
                 output: Path,
                 gain: float,
                 chunk_complex: int,
                 level_name: str,
                 reference_gain: float) -> dict:
    clean_size = clean.stat().st_size
    interference_size = interference.stat().st_size
    if clean_size != interference_size:
        raise ValueError(
            f"input size mismatch: clean={clean_size} interference={interference_size}")
    if clean_size % 4:
        raise ValueError(f"input is not an integral SC16 stream: {clean}")

    clean_mm = np.memmap(clean, dtype=np.int16, mode="r")
    interference_mm = np.memmap(interference, dtype=np.int16, mode="r")
    clipped_components = 0
    component_count = 0
    sum_power = 0.0
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.name + ".tmp")
    if temporary.exists():
        temporary.unlink()

    with temporary.open("wb") as stream:
        for lo in range(0, clean_mm.size, chunk_complex * 2):
            hi = min(lo + chunk_complex * 2, clean_mm.size)
            clean_chunk = np.asarray(clean_mm[lo:hi], dtype=np.float32)
            interference_chunk = np.asarray(interference_mm[lo:hi],
                                             dtype=np.float32)
            mixed = clean_chunk + float(gain) * interference_chunk
            clipped = (mixed > 32767.0) | (mixed < -32768.0)
            clipped_components += int(np.count_nonzero(clipped))
            component_count += mixed.size
            sum_power += float(np.square(mixed, dtype=np.float64).sum())
            mixed = np.clip(np.rint(mixed), -32768, 32767).astype(np.int16)
            stream.write(mixed.tobytes(order="C"))

    os.replace(temporary, output)
    metadata = {
        "level": level_name,
        "clean_input": str(clean),
        "interference_input": str(interference),
        "output": str(output),
        "gain_amplitude": float(gain),
        "gain_power_db": float(20.0 * np.log10(abs(gain)))
        if gain != 0 else float("-inf"),
        "relative_to_gain1_power_db": float(
            20.0 * np.log10(abs(gain / reference_gain)))
        if gain != 0 and reference_gain != 0 else float("-inf"),
        "sample_format": "sc16",
        "complex_sample_count": clean_size // 4,
        "file_bytes": output.stat().st_size,
        "combined_component_rms_before_clip": float(
            np.sqrt(sum_power / component_count)),
        "clipped_component_count_before_clip": clipped_components,
        "clipped_component_fraction_before_clip": float(
            clipped_components / component_count),
        "sha256": sha256_file(output),
    }
    metadata_path = output.with_name(output.stem + "_metadata.json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n",
                             encoding="utf-8")
    print(json.dumps(metadata, sort_keys=True))
    return metadata


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", required=True, type=Path)
    ap.add_argument("--interference", required=True, type=Path)
    ap.add_argument("--output-dir", required=True, type=Path)
    ap.add_argument(
        "--gains", nargs="+", type=float, default=None,
        help="DW1000 amplitude gains; defaults to 0, -6, -12 dB power levels")
    ap.add_argument("--chunk-complex", type=int, default=4_000_000)
    return ap


def main() -> int:
    args = parser().parse_args()
    for path in (args.clean, args.interference):
        if not path.is_file():
            raise SystemExit(f"missing input: {path}")
    if args.chunk_complex <= 0:
        raise SystemExit("--chunk-complex must be positive")

    gains = tuple(DEFAULT_GAINS if args.gains is None else args.gains)
    if not gains or gains[0] <= 0:
        raise SystemExit("the first gain must be positive and is the gain1 reference")

    all_metadata = []
    for level_index, gain in enumerate(gains, start=1):
        level_name = f"gain{level_index}"
        output = args.output_dir / f"qm35_clean_plus_dw1000_{level_name}.dat"
        all_metadata.append(generate_one(
            args.clean, args.interference, output, gain, args.chunk_complex,
            level_name, gains[0]))
    manifest = args.output_dir / "manifest.json"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(all_metadata, indent=2) + "\n",
                        encoding="utf-8")
    print(f"wrote {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
