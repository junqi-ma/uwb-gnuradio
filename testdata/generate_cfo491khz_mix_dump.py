#!/usr/bin/env python3
"""Mix tone-removed QM35 + DW1000 with a 491.370 kHz relative CFO and write
a scheduled SC16 dump (capture.iq + capture.jsonl).

491.370 kHz = 998.4e6 / (2 * 1016) is exactly π radians per HRP SYNC, i.e.
the per-SYNC phase Nyquist.  QM35 is left unchanged so dump geometry from
the clean capture still matches.  DW1000 is rotated in the native 737.28
MS/s domain.

Default rotation also cancels the measured natural relative CFO
(DW ≈ −1751 Hz, QM35 ≈ +1848 Hz) so the mix relative is the half-SYNC
frequency rather than 491 kHz plus that leftover 3.6 kHz.

Example:

  python3 testdata/generate_cfo491khz_mix_dump.py
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np


FS737 = 737.28e6
FS998 = 998.4e6
SYNC_LEN_998 = 1016
# π per SYNC at the 998.4 MS/s work rate (and at 737.28 after 48/65).
F_HALF_SYNC_HZ = FS998 / (2.0 * SYNC_LEN_998)

# MATLAB medians on the 2026-08-11 separate captures (FCS-pass).
CFO_QM35_HZ = 1847.82
CFO_DW1000_HZ = -1750.65

DEFAULT_CLEAN = Path(
    "/mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817/"
    "tone_removed_inputs/qm35_clean_tone_removed.dat")
DEFAULT_DW = Path(
    "/mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817/"
    "tone_removed_inputs/dw1000_clean_tone_removed.dat")
DEFAULT_GEOMETRY = Path(
    "/mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump/capture.jsonl")
DEFAULT_OUT = Path(
    "/mnt/f/UWB基带数据/qm35_dw1000_cfo491kHz_scheduled_sc16_dump")


def load_jsonl(path: Path) -> list[dict]:
    metas = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                metas.append(json.loads(line))
    return metas


def rotation_hz(compensate_natural: bool) -> tuple[float, float]:
    """Return (dw_rotation_hz, expected_relative_hz = CFO_DW' - CFO_QM)."""
    natural_rel = CFO_DW1000_HZ - CFO_QM35_HZ
    if compensate_natural:
        dw_rot = F_HALF_SYNC_HZ - natural_rel
    else:
        dw_rot = F_HALF_SYNC_HZ
    expected_rel = natural_rel + dw_rot
    return dw_rot, expected_rel


def mix_window(qm: np.ndarray,
               dw: np.ndarray,
               start: int,
               dw_rot_hz: float,
               gain: float) -> tuple[np.ndarray, int]:
    """qm/dw are interleaved int16. Returns mixed int16 and clip count."""
    n = qm.size // 2
    q = qm.astype(np.float32).reshape(-1, 2)
    d = dw.astype(np.float32).reshape(-1, 2)
    dc = d[:, 0] + 1j * d[:, 1]
    idx = np.arange(start, start + n, dtype=np.float64)
    phase = (2.0 * np.pi * dw_rot_hz / FS737) * idx
    rot = np.exp(1j * phase)
    dc *= rot
    mixed = np.empty_like(q)
    mixed[:, 0] = q[:, 0] + float(gain) * dc.real
    mixed[:, 1] = q[:, 1] + float(gain) * dc.imag
    clipped = int(np.count_nonzero((mixed > 32767.0) | (mixed < -32768.0)))
    out = np.clip(np.rint(mixed), -32768, 32767).astype(np.int16).reshape(-1)
    return out, clipped


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", type=Path, default=DEFAULT_CLEAN)
    ap.add_argument("--interference", type=Path, default=DEFAULT_DW)
    ap.add_argument("--geometry-json", type=Path, default=DEFAULT_GEOMETRY)
    ap.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--gain", type=float, default=1.0,
                    help="DW1000 amplitude gain (default 1.0 = 0 dB)")
    ap.add_argument("--keep-natural-cfo", action="store_true",
                    help="add exactly 491.370 kHz on top of the natural "
                         "relative CFO instead of cancelling it")
    return ap


def main() -> int:
    args = parser().parse_args()
    for path in (args.clean, args.interference, args.geometry_json):
        if not path.is_file():
            raise SystemExit(f"missing: {path}")
    if args.gain == 0:
        raise SystemExit("--gain must be non-zero")

    compensate = not args.keep_natural_cfo
    dw_rot_hz, expected_rel = rotation_hz(compensate)
    metas = load_jsonl(args.geometry_json)
    if not metas:
        raise SystemExit("empty geometry jsonl")

    out_dir = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    iq_path = out_dir / "capture.iq"
    jsonl_path = out_dir / "capture.jsonl"
    meta_path = out_dir / "mix_metadata.json"
    tmp_iq = iq_path.with_name(iq_path.name + ".tmp")
    if tmp_iq.exists():
        tmp_iq.unlink()

    qm_mm = np.memmap(args.clean, dtype=np.int16, mode="r")
    dw_mm = np.memmap(args.interference, dtype=np.int16, mode="r")
    if qm_mm.size != dw_mm.size:
        raise SystemExit(
            f"size mismatch clean={qm_mm.size} dw={dw_mm.size}")

    offset = 0
    clipped = 0
    written_windows = 0
    print(
        f"relative target={expected_rel:.6f} Hz  "
        f"DW rotation={dw_rot_hz:.6f} Hz  "
        f"compensate_natural={compensate}  gain={args.gain}",
        flush=True)

    with tmp_iq.open("wb") as iq_f, jsonl_path.open("w") as jf:
        for m in metas:
            n = int(m.get("sample_count", 0))
            wstart = int(m.get("window_start_sample",
                               m.get("start_sample", -1)))
            if n <= 0 or wstart < 0:
                continue
            lo = wstart * 2
            hi = (wstart + n) * 2
            if hi > qm_mm.size:
                print(f"skip window start={wstart} n={n} past EOF", flush=True)
                continue
            qm = np.asarray(qm_mm[lo:hi])
            dw = np.asarray(dw_mm[lo:hi])
            mixed, nclip = mix_window(qm, dw, wstart, dw_rot_hz, args.gain)
            clipped += nclip
            iq_f.write(mixed.tobytes(order="C"))
            m2 = dict(m)
            m2["file_offset_samples"] = offset
            m2["sample_rate"] = int(FS737)
            m2["sample_format"] = "sc16"
            jf.write(json.dumps(m2) + "\n")
            offset += n
            written_windows += 1
            if written_windows % 10 == 0 or written_windows == len(metas):
                print(f"  windows {written_windows}/{len(metas)}", flush=True)

    os.replace(tmp_iq, iq_path)
    info = {
        "clean_input": str(args.clean),
        "interference_input": str(args.interference),
        "geometry_jsonl": str(args.geometry_json),
        "output_dir": str(out_dir),
        "gain_amplitude": float(args.gain),
        "gain_power_db": float(20.0 * np.log10(abs(args.gain))),
        "fs_native_hz": FS737,
        "fs_work_hz": FS998,
        "half_sync_cfo_hz": F_HALF_SYNC_HZ,
        "half_sync_phase_rad_per_sync": float(np.pi),
        "compensate_natural_cfo": compensate,
        "measured_cfo_qm35_hz": CFO_QM35_HZ,
        "measured_cfo_dw1000_hz": CFO_DW1000_HZ,
        "measured_natural_relative_hz": CFO_DW1000_HZ - CFO_QM35_HZ,
        "dw1000_rotation_hz": dw_rot_hz,
        "expected_relative_cfo_hz": expected_rel,
        "relative_definition": "CFO_DW - CFO_QM35 after mix",
        "windows_written": written_windows,
        "complex_samples": offset,
        "file_bytes": iq_path.stat().st_size,
        "clipped_component_count": clipped,
        "sample_format": "sc16",
        "note": (
            "QM35 unmodified. DW1000 rotated at native 737.28 MS/s using "
            "absolute capture sample index, then added. Dump windows copied "
            "from the clean QM35 scheduled geometry."
        ),
    }
    meta_path.write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(info, indent=2), flush=True)
    print(f"wrote {iq_path}", flush=True)
    print(f"wrote {jsonl_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
