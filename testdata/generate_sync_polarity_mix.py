#!/usr/bin/env python3
"""Generate SYNC polarity-coded mixes for QM35/DW1000 validation.

This script implements the TX/RX polarity-coding experiment described in
``UWB模拟域SYNC极性编码干扰抑制分析.md`` §1--§2 for offline validation:

  q_c(t) = c(t) * q(t)                 # TX code on QM35 only, per SYNC
  m(t)   = q_c(t) + g * d(t)           # ADC-domain mix (raw)
  r(t)   = c(t) * m(t)                 # RX decode (MATLAB sees this)
       = q(t) + g * c(t) * d(t)       # QM35 restored, DW1000 multiplied

c(t) is an alternating code ``(+1,-1,+1,-1,...)`` defined per QM35 SYNC
symbol.  Outside the 64-SYNC preamble of each QM35 slot, c(t)=+1.

Inputs must be tone-removed SC16 captures with identical length and rate
(737.28 MS/s).  The default inputs are the notched files so results are
directly comparable to ``qm35_high_power_dw1000_mix_notched_20260817``.

Outputs (per gain) are two raw SC16 files plus metadata/manifest, and an
optional scheduled-dump directory that follows the ``capture.iq +
capture.jsonl`` contract from ``docs/phase1/下一步_锁定后SC16截取与DW1000头尾预留.md``.
The dump is sliced from the RX-decoded file with the same window geometry
as the clean capture so ``testdata/decode_scheduled_sc16_dump.py`` can be
run unchanged.

Example:

  python3 testdata/generate_sync_polarity_mix.py \\
    --clean /mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817/tone_removed_inputs/qm35_clean_tone_removed.dat \\
    --interference /mnt/f/UWB基带数据/qm35_high_power_dw1000_mix_notched_20260817/tone_removed_inputs/dw1000_clean_tone_removed.dat \\
    --geometry-json /mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump/capture.jsonl \\
    --csv /mnt/f/UWB基带数据/qm35_clean_scheduled_sc16_dump/scheduled_dump_cpp.csv \\
    --output-dir /mnt/f/UWB基带数据/qm35_sync_polarity_notched_test \\
    --gains 1.0 0.501187 0.251188

For a quick smoke test on the first 0.08 s (59_... samples), add
``--max-complex 60000000`` and inspect the JSON logs.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

DEFAULT_GAINS = (1.0, 10.0 ** (-6.0 / 20.0), 10.0 ** (-12.0 / 20.0))
FS737 = 737.28e6
FS998 = 998.4e6
SYNC_REPETITIONS_QM35 = 64
# One HRP SYNC at 998.4 MS/s is 1016 samples (see UWB_test_signal_description.md).
SYNC_LEN_998 = 1016
SYNC_PERIOD_NATIVE = SYNC_LEN_998 * 48.0 / 65.0  # 750.276923... samples @737.28
FILTER_DELAY_QMIN = (2707 - 1) / 2  # quality_minorder taps, see resampler_65_48


def sha256_file(path: Path, chunk_bytes: int = 16 * 1024 * 1024) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            b = f.read(chunk_bytes)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def load_geometry(jsonl_path: Path):
    """Load scheduled window geometry from capture.jsonl.

    Returns list of dicts with keys: window_start_sample,
    predicted_start_sample, pre_guard_samples, capture_samples,
    post_guard_samples, sample_count, packet_id, schedule_index,
    capture_mode.
    """
    metas = []
    with jsonl_path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                metas.append(json.loads(line))
    return metas


def load_csv_anchors(csv_path: Path) -> tuple[dict[int, int], dict[int, int]]:
    """Map anchors from dump CSV.

    Returns (by_schedule_index, by_packet_id) where values are
    det_minus_pred in 998.4 samples.  Acquisition (schedule_index=-1)
    is kept in by_packet_id[0].
    """
    by_sched: dict[int, int] = {}
    by_pid: dict[int, int] = {}
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # det_minus
            v = row.get("qm35_det_minus_pred", "")
            det_minus = None
            if v not in ("", None):
                try:
                    det_minus = int(float(v))
                except Exception:
                    det_minus = None
            if det_minus is None:
                det = row.get("qm35_detected_start", "")
                pred = row.get("qm35_predicted_start_out", "")
                if det not in ("", None) and pred not in ("", None):
                    try:
                        det_minus = int(int(det) - int(pred))
                    except Exception:
                        det_minus = None
            if det_minus is None:
                continue
            try:
                pid = int(row.get("packet_id", -1))
                by_pid[pid] = det_minus
            except Exception:
                pass
            try:
                k = int(row["schedule_index"])
                if k >= 0:
                    by_sched[k] = det_minus
                else:
                    # keep acquisition det_minus under packet_id already
                    pass
            except Exception:
                continue
    return by_sched, by_pid


def build_intervals(
    metas,
    csv_anchors: tuple[dict[int, int], dict[int, int]] | None,
    sync_repetitions: int = SYNC_REPETITIONS_QM35,
) -> list[tuple[int, int, int]]:
    """Build list of (start, end, code) per SYNC symbol across all slots.

    Each scheduled/provisional/acquisition window contributes sync_repetitions
    intervals.  The anchor (first SYNC start) is derived as:

      anchor = predicted_start + round(det_minus_pred * 48/65)

    If csv_anchors is missing for a given schedule_index/packet_id, falls
    back to predicted_start (no offset).

    Returns sorted list of (start, end_exclusive, c) where c in {+1,-1}.
    """
    by_sched, by_pid = (csv_anchors if csv_anchors is not None else (None, None))
    intervals: list[tuple[int, int, int]] = []
    for m in metas:
        mode = m.get("capture_mode", "")
        if mode not in ("scheduled", "provisional", "acquisition"):
            continue
        pred = int(m.get("predicted_start_sample", -1))
        if pred < 0:
            # acquisition fallback: window_start + pre_guard
            try:
                pred = int(m.get("window_start_sample", -1)) + int(m.get("pre_guard_samples", m.get("pre_trigger_samples", 0)))
            except Exception:
                continue
            if pred < 0:
                continue
        det_minus = None
        # try schedule_index first
        try:
            k = int(m.get("schedule_index", -9999))
            if by_sched is not None and k in by_sched:
                det_minus = by_sched[k]
        except Exception:
            pass
        # then packet_id
        if det_minus is None:
            try:
                pid = int(m.get("packet_id", -9999))
                if by_pid is not None and pid in by_pid:
                    det_minus = by_pid[pid]
            except Exception:
                pass
        if det_minus is not None:
            anchor = pred + int(round(det_minus * 48.0 / 65.0))
        else:
            anchor = pred
        # Alternate code per SYNC: c_k = (-1)^k
        for s in range(sync_repetitions):
            b0 = int(round(anchor + s * SYNC_PERIOD_NATIVE))
            b1 = int(round(anchor + (s + 1) * SYNC_PERIOD_NATIVE))
            if b1 <= b0:
                b1 = b0 + 1
            c = 1 if (s % 2 == 0) else -1
            intervals.append((b0, b1, c))
    intervals.sort(key=lambda x: x[0])
    # Merge check: ensure no zero-length, and sort.
    return intervals


def apply_intervals_chunk(
    chunk: np.ndarray,
    chunk_start_complex: int,
    intervals: list[tuple[int, int, int]],
    interval_idx_start: int = 0,
) -> tuple[np.ndarray, int]:
    """Multiply chunk (float32 interleaved view) by code in overlapping intervals.

    chunk is float32 array of size chunk_complex*2 (I0,Q0,I1,Q1,...).
    Returns (modified chunk, next_interval_idx) where next_interval_idx is
    the first interval that may still overlap the next chunk.
    """
    # Use view of complex? Easier to process as int16 interleaved: multiply
    # I and Q components by c.
    # chunk is float32 already; just multiply both I and Q per sample.
    # Find first interval with end > chunk_start
    n_complex = chunk.size // 2
    chunk_end = chunk_start_complex + n_complex
    # Advance pointer
    idx = interval_idx_start
    while idx < len(intervals) and intervals[idx][1] <= chunk_start_complex:
        idx += 1
    # Iterate overlapping intervals
    j = idx
    while j < len(intervals) and intervals[j][0] < chunk_end:
        s, e, c = intervals[j]
        if c == -1:
            # overlap [max(s, cs), min(e, ce))
            lo = max(s, chunk_start_complex)
            hi = min(e, chunk_end)
            if hi > lo:
                # indices in chunk: (lo-cs)*2 .. (hi-cs)*2
                a = (lo - chunk_start_complex) * 2
                b = (hi - chunk_start_complex) * 2
                # multiply
                chunk[a:b] *= -1.0
        j += 1
    return chunk, idx


def generate_mixes(
    clean: Path,
    interference: Path,
    output_dir: Path,
    gains: tuple[float, ...],
    intervals: list[tuple[int, int, int]],
    chunk_complex: int,
    sync_repetitions: int,
    code_pattern: str,
    geometry_path: Path | None,
    csv_path: Path | None,
    max_complex: int | None = None,
) -> list[dict]:
    clean_size = clean.stat().st_size
    inter_size = interference.stat().st_size
    if clean_size != inter_size:
        raise ValueError(f"input size mismatch: clean={clean_size} inter={inter_size}")
    if clean_size % 4:
        raise ValueError(f"not integral SC16: {clean}")
    n_complex_total = clean_size // 4
    if max_complex is not None and max_complex > 0:
        n_complex_total = min(n_complex_total, max_complex)
        clean_size = n_complex_total * 4
    clean_mm = np.memmap(clean, dtype=np.int16, mode="r")
    inter_mm = np.memmap(interference, dtype=np.int16, mode="r")

    all_meta = []
    ref_gain = gains[0] if gains else 1.0

    # Precompute chunk boundaries for intervals: sort already.
    # For each gain we will do two passes: raw_adc and decoded.
    # To avoid doubling IO, compute raw then decoded = raw * c inside intervals.

    for level_index, gain in enumerate(gains, start=1):
        level_name = f"gain{level_index}"
        # File names follow the notched convention but with polarity suffix.
        raw_path = output_dir / f"qm35_sync_polarity_gain{level_index}_raw.dat"
        decoded_path = output_dir / f"qm35_sync_polarity_gain{level_index}_decoded.dat"
        # Purge old temps
        for p in (raw_path, decoded_path):
            tmp = p.with_name(p.name + ".tmp")
            if tmp.exists():
                tmp.unlink()

        clipped_raw = 0
        clipped_dec = 0
        sum_power_raw = 0.0
        sum_power_dec = 0.0
        component_count = 0

        raw_tmp = raw_path.with_name(raw_path.name + ".tmp")
        dec_tmp = decoded_path.with_name(decoded_path.name + ".tmp")
        raw_tmp.parent.mkdir(parents=True, exist_ok=True)

        # Need interval pointer per file pass but intervals are same per chunk
        # We'll stream chunk by chunk.
        raw_f = raw_tmp.open("wb")
        dec_f = dec_tmp.open("wb")
        try:
            interval_ptr = 0
            # For decoded, we need also raw chunk to derive decoded via c multiply,
            # but we can compute decoded directly as q + g*c*d . Equivalent to
            # (c*q + g*d)*c = q + g*c*d . Instead of recomputing, we can apply
            # second multiply to raw. Simpler: compute mixed_raw then mixed_dec = mixed_raw * c.
            # So compute mixed_raw per chunk, write it, then copy and multiply for dec.
            # But to avoid holding both, generate both in one pass:
            for lo_complex in range(0, n_complex_total, chunk_complex):
                hi_complex = min(lo_complex + chunk_complex, n_complex_total)
                n = hi_complex - lo_complex
                lo_bytes = lo_complex * 2
                hi_bytes = hi_complex * 2
                clean_chunk = np.asarray(clean_mm[lo_bytes:hi_bytes], dtype=np.float32)
                inter_chunk = np.asarray(inter_mm[lo_bytes:hi_bytes], dtype=np.float32)

                # Build coded_q: copy clean and multiply per interval
                coded_q = clean_chunk.copy()
                # apply code to coded_q (inside intervals multiply by c, alternating)
                # Need to handle interval pointer progression; we reuse apply_intervals_chunk
                # but coded_q is the q part only; outside intervals c=1 so unchanged.
                # Use same pointer logic but for coded_q.
                # We maintain a per-chunk pointer that advances monotonically.
                coded_q, next_ptr = apply_intervals_chunk(coded_q, lo_complex, intervals, interval_ptr)
                # For next chunk, interval_ptr moves to next_ptr (but apply_intervals_chunk already
                # advanced for the chunk; however we called it for coded_q, its pointer is for decoded multiply
                # For coded_q we advanced to next_ptr; that is also correct start for next lo.
                # For decoded we will need same logical pointer but after raw generation.
                # So remember pointer for decoded: same as after coded_q multiply.
                # However apply_intervals_chunk mutates to find overlapping intervals per chunk from pointer.
                # So we should have separate pointer per gain pass? Since intervals sorted, pointer monotonically increases.
                # coded_q pointer after this chunk is next_ptr; next iteration should start from next_ptr.
                interval_ptr = next_ptr

                # raw_adc = coded_q + g*d
                mixed_raw = coded_q + float(gain) * inter_chunk
                # stats raw
                clipped_raw += int(np.count_nonzero((mixed_raw > 32767.0) | (mixed_raw < -32768.0)))
                component_count_chunk = mixed_raw.size
                sum_power_raw += float(np.square(mixed_raw, dtype=np.float64).sum())
                # decoded = q + g*c*d  = mixed_raw * c  (c inside intervals)
                # To compute decoded without re-reading, multiply mixed_raw by c inside intervals
                mixed_dec = mixed_raw.copy()
                # Apply c multiply to mixed_dec (second pass)
                # Need to re-apply with same intervals, but now starting from lo_complex
                # We already have interval indices: overlapping intervals for this chunk are those with
                # s < hi and e > lo. Multiplying mixed_dec by c again for c=-1 intervals gives equivalent.
                # Since mixed_raw already has coded_q inside, multiplying whole mixed_raw by c yields:
                # inside c=-1 intervals: (-q + g*d)*(-1) = q - g*d .. wait not same as q + g*c*d?
                # Let's verify: q_c = c*q. raw = c*q + g*d. raw*c = c*c*q + g*c*d = q + g*c*d  (correct)
                # So raw*c is correct. We can get decoded by multiplying raw by c in c=-1 intervals.
                # So reuse interval logic on mixed_dec (same intervals, c=-1)
                # Need to re-derive overlapping pointer: we already have it but need to apply.
                # Instead of recomputing pointer search, just iterate intervals overlapping this chunk and flip.
                # We have interval_ptr_start for this chunk before advance; but we mutated interval_ptr.
                # So re-iterate from (interval_ptr before advance) would be expensive.
                # Simpler: directly compute overlap loop again using binary search or linear scan per chunk for decoded.
                # For correctness and simplicity with small chunk count, just loop over intervals.
                # Let's do explicit per-chunk flip for decoded:
                # Use intervals list and check overlap quickly via linear scan from current pointer-allow lookback?
                # Since intervals sorted and chunk size large (4M complex ~ 2k intervals per chunk?), linear scan per chunk is fine: 99 windows *64 = 6336 intervals total. So at most 6336 intervals total, scanning all per chunk (92 chunks) => 580k checks, trivial.
                # So just brute force per chunk.
                for (s, e, c) in intervals:
                    if e <= lo_complex or s >= hi_complex:
                        continue
                    if c == -1:
                        lo_o = max(s, lo_complex)
                        hi_o = min(e, hi_complex)
                        a = (lo_o - lo_complex) * 2
                        b = (hi_o - lo_complex) * 2
                        mixed_dec[a:b] *= -1.0
                # stats decoded
                clipped_dec += int(np.count_nonzero((mixed_dec > 32767.0) | (mixed_dec < -32768.0)))
                sum_power_dec += float(np.square(mixed_dec, dtype=np.float64).sum())

                # clip and write
                mixed_raw = np.clip(np.rint(mixed_raw), -32768, 32767).astype(np.int16)
                mixed_dec = np.clip(np.rint(mixed_dec), -32768, 32767).astype(np.int16)
                raw_f.write(mixed_raw.tobytes(order="C"))
                dec_f.write(mixed_dec.tobytes(order="C"))
            # count components: total interleaved int16 components
            component_total = n_complex_total * 2
            # sum_power already summed over components
        finally:
            raw_f.close()
            dec_f.close()

        os.replace(raw_tmp, raw_path)
        os.replace(dec_tmp, decoded_path)

        # metadata generation (mirrors generate_high_power_dw1000_mix.py plus polarity fields)
        def make_meta(path: Path, clipped: int, sum_pow: float, suffix: str) -> dict:
            rms = float(np.sqrt(sum_pow / component_total)) if component_total else 0.0
            frac = float(clipped / component_total) if component_total else 0.0
            return {
                "level": level_name,
                "suffix": suffix,
                "clean_input": str(clean),
                "interference_input": str(interference),
                "output": str(path),
                "gain_amplitude": float(gain),
                "gain_power_db": float(20.0 * np.log10(abs(gain))) if gain != 0 else float("-inf"),
                "relative_to_gain1_power_db": float(20.0 * np.log10(abs(gain / ref_gain))) if gain != 0 and ref_gain != 0 else float("-inf"),
                "sample_format": "sc16",
                "complex_sample_count": n_complex_total,
                "file_bytes": path.stat().st_size,
                "combined_component_rms_before_clip": rms,
                "clipped_component_count_before_clip": clipped,
                "clipped_component_fraction_before_clip": frac,
                "sha256": sha256_file(path),
                "coding": {
                    "pattern": code_pattern,
                    "sync_repetitions": int(sync_repetitions),
                    "sync_period_native_samples": float(SYNC_PERIOD_NATIVE),
                    "sync_period_us": float(SYNC_PERIOD_NATIVE / FS737 * 1e6),
                    "fs_hz": float(FS737),
                    "anchor_source": "csv_det_minus_pred" if csv_path else "predicted_only",
                    "geometry_jsonl": str(geometry_path) if geometry_path else "",
                    "csv_path": str(csv_path) if csv_path else "",
                    "intervals_total": len(intervals),
                    "intervals_applied_complex": int(sum(e - s for s, e, _ in intervals)),
                },
                "max_complex_truncated": bool(max_complex is not None and max_complex > 0 and max_complex < clean.stat().st_size // 4),
            }

        meta_raw = make_meta(raw_path, clipped_raw, sum_power_raw, "raw_adc")
        meta_dec = make_meta(decoded_path, clipped_dec, sum_power_dec, "decoded_rx")

        raw_meta_path = raw_path.with_name(raw_path.stem + "_metadata.json")
        dec_meta_path = decoded_path.with_name(decoded_path.stem + "_metadata.json")
        raw_meta_path.write_text(json.dumps(meta_raw, indent=2) + "\n", encoding="utf-8")
        dec_meta_path.write_text(json.dumps(meta_dec, indent=2) + "\n", encoding="utf-8")

        print(json.dumps({"level": level_name, "gain": gain, "raw": str(raw_path), "decoded": str(decoded_path)}, sort_keys=True))

        all_meta.append({"raw": meta_raw, "decoded": meta_dec})

    # Write manifests
    manifest = output_dir / "manifest.json"
    # Flatten decoded metas for compatibility with previous mix manifests (one per gain)
    flat_decoded = [m["decoded"] for m in all_meta]
    manifest.write_text(json.dumps(flat_decoded, indent=2) + "\n", encoding="utf-8")
    manifest_raw = output_dir / "manifest_raw.json"
    manifest_raw.write_text(json.dumps([m["raw"] for m in all_meta], indent=2) + "\n", encoding="utf-8")
    manifest_full = output_dir / "manifest_full.json"
    manifest_full.write_text(json.dumps(all_meta, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {manifest}")
    print(f"wrote {manifest_raw}")
    print(f"wrote {manifest_full}")
    return all_meta


def write_dump_from_decoded(
    decoded_path: Path,
    output_dir: Path,
    geometry_path: Path,
    chunk_complex: int = 4_000_000,
) -> Path:
    """Slice the RX-decoded whole file into a scheduled-dump directory.

    Uses the same window geometry as the clean dump so MATLAB's
    decode_scheduled_sc16_dump.m can be run unchanged.

    Writes: dump_dir/capture.iq + dump_dir/capture.jsonl
    Returns dump_dir path.
    """
    # Each gain gets its own dump subdir
    # For simplicity, the caller creates per-gain dump by invoking this per decoded file.
    raise NotImplementedError


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Generate SYNC polarity-coded QM35+DW1000 mixes (notched, alternating).")
    ap.add_argument("--clean", required=True, type=Path, help="QM35 clean SC16 (tone-removed recommended)")
    ap.add_argument("--interference", required=True, type=Path, help="DW1000 SC16 (tone-removed recommended)")
    ap.add_argument("--geometry-json", required=True, type=Path, help="capture.jsonl from qm35_clean_scheduled_sc16_dump")
    ap.add_argument("--csv", required=False, type=Path, default=None, help="scheduled_dump_cpp.csv for per-slot det_minus_pred")
    ap.add_argument("--output-dir", required=True, type=Path)
    ap.add_argument("--gains", nargs="+", type=float, default=None, help="DW1000 amplitude gains; defaults to 0, -6, -12 dB")
    ap.add_argument("--sync-repetitions", type=int, default=SYNC_REPETITIONS_QM35, help="QM35 SYNC repetitions per slot (default 64)")
    ap.add_argument("--code", choices=["alternating"], default="alternating", help="Polarity code pattern (first round only alternating)")
    ap.add_argument("--chunk-complex", type=int, default=4_000_000)
    ap.add_argument("--max-complex", type=int, default=0, help="Truncate to first N complex samples (0 = full file, for smoke)")
    ap.add_argument("--write-dumps", action="store_true", help="Also write per-gain scheduled dump dirs (capture.iq+jsonl)")
    return ap


def main() -> int:
    args = parser().parse_args()
    for p in (args.clean, args.interference, args.geometry_json):
        if not p.is_file():
            raise SystemExit(f"missing input: {p}")
    if args.csv is not None and not args.csv.is_file():
        raise SystemExit(f"missing csv: {args.csv}")
    if args.chunk_complex <= 0:
        raise SystemExit("--chunk-complex must be positive")
    gains = tuple(DEFAULT_GAINS if args.gains is None else tuple(args.gains))
    if not gains or gains[0] <= 0:
        raise SystemExit("the first gain must be positive and is the gain1 reference")
    if args.sync_repetitions <= 0:
        raise SystemExit("--sync-repetitions must be positive")

    metas = load_geometry(args.geometry_json)
    csv_anchors = None
    if args.csv is not None:
        csv_anchors = load_csv_anchors(args.csv)
        by_sched, by_pid = csv_anchors
        total_anchors = len(by_sched) + (1 if 0 in by_pid else 0)
        if total_anchors == 0 and len(by_pid) == 0:
            print(f"WARN: no anchors parsed from {args.csv}, falling back to predicted_start", flush=True)
            csv_anchors = None
        else:
            print(f"loaded {len(by_sched)} scheduled + {len(by_pid)} pid anchors from {args.csv}", flush=True)

    intervals = build_intervals(metas, csv_anchors, sync_repetitions=args.sync_repetitions)
    total_coded = sum(e - s for s, e, _ in intervals)
    print(f"built {len(intervals)} SYNC intervals ({args.sync_repetitions} per slot), "
          f"total coded complex samples {total_coded} "
          f"({total_coded / FS737 * 1e6:.1f} us @737.28)", flush=True)
    # Sanity: 64 * 750.2769 ~48017 per slot; 99 slots -> ~4.75M complex samples (~6.4 ms)
    # Check boundary precision: Ts rounding error accumulates? Verify per-slot total.
    if metas:
        # Quick check first slot interval contiguity
        first_slot_intervals = [x for x in intervals if x[0] < metas[1].get("predicted_start_sample", 1 << 62)] if len(metas) > 1 else intervals[:args.sync_repetitions]
        if first_slot_intervals:
            span = first_slot_intervals[-1][1] - first_slot_intervals[0][0]
            expected = round(args.sync_repetitions * SYNC_PERIOD_NATIVE)
            if abs(span - expected) > 2:
                print(f"WARN: first slot span {span} vs expected {expected} (rounding)", flush=True)

    all_meta = generate_mixes(
        clean=args.clean,
        interference=args.interference,
        output_dir=args.output_dir,
        gains=gains,
        intervals=intervals,
        chunk_complex=args.chunk_complex,
        sync_repetitions=args.sync_repetitions,
        code_pattern=args.code,
        geometry_path=args.geometry_json,
        csv_path=args.csv,
        max_complex=args.max_complex if args.max_complex > 0 else None,
    )

    if args.write_dumps:
        # Generate per-gain dump dirs by slicing decoded files with same geometry.
        for level_idx, entry in enumerate(all_meta, start=1):
            decoded_path = Path(entry["decoded"]["output"])
            dump_dir = args.output_dir / f"dump_gain{level_idx}"
            dump_dir.mkdir(parents=True, exist_ok=True)
            # Reuse capture.jsonl geometry verbatim, but recompute file_offset_samples
            dump_jsonl = dump_dir / "capture.jsonl"
            dump_iq = dump_dir / "capture.iq"
            # Copy jsonl with updated file_offset_samples (contiguous dump)
            src_metas = load_geometry(args.geometry_json)
            # Filter to scheduled/provisional only? Keep all as original jsonl did (includes acquisition).
            # For polarity validation we want scheduled windows (code applied there); acquisition window
            # has pre 2032 unrelated to 64-SYNC coding (its SYNC not coded? We coded only scheduled slots).
            # Keep same file_offset_samples recomputed as running sum.
            offset = 0
            decoded_mm = np.memmap(decoded_path, dtype=np.int16, mode="r")
            tmp_iq = dump_iq.with_name(dump_iq.name + ".tmp")
            with tmp_iq.open("wb") as out_f, dump_jsonl.open("w") as jf:
                for m in src_metas:
                    n = int(m.get("sample_count", 0))
                    wstart = int(m.get("window_start_sample", m.get("start_sample", -1)))
                    if n <= 0 or wstart < 0:
                        continue
                    # Skip windows that would exceed decoded file (if truncated)
                    if (wstart + n) * 2 > decoded_mm.size:
                        continue
                    sl = np.array(decoded_mm[wstart * 2:(wstart + n) * 2])
                    out_f.write(sl.tobytes(order="C"))
                    m2 = dict(m)
                    m2["file_offset_samples"] = offset
                    # Ensure sample_rate fields
                    m2["sample_rate"] = int(FS737)
                    m2["sample_format"] = "sc16"
                    jf.write(json.dumps(m2) + "\n")
                    offset += n
            os.replace(tmp_iq, dump_iq)
            print(f"wrote dump {dump_dir}  windows={offset} samples", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
