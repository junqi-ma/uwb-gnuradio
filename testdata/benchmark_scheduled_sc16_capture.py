#!/usr/bin/env python3
"""Measure fixed-schedule SC16 extraction on real captures.

This is intentionally a scheduled-window test, not a global energy detector:
the input t0 is known and the extractor must emit one EverySlot window per
QM35 period regardless of DW1000 energy inside or outside the window.

Example:
  PYTHONPATH=$PWD/gr-uwb/build/test_modules \
  LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \
  python3 testdata/benchmark_scheduled_sc16_capture.py \
    --inputs /mnt/f/UWB.../qm35_clean_plus_dw1000_gain1.dat \
             /mnt/f/UWB.../qm35_clean_plus_dw1000_gain2.dat \
    --t0 3543362 \
    --results /mnt/f/UWB.../scheduled_detection_results.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np
from gnuradio import blocks, gr, uwb
import pmt


FS_NATIVE = 737.28e6
PERIOD_S = 0.005
PRE = 7373
CAPTURE = 140083
POST = 3023
POOL = 16


def dict_i64(meta, key: str, default: int = -1) -> int:
    value = pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)
    if pmt.is_uint64(value):
        return int(pmt.to_uint64(value))
    if pmt.is_integer(value):
        return int(pmt.to_long(value))
    return default


def expected_windows(n_samples: int, t0: int, period_samples: int) -> tuple[int, int]:
    complete = 0
    attempted = 0
    while True:
        predicted = t0 + complete * period_samples
        if predicted - PRE >= n_samples:
            break
        attempted += 1
        if predicted - PRE >= 0 and predicted + CAPTURE + POST <= n_samples:
            complete += 1
        else:
            break
    return complete, attempted


def run_one(path: Path, t0: int, max_items: int, verify: bool) -> dict:
    n_samples = path.stat().st_size // 4
    period_samples = int(round(PERIOD_S * FS_NATIVE))
    expected_complete, expected_attempted = expected_windows(
        n_samples, t0, period_samples)

    source = blocks.file_source(2 * gr.sizeof_short, str(path), False)
    extractor = uwb.scheduled_extractor_sc16(
        FS_NATIVE, PERIOD_S, t0, PRE, CAPTURE, POST, POOL)
    debug = blocks.message_debug()
    top = gr.top_block("benchmark_scheduled_sc16_capture")
    top.connect(source, extractor)
    top.msg_connect(extractor, "packet", debug, "store")
    source.set_max_noutput_items(max_items)
    extractor.set_max_noutput_items(max_items)
    top.set_max_noutput_items(max_items)

    started = time.perf_counter()
    top.run()
    wall_s = time.perf_counter() - started

    source_mm = np.memmap(path, dtype=np.int16, mode="r")
    exact = 0
    bad_geometry = 0
    grid_errors = 0
    starts = []
    rms = []
    for index in range(debug.num_messages()):
        msg = debug.get_message(index)
        if not pmt.is_pair(msg):
            bad_geometry += 1
            continue
        meta = pmt.car(msg)
        data = pmt.cdr(msg)
        start = dict_i64(meta, "window_start_sample")
        predicted = dict_i64(meta, "predicted_start_sample")
        count = dict_i64(meta, "sample_count", 0)
        schedule_index = dict_i64(meta, "schedule_index", -1)
        starts.append(start)
        if start < 0 or count <= 0 or start + count > n_samples:
            bad_geometry += 1
            continue
        if predicted != t0 + schedule_index * period_samples:
            grid_errors += 1
        if not pmt.is_s16vector(data):
            bad_geometry += 1
            continue
        actual = np.asarray(pmt.s16vector_elements(data), dtype=np.int16)
        expected = np.asarray(source_mm[start * 2:(start + count) * 2],
                              dtype=np.int16)
        if actual.shape != expected.shape or not np.array_equal(actual, expected):
            bad_geometry += 1
        else:
            exact += 1
        if actual.size:
            pairs = actual.astype(np.float64).reshape(-1, 2)
            rms.append(float(np.sqrt(np.mean(np.sum(pairs * pairs, axis=1)))))

    result = {
        "input": str(path),
        "complex_samples": int(n_samples),
        "t0_native": int(t0),
        "period_native_samples": int(period_samples),
        "window_geometry": {"pre": PRE, "capture": CAPTURE, "post": POST},
        "expected_complete_windows": int(expected_complete),
        "expected_attempted_windows": int(expected_attempted),
        "scheduled_windows": int(extractor.scheduled_windows()),
        "completed_windows": int(extractor.completed_windows()),
        "emitted_windows": int(extractor.emitted_windows()),
        "dropped_windows": int(extractor.dropped_windows()),
        "message_windows": int(debug.num_messages()),
        "bit_exact_windows": int(exact),
        "bad_geometry_or_iq_windows": int(bad_geometry),
        "grid_errors": int(grid_errors),
        "window_count_error": int(debug.num_messages() - expected_complete),
        "wall_seconds": wall_s,
        "wall_input_msps": n_samples / wall_s / 1e6,
        "process_total_us": int(extractor.process_total_us()),
        "copy_total_us": int(extractor.copy_total_us()),
        "publish_total_us": int(extractor.publish_total_us()),
        "process_input_msps": (
            n_samples / extractor.process_total_us()
            if extractor.process_total_us() else math.inf),
        "window_rms_mean": float(np.mean(rms)) if rms else math.nan,
        "window_rms_p95": float(np.percentile(rms, 95)) if rms else math.nan,
        "first_window_start": starts[0] if starts else None,
        "last_window_start": starts[-1] if starts else None,
        "pass": (
            extractor.emitted_windows() == expected_complete
            and extractor.dropped_windows() == expected_attempted - expected_complete
            and exact == expected_complete
            and bad_geometry == 0
            and grid_errors == 0
        ),
    }
    print(json.dumps(result, sort_keys=True))
    return result


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--inputs", nargs="+", required=True, type=Path)
    ap.add_argument("--t0", type=int, default=3543362)
    ap.add_argument("--max-items", type=int, default=1 << 20)
    ap.add_argument("--no-verify", action="store_true")
    ap.add_argument("--results", type=Path, default=None)
    return ap


def main() -> int:
    args = parser().parse_args()
    if args.max_items <= 0:
        raise SystemExit("--max-items must be positive")
    for path in args.inputs:
        if not path.is_file():
            raise SystemExit(f"missing input: {path}")
    results = [run_one(path, args.t0, args.max_items, not args.no_verify)
               for path in args.inputs]
    if args.results:
        args.results.parent.mkdir(parents=True, exist_ok=True)
        args.results.write_text(json.dumps(results, indent=2) + "\n",
                                encoding="utf-8")
        print(f"wrote {args.results}")
    return 0 if all(item["pass"] for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
