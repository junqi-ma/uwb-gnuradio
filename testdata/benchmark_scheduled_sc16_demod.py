#!/usr/bin/env python3
"""Run known-t0 scheduled SC16 extraction through the PDU demod path."""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
from gnuradio import blocks, gr, uwb
import pmt


FS_NATIVE = 737.28e6
FS_DEMOD = 998.4e6
PERIOD_S = 0.005
PRE = 7373
CAPTURE = 140083
POST = 3023


def expected_windows(n_samples: int, t0: int) -> tuple[int, int]:
    period_samples = int(round(PERIOD_S * FS_NATIVE))
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


def dict_value(meta, key: str, default):
    value = pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)
    try:
        if isinstance(default, str):
            return pmt.symbol_to_string(value) if pmt.is_symbol(value) else default
        if isinstance(default, bool):
            return bool(pmt.to_bool(value))
        if isinstance(default, int):
            if pmt.is_uint64(value):
                return int(pmt.to_uint64(value))
            return int(pmt.to_long(value))
        return float(pmt.to_double(value))
    except Exception:
        return default


def parse_result(msg):
    meta = pmt.car(msg) if pmt.is_pair(msg) else msg
    return {
        "packet_id": dict_value(meta, "packet_id", -1),
        "status": dict_value(meta, "status", ""),
        "fcs_pass": dict_value(meta, "fcs_pass", False),
        "timing_metric": dict_value(meta, "timing_metric", math.nan),
        "cfo_hz": dict_value(meta, "cfo_hz", math.nan),
    }


def run_one(path: Path, template: np.ndarray, t0: int,
            workers: int, max_items: int) -> dict:
    source = blocks.file_source(2 * gr.sizeof_short, str(path), False)
    extractor = uwb.scheduled_extractor_sc16(
        FS_NATIVE, PERIOD_S, t0, PRE, CAPTURE, POST, 16)
    resampler = uwb.pdu_rational_resampler_ccf_65_48("quality_minorder")
    demod = uwb.realtime_demodulator.make_from_template(
        template.tolist(), max(1, workers), 64, "4z2", 0, "bypass", 9, 64, 14)
    debug = blocks.message_debug()
    top = gr.top_block("benchmark_scheduled_sc16_demod")
    top.connect(source, extractor)
    top.msg_connect(extractor, "packet", resampler, "packet")
    top.msg_connect(resampler, "packet", demod, "samples")
    top.msg_connect(demod, "result", debug, "store")
    source.set_max_noutput_items(max_items)
    extractor.set_max_noutput_items(max_items)
    top.set_max_noutput_items(max_items)

    started = time.perf_counter()
    top.run()
    try:
        demod.drain()
    except Exception:
        pass
    top.stop()
    top.wait()
    wall_s = time.perf_counter() - started

    results = [parse_result(debug.get_message(i))
               for i in range(debug.num_messages())]
    status_counts = {}
    for item in results:
        status_counts[item["status"]] = status_counts.get(item["status"], 0) + 1
    fcs = sum(1 for item in results if item["fcs_pass"])
    n_samples = path.stat().st_size // 4
    expected_complete, expected_attempted = expected_windows(n_samples, t0)
    result = {
        "input": str(path),
        "complex_samples": n_samples,
        "t0_native": t0,
        "extractor_scheduled_windows": extractor.scheduled_windows(),
        "extractor_emitted_windows": extractor.emitted_windows(),
        "extractor_dropped_windows": extractor.dropped_windows(),
        "expected_complete_windows": expected_complete,
        "expected_attempted_windows": expected_attempted,
        "resampler_received": resampler.pdus_received(),
        "resampler_emitted": resampler.pdus_emitted(),
        "resampler_dropped": resampler.pdus_dropped(),
        "demod_received": demod.jobs_received(),
        "demod_completed": demod.jobs_completed(),
        "demod_failed": demod.jobs_failed(),
        "demod_dropped": demod.jobs_dropped(),
        "result_messages": len(results),
        "status_counts": status_counts,
        "fcs_pass": fcs,
        "fcs_total": len(results),
        "wall_seconds": wall_s,
        "wall_input_msps": n_samples / wall_s / 1e6,
        "demod_latency_p95_us": demod.latency_p95_us(),
        "resampler_mean_us": (
            resampler.resample_total_us() / max(1, resampler.pdus_emitted())),
        "pass_accounting": (
            extractor.emitted_windows() == expected_complete
            and extractor.dropped_windows() == expected_attempted - expected_complete
            and resampler.pdus_dropped() == 0
            and demod.jobs_completed() + demod.jobs_failed()
            + demod.jobs_dropped() == demod.jobs_received()),
    }
    print(json.dumps(result, sort_keys=True))
    return result


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser()
    ap.add_argument("--inputs", nargs="+", required=True, type=Path)
    ap.add_argument("--template", type=Path,
                    default=Path("testdata/reference_preamble.bin"))
    ap.add_argument("--t0", type=int, default=3543362)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--max-items", type=int, default=1 << 20)
    ap.add_argument("--results", type=Path, default=None)
    return ap


def main() -> int:
    args = parser().parse_args()
    if args.max_items <= 0:
        raise SystemExit("--max-items must be positive")
    if not args.template.is_file():
        raise SystemExit(f"missing template: {args.template}")
    for path in args.inputs:
        if not path.is_file():
            raise SystemExit(f"missing input: {path}")
    template = np.fromfile(args.template, dtype=np.complex64)
    results = [run_one(path, template, args.t0, args.workers, args.max_items)
               for path in args.inputs]
    if args.results:
        args.results.parent.mkdir(parents=True, exist_ok=True)
        args.results.write_text(json.dumps(results, indent=2) + "\n",
                                encoding="utf-8")
        print(f"wrote {args.results}")
    return 0 if all(item["pass_accounting"] for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
