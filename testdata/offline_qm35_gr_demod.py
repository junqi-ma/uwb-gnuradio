#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline GNU Radio flowgraph for the QM35 X410 SC16 capture:

  file_source (interleaved int16 I/Q)
    -> interleaved_short_to_complex  (scale=1, matches prior diagnostics)
    -> UwbScheduledExtractor @737.28 MS/s  (t0/T known)
    -> UwbPduRationalResamplerCcf65_48 (65/48 -> 998.4)
    -> UwbRealtimeDemodulator (code-9, 64 SYNC, 4z2)
    -> message_debug (result)

Capture defaults match decode_qm35_sc16.py / docs:
  F:\\UWB基带数据\\qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat
  t0=3543552, T=5 ms (=3686400 samples @737.28)

Usage (from repo root):
  PYTHONPATH=$PWD/gr-uwb/build/test_modules LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib \\
    python3 testdata/offline_qm35_gr_demod.py [dat_path] \\
      [--max-slots=N] [--workers=N] [--no-lock] [--stream-from-disk]
      [--sc16-front-end] [--cir-filter-mode=auto|full|rake|bypass]

  Default input path preloads SC16→CF32 into RAM then vector_source (no disk
  during demod).  Use --stream-from-disk for the old file_source path.
  --sc16-front-end keeps SC16 through scheduled extraction and converts only
  inside the PDU 65/48 resampler (which still emits FC32).
"""
from __future__ import annotations

import collections
import csv
import os
import sys
import time

import numpy as np
from gnuradio import blocks, gr, uwb
import pmt as p

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))

DEFAULT_DAT = (
    "/mnt/f/UWB基带数据/qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat"
)
PREAMBLE = os.path.join(HERE, "reference_preamble.bin")
TAPS_MIN = os.path.join(HERE, "resampler_65_48", "taps_quality_minorder.txt")
TAPS_RT = os.path.join(HERE, "resampler_65_48", "taps_realtime.txt")
MATLAB_RESULTS = os.path.join(HERE, "qm35_matlab_compare", "matlab_results.csv")

FS737 = 737.28e6
FS998 = 998.4e6
T0 = 3543552
PERIOD_S = 0.005
# Window geometry @737.28 — generous around ~160 us QM35 packet.
PRE = 30000
CAP = 160000
POST = 10000


def as_int(v):
    for f in (p.to_long, p.to_uint64):
        try:
            return int(f(v))
        except Exception:
            pass
    return -1


def as_num(v):
    for f in (p.to_double, p.to_long, p.to_uint64):
        try:
            return float(f(v))
        except Exception:
            pass
    return float("nan")


def dict_get(md, key, default=None):
    return p.dict_ref(md, p.intern(key), default if default is not None else p.PMT_NIL)


def parse_result(msg):
    md = p.car(msg) if p.is_pair(msg) else msg
    status = p.symbol_to_string(dict_get(md, "status", p.intern("?")))
    fcs = bool(p.to_bool(dict_get(md, "fcs_pass", p.from_bool(False))))
    return {
        "packet_id": as_int(dict_get(md, "packet_id", p.from_uint64(0))),
        "status": status,
        "fcs": fcs,
        "det_start": as_int(dict_get(md, "detected_start_sample", p.from_long(-1))),
        "pred_start": as_int(dict_get(md, "predicted_start_sample", p.from_long(-1))),
        "win_start": as_int(dict_get(md, "window_start_sample", p.from_long(-1))),
        "timing_metric": as_num(dict_get(md, "timing_metric", p.from_double(-1))),
        "timing_peaks": as_int(dict_get(md, "timing_peaks", p.from_uint64(0))),
        "cfo_hz": as_num(dict_get(md, "cfo_hz", p.from_double(0))),
        "cfo_peaks_used": as_int(dict_get(md, "cfo_peaks_used", p.from_uint64(0))),
        "cfo_skipped_peaks": as_int(dict_get(md, "cfo_skipped_peaks", p.from_uint64(0))),
        "cfo_fit_first_peak": as_int(dict_get(md, "cfo_fit_first_peak_sample", p.from_long(-1))),
        "cfo_fit_last_peak": as_int(dict_get(md, "cfo_fit_last_peak_sample", p.from_long(-1))),
        "sfd_metric": as_num(dict_get(md, "sfd_metric", p.from_double(-1))),
        "sfd_final_det": as_int(dict_get(md, "sfd_start_sample", p.from_long(-1))),
        "sfd_initial_pred": as_int(dict_get(md, "sfd_initial_predicted_sample", p.from_long(-1))),
        "sfd_boot_det": as_int(dict_get(md, "sfd_bootstrap_detected_sample", p.from_long(-1))),
        "sfd_boot_first_valid_back": as_int(dict_get(md, "sfd_bootstrap_first_threshold_backtrack_symbols", p.from_long(-1))),
        "ns_sfd_metric": as_num(dict_get(md, "ns_sfd_metric", p.from_double(-1))),
        "ns_sfd_start": as_int(dict_get(md, "ns_sfd_start_chip", p.from_long(-1))),
        "phr_len": as_int(dict_get(md, "phr_psdu_length", p.from_long(-1))),
        "phr_unc": bool(p.to_bool(dict_get(md, "phr_uncorrectable", p.from_bool(False)))),
        "payload_n": as_int(dict_get(md, "payload_nbytes", p.from_long(-1))),
        "t_total_us": as_num(dict_get(md, "stage_total_us", p.from_double(-1))),
        "t_timing_us": as_num(dict_get(md, "stage_timing_us", p.from_double(-1))),
        "t_timing_coarse_us": as_num(dict_get(md, "stage_timing_coarse_us", p.from_double(-1))),
        "t_timing_fine_track_us": as_num(dict_get(md, "stage_timing_fine_track_us", p.from_double(-1))),
        "t_resample_us": as_num(dict_get(md, "resample_us", p.from_double(-1))),
        "t_cfo_us": as_num(dict_get(md, "stage_cfo_us", p.from_double(-1))),
        "t_sfd_us": as_num(dict_get(md, "stage_sfd_us", p.from_double(-1))),
        "sfd_boot_windows": as_int(dict_get(md, "sfd_bootstrap_windows", p.from_uint64(0))),
        "sfd_boot_coarse": as_int(dict_get(md, "sfd_bootstrap_coarse_correlations", p.from_uint64(0))),
        "sfd_boot_fine": as_int(dict_get(md, "sfd_bootstrap_fine_correlations", p.from_uint64(0))),
        "sfd_final_windows": as_int(dict_get(md, "sfd_final_windows", p.from_uint64(0))),
        "sfd_final_coarse": as_int(dict_get(md, "sfd_final_coarse_correlations", p.from_uint64(0))),
        "sfd_final_fine": as_int(dict_get(md, "sfd_final_fine_correlations", p.from_uint64(0))),
        "t_cir_us": as_num(dict_get(md, "stage_cir_us", p.from_double(-1))),
        "t_ns_sfd_us": as_num(dict_get(md, "stage_ns_sfd_us", p.from_double(-1))),
        "t_phr_us": as_num(dict_get(md, "stage_phr_us", p.from_double(-1))),
        "t_payload_us": as_num(dict_get(md, "stage_payload_us", p.from_double(-1))),
        "t_queue_us": as_num(dict_get(md, "queue_delay_us", p.from_double(-1))),
        "t_demod_lat_us": as_num(dict_get(md, "demod_latency_us", p.from_double(-1))),
        "t_wall_lat_us": as_num(dict_get(md, "wall_latency_us", p.from_double(-1))),
        "t_cir_est_us": as_num(dict_get(md, "cir_estimate_us", p.from_double(-1))),
        "t_cir_soft_us": as_num(dict_get(md, "cir_softfir_us", p.from_double(-1))),
        "t_cir_post_us": as_num(dict_get(md, "cir_postprocess_us", p.from_double(-1))),
    }


def print_packet_detection_statistics(results, scheduled_slots):
    """Report preamble/timing detection independently of decode/FCS outcome."""
    print()
    print("=== Packet detection statistics ===")
    published = len(results)
    timing_ok = [r for r in results if r["det_start"] >= 0]
    full_train = [r for r in timing_ok if r["timing_peaks"] >= 64]
    partial_train = [r for r in timing_ok if 0 < r["timing_peaks"] < 64]
    timing_failed = [r for r in results if r["status"] == "timing_failed"]
    print(f"scheduled_windows={scheduled_slots} published_results={published} "
          f"eos_unpublished={max(0, scheduled_slots - published)}")
    print(f"preamble_detected={len(timing_ok)}/{published} "
          f"({len(timing_ok)/published if published else 0:.1%}) "
          f"timing_failed={len(timing_failed)}")
    print(f"full_64_sync={len(full_train)}/{len(timing_ok)} "
          f"({len(full_train)/len(timing_ok) if timing_ok else 0:.1%}) "
          f"partial_sync={len(partial_train)}")

    if not timing_ok:
        return
    metrics = np.asarray([r["timing_metric"] for r in timing_ok], dtype=np.float64)
    offsets = np.asarray(
        [r["det_start"] - r["pred_start"] for r in timing_ok
         if r["pred_start"] >= 0], dtype=np.float64)
    print("timing_metric: "
          f"min={np.min(metrics):.3f} p05={np.percentile(metrics, 5):.3f} "
          f"med={np.median(metrics):.3f} p95={np.percentile(metrics, 95):.3f} "
          f"max={np.max(metrics):.3f}")
    if offsets.size:
        print("detected-predicted (998.4 MS/s samples): "
              f"min={np.min(offsets):+.0f} p05={np.percentile(offsets, 5):+.0f} "
              f"med={np.median(offsets):+.0f} p95={np.percentile(offsets, 95):+.0f} "
              f"max={np.max(offsets):+.0f}")
        if offsets.size >= 2:
            slots = np.asarray([r["packet_id"] for r in timing_ok
                                if r["pred_start"] >= 0], dtype=np.float64)
            slope, intercept = np.polyfit(slots, offsets, 1)
            print(f"schedule drift: offset={intercept:+.1f} "
                  f"{slope:+.3f} samples/slot")

    # MATLAB reports the same absolute 998.4 MS/s coordinate.  This check is
    # intentionally detection-only: it does not require PHR/payload/FCS.
    if not os.path.isfile(MATLAB_RESULTS):
        print("MATLAB start comparison: unavailable (matlab_results.csv missing)")
        return
    try:
        with open(MATLAB_RESULTS, newline="", encoding="utf-8") as f:
            matlab = {int(row["slot"]): row for row in csv.DictReader(f)}
        diffs = []
        for r in timing_ok:
            row = matlab.get(r["packet_id"])
            if not row:
                continue
            matlab_start = int(float(row["detected_start_out"]))
            if matlab_start >= 0:
                diffs.append(r["det_start"] - matlab_start)
        if diffs:
            d = np.asarray(diffs, dtype=np.float64)
            exact = int(np.count_nonzero(d == 0.0))
            print("MATLAB start comparison: "
                  f"common={d.size} exact={exact}/{d.size} "
                  f"max_abs_error={np.max(np.abs(d)):.0f} samples")
    except (KeyError, OSError, ValueError) as err:
        print(f"MATLAB start comparison: unavailable ({err})")


def print_sfd_correlation_statistics(results):
    """Print exact normalized SFD correlation evaluations per demod attempt."""
    if not results:
        return
    print()
    print("=== SFD correlation-count statistics ===")
    print("One correlation = one normalized complex dot against the 8128-sample "
          "QM35 4z2 SFD template; bootstrap and final passes are distinct.")
    labels = (
        ("bootstrap", "sfd_boot_windows", "sfd_boot_coarse", "sfd_boot_fine"),
        ("final", "sfd_final_windows", "sfd_final_coarse", "sfd_final_fine"),
    )
    grand = []
    print(f"{'pass':<10} {'windows':>10} {'coarse':>10} {'fine':>10} "
          f"{'total':>10} {'total/attempt':>15}")
    for label, wk, ck, fk in labels:
        wins = np.asarray([r[wk] for r in results], dtype=np.int64)
        coarse = np.asarray([r[ck] for r in results], dtype=np.int64)
        fine = np.asarray([r[fk] for r in results], dtype=np.int64)
        total = coarse + fine
        grand.append(total)
        print(f"{label:<10} {int(np.sum(wins)):10d} {int(np.sum(coarse)):10d} "
              f"{int(np.sum(fine)):10d} {int(np.sum(total)):10d} "
              f"{np.mean(total):15.1f}")
    total = grand[0] + grand[1]
    print(f"{'all':<10} {'' :>10} {'' :>10} {'' :>10} "
          f"{int(np.sum(total)):10d} {np.mean(total):15.1f}")
    print("per-attempt total correlations: "
          f"min={int(np.min(total))} p05={np.percentile(total, 5):.0f} "
          f"med={np.median(total):.0f} p95={np.percentile(total, 95):.0f} "
          f"max={int(np.max(total))}")


def print_sfd_prediction_offset_statistics(results):
    """Compare the initial preamble-derived SFD guess with final SFD timing."""
    pairs = [r for r in results if r["sfd_initial_pred"] >= 0
             and r["sfd_final_det"] >= 0]
    if not pairs:
        return
    initial = np.asarray([r["sfd_initial_pred"] for r in pairs], dtype=np.int64)
    final = np.asarray([r["sfd_final_det"] for r in pairs], dtype=np.int64)
    bootstrap = np.asarray([r["sfd_boot_det"] for r in pairs], dtype=np.int64)
    delta = final - initial
    print()
    print("=== Initial-preamble SFD prediction offset (998.4 MS/s samples) ===")
    print("delta = final CFO-aligned SFD start - initial timing/SYNC prediction")
    print(f"n={delta.size} min={np.min(delta):+d} p05={np.percentile(delta, 5):+.0f} "
          f"med={np.median(delta):+.0f} p95={np.percentile(delta, 95):+.0f} "
          f"max={np.max(delta):+d} mean={np.mean(delta):+.1f}")
    exact = int(np.count_nonzero(delta == 0))
    within16 = int(np.count_nonzero(np.abs(delta) <= 16))
    print(f"exact={exact}/{delta.size} within_±16_samples={within16}/{delta.size} "
          f"max_abs={np.max(np.abs(delta))} "
          f"({np.max(np.abs(delta))/998.4:.3f} ns)")
    bins = collections.Counter(delta.tolist())
    print("offset histogram (SYNC symbols; samples: packets): " +
          ", ".join(f"{off // 1016:+d} ({off:+d}): {count}"
                    for off, count in sorted(bins.items())))
    first_valid = np.asarray([r["sfd_boot_first_valid_back"] for r in pairs],
                             dtype=np.int64)
    required = -delta // 1016
    usable = first_valid >= 0
    if np.any(usable):
        false_early = int(np.count_nonzero(first_valid[usable] < required[usable]))
        exact_first = int(np.count_nonzero(first_valid[usable] == required[usable]))
        print("threshold-stop diagnostic (current global-max search, no behavior change): "
              f"first_valid_available={np.count_nonzero(usable)}/{delta.size} "
              f"first_valid_equals_correct={exact_first}/{np.count_nonzero(usable)} "
              f"false_early={false_early}")
    valid_boot = bootstrap >= 0
    if np.any(valid_boot):
        bdelta = final[valid_boot] - bootstrap[valid_boot]
        print("bootstrap-correlated SFD vs final: "
              f"max_abs={np.max(np.abs(bdelta))} samples, "
              f"exact={np.count_nonzero(bdelta == 0)}/{bdelta.size}")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    dat = args[0] if args else DEFAULT_DAT
    max_slots = None
    num_workers = 2
    timing_coarse_stride = 14
    cir_filter_mode = "bypass"
    output_csv = os.path.join(HERE, "offline_qm35_gr_demod_results.csv")
    for f in flags:
        if f.startswith("--max-slots="):
            max_slots = int(f.split("=", 1)[1])
        elif f.startswith("--workers="):
            num_workers = max(1, int(f.split("=", 1)[1]))
        elif f.startswith("--timing-coarse-stride="):
            timing_coarse_stride = max(1, int(f.split("=", 1)[1]))
        elif f.startswith("--cir-filter-mode="):
            cir_filter_mode = f.split("=", 1)[1]
            if cir_filter_mode not in ("auto", "full", "rake", "bypass"):
                raise ValueError("--cir-filter-mode must be auto, full, rake, or bypass")
        elif f.startswith("--output-csv="):
            output_csv = os.path.abspath(f.split("=", 1)[1])

    if not os.path.isfile(dat):
        print(f"ERROR: file not found: {dat}", file=sys.stderr)
        return 2

    nbytes = os.path.getsize(dat)
    n_complex = nbytes // 4
    duration_s = n_complex / FS737
    # How many full scheduled slots fit (window must fully fit).
    win = PRE + CAP + POST
    last_pred = T0
    n_slots = 0
    while True:
        pred = T0 + n_slots * int(round(PERIOD_S * FS737))
        # window end = pred + CAP + POST must be < n_complex
        if pred + CAP + POST > n_complex:
            break
        if pred < PRE:
            n_slots += 1
            continue
        last_pred = pred
        n_slots += 1
        if max_slots is not None and n_slots >= max_slots:
            break
    # recount cleanly
    n_slots = 0
    for k in range(0, 1000):
        pred = T0 + k * int(round(PERIOD_S * FS737))
        if pred < PRE:
            continue
        if pred + CAP + POST > n_complex:
            break
        n_slots += 1
        if max_slots is not None and n_slots >= max_slots:
            break

    period_samples = int(round(PERIOD_S * FS737))
    print("=== Offline GR demod: QM35 SC16 capture ===")
    print(f"file: {dat}")
    print(f"samples: {n_complex} @737.28  duration={duration_s:.4f}s")
    print(f"schedule: t0={T0} period={period_samples} ({PERIOD_S*1e3:.1f} ms) "
          f"window=[{PRE},{CAP},{POST}] slots={n_slots}")

    tmpl = np.fromfile(PREAMBLE, np.complex64)
    taps_path = TAPS_MIN if os.path.isfile(TAPS_MIN) else TAPS_RT
    print(f"resampler taps: {os.path.basename(taps_path)}  "
          f"template_len={len(tmpl)}")

    # --- preload source ---
    # Only the samples needed for scheduled windows (+ pad for EOS sentinel).
    last_end = T0 + (n_slots - 1) * period_samples + CAP + POST
    n_stream = min(n_complex,
                   int(last_end + period_samples // 4 + 4096))
    stream_from_disk = "--stream-from-disk" in flags
    sc16_front_end = "--sc16-front-end" in flags
    load_s = 0.0
    convert_s = 0.0
    stage_s = 0.0
    if stream_from_disk:
        print("input: stream-from-disk (file_source)")
        if sc16_front_end:
            src = blocks.file_source(2 * gr.sizeof_short, dat, False)
            head = blocks.head(2 * gr.sizeof_short, n_stream)
            connect_src = lambda tb, ext: tb.connect(src, head, ext)
        else:
            src = blocks.file_source(gr.sizeof_short, dat, False)
            s2c = blocks.interleaved_short_to_complex(False, False, 1.0)
            head = blocks.head(gr.sizeof_gr_complex, n_stream)
            connect_src = lambda tb, ext: tb.connect(src, s2c, head, ext)
    else:
        preload_gib = n_stream * (4 if sc16_front_end else 8) / 1e9
        preload_fmt = "SC16" if sc16_front_end else "CF32"
        print(f"input: preload to RAM  n_stream={n_stream} "
              f"({preload_gib:.2f} GiB {preload_fmt}) ...", flush=True)
        t_load0 = time.time()
        # Interleaved int16 I,Q — same scale=1 as interleaved_short_to_complex.
        n_i16 = n_stream * 2
        raw = np.fromfile(dat, dtype=np.int16, count=n_i16)
        if raw.size < n_i16:
            raise RuntimeError(
                f"short read: got {raw.size} int16, need {n_i16}")
        sc16_bytes = int(raw.nbytes)
        load_s = time.time() - t_load0
        if sc16_front_end:
            print(f"  loaded in {load_s:.2f}s  "
                  f"({sc16_bytes / 1e9:.2f} GiB SC16; no full-stream FC32)",
                  flush=True)
        else:
            # Own CF32 buffer (not a view of staging) so we can free SC16.
            t_convert0 = time.time()
            iq = np.empty(n_stream, dtype=np.complex64)
            iq.real = raw[0::2]
            iq.imag = raw[1::2]
            del raw
            convert_s = time.time() - t_convert0
            print(f"  loaded in {load_s:.2f}s  "
                  f"({sc16_bytes / 1e9:.2f} GiB SC16 → "
                  f"{iq.nbytes / 1e9:.2f} GiB CF32)", flush=True)
        # Prefer RAM-backed file_source via /dev/shm when the array is huge:
        # vector_source_c(numpy) may copy through Python and thrash.
        t_vs0 = time.time()
        shm_path = None
        if n_stream >= 8_000_000 and os.path.isdir("/dev/shm"):
            suffix = "sc16" if sc16_front_end else "cf32"
            shm_path = f"/dev/shm/qm35_offline_{os.getpid()}.{suffix}"
            if sc16_front_end:
                raw.tofile(shm_path)
                del raw
                src = blocks.file_source(2 * gr.sizeof_short, shm_path, False)
            else:
                iq.tofile(shm_path)
                src = blocks.file_source(gr.sizeof_gr_complex, shm_path, False)
            stage_s = time.time() - t_vs0
            print(f"  feed via /dev/shm file_source ({shm_path}) in "
                  f"{stage_s:.2f}s", flush=True)
            if not sc16_front_end:
                del iq  # data lives on tmpfs
        else:
            if sc16_front_end:
                # Small captures only; full captures use the /dev/shm branch.
                src = blocks.vector_source_s(raw, False)
                raise RuntimeError("--sc16-front-end requires /dev/shm for paired SC16 source")
            src = blocks.vector_source_c(iq, False)
            stage_s = time.time() - t_vs0
            print(f"  vector_source ready in {stage_s:.2f}s",
                  flush=True)
        connect_src = lambda tb, ext: tb.connect(src, ext)

    # Wide timing search (demod core) + learn-then-freeze schedule lock.
    # SYNC/preamble timing → schedule_feedback → lock_obs learns fixed δ
    # (T = 5000 us + δ), freezes on Hold.  Not FCS fast loop.
    disable_lock = "--no-lock" in flags
    if sc16_front_end:
        if not disable_lock:
            raise RuntimeError("SC16 scheduled extractor currently requires --no-lock")
        extractor = uwb.scheduled_extractor_sc16(
            FS737, PERIOD_S, T0, PRE, CAP, POST, 16,
            uwb.scheduled_extractor.EmitPolicy.EverySlot)
        print("schedule_lock=OFF (SC16 scheduled extraction) + wide timing search")
    else:
        extractor = uwb.scheduled_extractor(
            FS737, PERIOD_S, T0, PRE, CAP, POST, 16,
            uwb.scheduled_extractor.EmitPolicy.EverySlot, False)
        extractor.set_schedule_lock_enabled(not disable_lock)
        print(f"schedule_lock={'ON (SYNC learn→freeze)' if not disable_lock else 'OFF'} "
              f"+ wide timing search")
    print(f"demod_workers={num_workers} queue_capacity=64 "
          f"timing_coarse_stride={timing_coarse_stride} "
          f"cir_filter_mode={cir_filter_mode}")
    resampler = uwb.pdu_rational_resampler_ccf_65_48.make_from_taps(
        np.fromfile(taps_path, np.float32).tolist(), FS998
    )
    demod = uwb.realtime_demodulator.make_from_template(
        tmpl.tolist(),
        num_workers,
        64,      # queue
        "4z2",
        0,       # full CIR
        cir_filter_mode,
        9,       # code index
        64,      # preamble repetitions
        timing_coarse_stride,
    )
    dbg = blocks.message_debug()
    dbg_status = blocks.message_debug()
    dbg_lock = blocks.message_debug()

    tb = gr.top_block("offline_qm35_gr_demod")
    connect_src(tb, extractor)
    tb.msg_connect(extractor, "packet", resampler, "packet")
    tb.msg_connect(resampler, "packet", demod, "samples")
    tb.msg_connect(demod, "result", dbg, "store")
    tb.msg_connect(demod, "status", dbg_status, "store")
    # SYNC timing → learn fixed δ, freeze on Hold for subsequent windows.
    if not disable_lock and not sc16_front_end:
        tb.msg_connect(demod, "schedule_feedback", extractor, "lock_obs")
        tb.msg_connect(extractor, "status", dbg_lock, "store")

    t0 = time.time()
    tb.start()
    # Wait until the pipeline is idle: extractor finished emitting and demod
    # has finished every received job.  Do not hard-require want==n_slots —
    # the last scheduled window can miss EOS by one sentinel sample.
    want = n_slots
    deadline = time.time() + max(180.0, n_slots * 15.0)
    last_print = 0.0
    idle_since = None
    prev_snap = None
    active_done_s = None
    while time.time() < deadline:
        done = dbg.num_messages()
        rx = demod.jobs_received()
        comp = demod.jobs_completed()
        fail = demod.jobs_failed()
        drop = demod.jobs_dropped()
        finished = comp + fail + drop
        ext_em = extractor.emitted_windows()
        snap = (done, ext_em, rx, finished)
        if snap != prev_snap:
            prev_snap = snap
            idle_since = time.time()
        # All known work done: every received job finished and result count
        # has caught up (or no more jobs are in flight).
        in_flight = max(0, rx - finished)
        # This is the performance endpoint: every scheduled window has passed
        # through extraction, resampling and demod.  Keep draining afterwards
        # only to collect message_debug output; its fixed idle timeout is not
        # receiver compute time.
        if (active_done_s is None and ext_em >= want and rx >= want and
                finished >= want and in_flight == 0):
            active_done_s = time.time() - t0
        if rx > 0 and in_flight == 0 and done >= finished and ext_em >= rx:
            # require a short idle so late message_debug posts land
            if idle_since is not None and (time.time() - idle_since) >= 0.3:
                break
        # Full success path
        if done >= want and in_flight == 0:
            break
        # No progress for a while after most slots — stop waiting for a missing
        # last window (common with EOS sentinel leaving one slot incomplete).
        if (ext_em >= max(1, want - 1) and in_flight == 0 and
                idle_since is not None and (time.time() - idle_since) >= 3.0):
            print(f"  ... idle-stop: results={done}/{want} ext={ext_em} "
                  f"rx={rx} finished={finished}")
            break
        if time.time() - last_print > 2.0:
            print(f"  ... progress: results={done}/{want} "
                  f"ext_emitted={ext_em} "
                  f"resamp={resampler.pdus_emitted()} "
                  f"demod_rx={rx} "
                  f"comp={comp} fail={fail} drop={drop} "
                  f"wall={time.time()-t0:.1f}s")
            last_print = time.time()
        time.sleep(0.05)
    # drain
    try:
        demod.drain()
    except Exception:
        pass
    time.sleep(0.2)
    tb.stop()
    tb.wait()
    wall = time.time() - t0
    # Drop tmpfs feed if used.
    try:
        if "shm_path" in locals() and shm_path and os.path.isfile(shm_path):
            os.remove(shm_path)
    except OSError:
        pass

    results = [parse_result(dbg.get_message(i)) for i in range(dbg.num_messages())]
    results.sort(key=lambda r: r["packet_id"])

    status_hist = collections.Counter(r["status"] for r in results)
    fcs_ok = sum(1 for r in results if r["fcs"])
    succ = [r for r in results if r["status"] == "success"]
    tus = [r["t_total_us"] for r in results if r["t_total_us"] == r["t_total_us"]]
    ttim = [r["t_timing_us"] for r in results if r["t_timing_us"] == r["t_timing_us"]]
    print()
    print("=== Pipeline counters ===")
    print(f"load_s={load_s:.2f}")
    print(f"format_convert_s={convert_s:.2f}  (SC16 stream → full FC32 buffer)")
    print(f"source_stage_s={stage_s:.2f}  (/dev/shm write or vector-source setup)")
    print(f"wall_s={wall:.2f}  (process only; excludes preload)")
    if active_done_s is not None:
        print(f"active_pipeline_s={active_done_s:.2f}  "
              "(all scheduled extraction + resample + demod complete; "
              "excludes idle-drain wait)")
        print(f"active_slots_per_s={n_slots/active_done_s:.3f}")
    print(f"throughput_slots_per_s="
          f"{(len(results) / wall) if wall > 0 else 0:.3f}")
    print(f"wall_ms_per_slot="
          f"{(1000.0 * wall / len(results)) if results else 0:.1f}")
    if load_s > 0:
        print(f"total_s={load_s + convert_s + stage_s + wall:.2f}  "
              "(load+format conversion+source staging+process)")
    print(f"extractor.emitted={extractor.emitted_windows()} "
          f"scheduled={extractor.scheduled_windows()}")
    if sc16_front_end:
        print(f"extractor_sc16.process_ms={extractor.process_total_us()/1000:.1f} "
              f"copy_ms={extractor.copy_total_us()/1000:.1f} "
              f"publish_ms={extractor.publish_total_us()/1000:.1f}")
    print(f"resampler.emitted={resampler.pdus_emitted()} "
          f"dropped={resampler.pdus_dropped()} "
          f"fir_total_ms={resampler.resample_total_us()/1000:.1f} "
          f"fir_max_us={resampler.resample_max_us()} "
          f"handler_total_ms={resampler.handler_total_us()/1000:.1f} "
          f"sc16_convert_ms={resampler.input_convert_total_us()/1000:.1f} "
          f"publish_ms={resampler.publish_total_us()/1000:.1f}")
    print(f"demod workers={demod.num_workers()} "
          f"rx={demod.jobs_received()} completed={demod.jobs_completed()} "
          f"failed={demod.jobs_failed()} dropped={demod.jobs_dropped()} "
          f"invalid={demod.invalid_inputs()}")
    print(f"results={len(results)}  fcs_pass={fcs_ok}  "
          f"fcs_rate={fcs_ok/len(results) if results else 0:.3f}")
    print(f"status histogram: {dict(status_hist)}")
    if tus:
        print(f"demod_stage_total_us: mean={float(np.mean(tus)):.0f} "
              f"med={float(np.median(tus)):.0f} "
              f"p95={float(np.percentile(tus, 95)):.0f} "
              f"max={float(np.max(tus)):.0f}")
    if ttim:
        print(f"demod_stage_timing_us: mean={float(np.mean(ttim)):.0f} "
              f"med={float(np.median(ttim)):.0f}")

    # `stage_total_us` is worker compute time only. The 65/48 PDU FIR is
    # propagated independently in `resample_us`; source and scheduled
    # extraction remain part of the end-to-end wall measurement.
    demod_compute_sum_us = float(np.sum(tus)) if tus else 0.0
    print("receiver timing scope: "
          f"demod_worker_compute_sum_ms={demod_compute_sum_us/1000:.1f} "
          f"ideal_{num_workers}worker_lower_bound_ms="
          f"{demod_compute_sum_us/(1000*max(1, num_workers)):.1f} "
          f"flowgraph_wall_ms={wall*1000:.1f} "
          "(wall includes source + extractor + resampler + queues)")

    print("front-end selection: energy_gate=NOT_RUN, detector_coarse=NOT_RUN "
          "(this QM35 flowgraph uses radar t0/T scheduled extraction); "
          "demod timing_coarse is reported below")

    # Per-stage demod breakdown (all results + success-only).
    stage_keys = [
        ("packet_detect", "t_timing_us"),
        ("cfo", "t_cfo_us"),
        ("sfd", "t_sfd_us"),
        ("cir", "t_cir_us"),
        ("ns_sfd", "t_ns_sfd_us"),
        ("phr", "t_phr_us"),
        ("payload", "t_payload_us"),
        ("queue", "t_queue_us"),
    ]
    # These timers are nested inside `cir`, and therefore must not be added
    # to the top-level table or its percentages would double-count CIR time.
    cir_detail_keys = [
        ("cir_est", "t_cir_est_us"),
        ("cir_soft", "t_cir_soft_us"),
        ("cir_post", "t_cir_post_us"),
    ]

    def _stats(vals):
        if not vals:
            return None
        a = np.asarray(vals, dtype=np.float64)
        return (float(np.mean(a)), float(np.median(a)),
                float(np.percentile(a, 95)), float(np.max(a)), float(np.sum(a)))

    print()
    print("=== Per-packet demod compute breakdown (µs) ===")
    print("packet_detect = preamble coarse search + fine correlation + "
          "SYNC tracking + period fit")
    print(f"{'stage':<14} {'scope':<8} {'mean':>8} {'med':>8} {'p95':>8} "
          f"{'max':>8} {'sum_ms':>8} {'%total':>7}")
    for scope_name, subset in (("all", results), ("success", succ)):
        tot_vals = [r["t_total_us"] for r in subset
                    if r["t_total_us"] == r["t_total_us"] and r["t_total_us"] >= 0]
        tot_sum = float(np.sum(tot_vals)) if tot_vals else 0.0
        st = _stats(tot_vals)
        if st:
            print(f"{'TOTAL':<14} {scope_name:<8} {st[0]:8.0f} {st[1]:8.0f} "
                  f"{st[2]:8.0f} {st[3]:8.0f} {st[4]/1000:8.1f} {'100.0':>7}")
        for label, key in stage_keys:
            vals = [r[key] for r in subset
                    if r.get(key, float("nan")) == r.get(key, float("nan"))
                    and r.get(key, -1) >= 0]
            st = _stats(vals)
            if not st:
                continue
            pct = (100.0 * st[4] / tot_sum) if tot_sum > 0 else 0.0
            print(f"{label:<14} {scope_name:<8} {st[0]:8.0f} {st[1]:8.0f} "
                  f"{st[2]:8.0f} {st[3]:8.0f} {st[4]/1000:8.1f} {pct:6.1f}%")

    print()
    print("=== Acquisition / upsampling detail (µs; not double-counted above) ===")
    print("timing_coarse = stride-14 preamble scan; timing_fine_track = local "
          "refine + SYNC tracking. resample_65_48 = FIR process+flush only.")
    print(f"{'stage':<22} {'scope':<8} {'mean':>8} {'med':>8} {'p95':>8} "
          f"{'max':>8} {'sum_ms':>8}")
    for scope_name, subset in (("all", results), ("success", succ)):
        for label, key in (("timing_coarse", "t_timing_coarse_us"),
                           ("timing_fine_track", "t_timing_fine_track_us"),
                           ("resample_65_48", "t_resample_us")):
            vals = [r[key] for r in subset
                    if r.get(key, float("nan")) == r.get(key, float("nan"))
                    and r.get(key, -1) >= 0]
            st = _stats(vals)
            if st:
                print(f"{label:<22} {scope_name:<8} {st[0]:8.0f} {st[1]:8.0f} "
                      f"{st[2]:8.0f} {st[3]:8.0f} {st[4]/1000:8.1f}")

    print()
    print("=== CIR internal breakdown (nested in cir; µs) ===")
    print(f"{'stage':<14} {'scope':<8} {'mean':>8} {'med':>8} {'p95':>8} "
          f"{'max':>8} {'sum_ms':>8} {'%cir':>7}")
    for scope_name, subset in (("all", results), ("success", succ)):
        cir_vals = [r["t_cir_us"] for r in subset
                    if r["t_cir_us"] == r["t_cir_us"] and r["t_cir_us"] >= 0]
        cir_sum = float(np.sum(cir_vals)) if cir_vals else 0.0
        for label, key in cir_detail_keys:
            vals = [r[key] for r in subset
                    if r.get(key, float("nan")) == r.get(key, float("nan"))
                    and r.get(key, -1) >= 0]
            st = _stats(vals)
            if not st:
                continue
            pct = (100.0 * st[4] / cir_sum) if cir_sum > 0 else 0.0
            print(f"{label:<14} {scope_name:<8} {st[0]:8.0f} {st[1]:8.0f} "
                  f"{st[2]:8.0f} {st[3]:8.0f} {st[4]/1000:8.1f} {pct:6.1f}%")
    if not sc16_front_end and extractor.schedule_lock_enabled():
        n_lock_st = dbg_lock.num_messages()
        locked_T_s = extractor.locked_packet_interval_s()
        locked_T_samp = locked_T_s * FS737
        delta = locked_T_samp - period_samples
        print(f"schedule_lock state={extractor.schedule_lock_state()} "
              f"updates={extractor.schedule_lock_updates()} "
              f"locked_T_s={locked_T_s:.9f} "
              f"locked_T_samp={locked_T_samp:.2f} "
              f"delta_samp={delta:+.2f} "
              f"locked_t0={extractor.locked_first_packet_sample():.1f} "
              f"bias_t0={extractor.locked_bias_t0_samples():+.1f} "
              f"status_msgs={n_lock_st}")

    print_packet_detection_statistics(results, n_slots)
    print_sfd_correlation_statistics(results)
    print_sfd_prediction_offset_statistics(results)

    print()
    print("=== Per-slot results ===")
    print(f"{'slot':>4} {'status':<16} {'fcs':>3} {'peaks':>5} "
          f"{'t_met':>6} {'cfoHz':>9} {'sfd':>6} {'ns_sfd':>6} "
          f"{'phr':>4} {'pay':>4} {'det-pred':>9} {'tus':>6}")
    for r in results:
        dpred = (r["det_start"] - r["pred_start"]
                 if r["det_start"] >= 0 and r["pred_start"] >= 0 else None)
        print(f"{r['packet_id']:4d} {r['status']:<16} {int(r['fcs']):3d} "
              f"{r['timing_peaks']:5d} {r['timing_metric']:6.3f} "
              f"{r['cfo_hz']:9.1f} {r['sfd_metric']:6.3f} "
              f"{r['ns_sfd_metric']:6.3f} {r['phr_len']:4d} "
              f"{r['payload_n']:4d} "
              f"{(dpred if dpred is not None else -999999):+9d} "
              f"{r['t_total_us']:6.0f}")

    # Seed alignment diagnostics for failures
    print()
    print("=== Failure analysis ===")
    fails = [r for r in results if r["status"] != "success"]
    if not fails:
        print("No failures.")
    else:
        by = collections.Counter(r["status"] for r in fails)
        print(f"failed={len(fails)} breakdown={dict(by)}")
        # timing_failed: look at det/pred offsets of successes for comparison
        if succ:
            offs = [r["det_start"] - r["pred_start"] for r in succ
                    if r["det_start"] >= 0]
            print(f"success det-pred: min={min(offs)} med={int(np.median(offs))} "
                  f"max={max(offs)} n={len(offs)}")
            lens = collections.Counter(r["phr_len"] for r in succ)
            print(f"success phr_len hist: {dict(lens)}")
        # show first few failures with more fields
        for r in fails[:12]:
            print(f"  FAIL slot={r['packet_id']} status={r['status']} "
                  f"peaks={r['timing_peaks']} t_met={r['timing_metric']:.3f} "
                  f"pred={r['pred_start']} det={r['det_start']} "
                  f"win={r['win_start']} cfo={r['cfo_hz']:.1f} "
                  f"sfd={r['sfd_metric']:.3f} ns={r['ns_sfd_metric']:.3f} "
                  f"phr_len={r['phr_len']} phr_unc={r['phr_unc']}")

    # Save CSV next to capture for later comparison
    with open(output_csv, "w", encoding="utf-8") as f:
        keys = list(results[0].keys()) if results else []
        f.write(",".join(keys) + "\n")
        for r in results:
            f.write(",".join(str(r[k]) for k in keys) + "\n")
    print(f"\nWrote {output_csv}")
    return 0 if fcs_ok > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
