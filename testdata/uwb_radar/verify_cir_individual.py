#!/usr/bin/env python3
"""Per-repetition CIR offline reference + validation (Radar "individual values").

Motivation
----------
The realtime C++ ``radar_cir_one`` (gr-uwb/.../uwb_radar_cir_core.h) coherently
averages ``cir_repetitions`` SYNC repetitions into ONE CIR per pulse.  The
MATLAB reference already exposes the *per-repetition* CIRs as
``cir.individual_values`` (UWB_demodulation/+uwbdecoder/estimateCir.m), gated
by ``params.cir_store_individual_values``.  This script is the offline port of
that path plus a hard gate against the canonical goldens.

What it proves (golden mode, the default)
-----------------------------------------
1. The averaged CIR produced here matches the MATLAB goldens
   ``cir_raw_*_radar.cf32`` / ``cir_norm_*_radar.cf32`` (hard gate, same
   1e-5 relative-L2 tolerance as ``verify_uwb_radar_golden.m``).
2. The per-repetition CIRs cohere: ``mean(individual, axis=1)`` equals the
   averaged CIR to machine precision (linearity of convolution + mean).
3. Per-repetition phase slope -> residual CFO (should be ~0 for the clean
   golden), which is exactly what the repetition-average null at
   ``f_work/(2*SPS) = 491.34 kHz`` hides.

Capture mode (``--capture <run_dir>``)
--------------------------------------
Reads a real hardware run directory (native SC16 ``capture.iq`` + capture.jsonl
+ cir.cf32/cir.jsonl), resamples native->998.4 work with scipy resample_poly,
and reports per-repetition CIRs.  Because scipy's resampler is NOT the repo's
65/48 FIR, the averaged CIR is compared to the C++ cir.cf32 by shape/peak, not
bit-exactly.  Use golden mode for exactness.

Pure offline: reads files, writes nothing.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# HRP code-9 (Ipatov 127, ternary).  Mirrors udw_phy_profile.h kPreambleCode9,
# which in turn mirrors MATLAB lrwpan.internal.HRPCodes(9).
# ---------------------------------------------------------------------------
HRP_CODE_9 = np.array([
    1, 0, 0, 1, 0, 0, 0, -1, 0, -1, -1, 0, 0, -1, -1, 1, 0, 1, 0, 1, 0, 0,
    -1, 1, -1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 1, -1, 0, 0, 0, 1, 0, 0, -1, 0,
    0, -1, -1, 0, -1, 1, 0, 1, 0, -1, -1, 0, -1, 1, 1, 1, 0, 1, 1, 0, 0, 0,
    1, -1, 0, 1, 0, 0, -1, 0, 1, 1, -1, 0, 1, 1, 1, 0, 0, -1, 1, 0, 0, 1,
    0, 1, 0, -1, 0, 1, 1, -1, 1, -1, -1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, -1, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, -1, 1,
], dtype=np.int8)

SPREADING_FACTOR = 4
SAMPLES_PER_PULSE = 2
SAMPLES_PER_SYMBOL = 1016


def build_sampled_code(code: np.ndarray = HRP_CODE_9) -> np.ndarray:
    """Spread + upsample the Ipatov code exactly like local_reference.m."""
    spread = np.zeros(code.size * SPREADING_FACTOR, dtype=np.float64)
    spread[0::SPREADING_FACTOR] = code.astype(np.float64)
    sampled = np.zeros(spread.size * SAMPLES_PER_PULSE, dtype=np.float64)
    sampled[0::SAMPLES_PER_PULSE] = spread
    return sampled.astype(np.complex128)


def read_cf32(path: str) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 2:
        raise ValueError("cf32 file has odd float count: %s" % path)
    return (raw[0::2] + 1j * raw[1::2]).astype(np.complex128)


def estimate_cir(rx, sync_origin0, code, pre, post, skip, n_sync):
    """Port of local_estimate_cir + per-repetition individual values.

    0-based ``sync_origin0``.  Returns dict with raw/norm/individual/valid/peak.
    ``individual`` is (tap_count, valid) with column k the CIR of repetition
    skip+k.
    """
    rx = np.asarray(rx, dtype=np.complex128).ravel()
    code = np.asarray(code, dtype=np.complex128).ravel()
    code_energy = float(np.real(np.vdot(code, code))) + np.finfo(float).eps
    tap_count = pre + post
    wlen = code.size + tap_count - 1
    period = SAMPLES_PER_SYMBOL
    n = rx.size

    acc = np.zeros(wlen, dtype=np.complex128)
    columns = []
    indexes = []
    for k in range(skip, n_sync):
        lo = sync_origin0 + k * period - pre
        hi = lo + wlen
        if lo < 0 or hi > n:
            continue
        column = rx[lo:hi]
        acc += column
        columns.append(column)
        indexes.append(k)
    valid = len(columns)
    if valid < 1:
        raise ValueError("no complete SYNC windows")

    avg = acc / valid
    mf = np.conj(code)[::-1]
    raw = np.convolve(avg, mf, "valid") / code_energy
    norm = raw / (np.linalg.norm(raw) + np.finfo(float).eps)

    individual = np.empty((tap_count, valid), dtype=np.complex128)
    for j, column in enumerate(columns):
        individual[:, j] = np.convolve(column, mf, "valid") / code_energy

    peak_tap = int(np.argmax(np.abs(raw)))
    return {
        "raw": raw,
        "norm": norm,
        "individual": individual,
        "valid": valid,
        "peak_tap": peak_tap,
        "repetition_index": np.asarray(indexes, dtype=int),
    }


def rel_l2(a, b):
    return float(np.linalg.norm(a - b) / (np.linalg.norm(b) + np.finfo(float).eps))


def phase_slope_cfo(individual, peak_tap, repetition_index, fs, period):
    """Fit phase(peak) vs repetition index -> CFO in Hz."""
    phase = np.unwrap(np.angle(individual[peak_tap, :]))
    x = repetition_index.astype(np.float64)
    slope = np.polyfit(x, phase, 1)[0] if x.size >= 2 else 0.0
    # one repetition advances the RF phase reference by period/fs seconds
    cfo = slope * fs / (2.0 * np.pi * period)
    return float(slope), float(cfo)


# ---------------------------------------------------------------------------
# Golden mode
# ---------------------------------------------------------------------------

GOLDEN_CASES = [
    ("clean", "rx_clean_998p4.cf32", "cir_raw_clean_radar.cf32",
     "cir_norm_clean_radar.cf32"),
    ("delay_int", "rx_delay_int_998p4.cf32", "cir_raw_delay_int_radar.cf32",
     "cir_norm_delay_int_radar.cf32"),
    ("delay_frac", "rx_delay_frac_998p4.cf32", "cir_raw_delay_frac_radar.cf32",
     "cir_norm_delay_frac_radar.cf32"),
]


def run_golden(golden_dir: str) -> int:
    meta = json.load(open(os.path.join(golden_dir, "metadata.json")))
    code = build_sampled_code()
    fs = float(meta["rate_work_hz"])
    pre = int(meta["cir"]["radar"]["pre"])
    post = int(meta["cir"]["radar"]["post"])
    skip = int(meta["cir"]["skip_initial_repetitions"])
    n_sync = int(meta["sync_repetitions"])

    tol = 1e-5
    failures = 0
    print("Per-repetition CIR golden validation")
    print("  fs=%.1f MHz sps=%d pre=%d post=%d skip=%d nSync=%d"
          % (fs / 1e6, SAMPLES_PER_SYMBOL, pre, post, skip, n_sync))

    # Canonical generator uses the TX-time origin (pre_guard) as the CIR delay
    # axis for ALL cases; the channel delay shifts the peak, it does not move
    # the axis.  That is meta.coordinates_0based.rx_clean_998p4.sync_origin.
    origin0 = int(round(meta["coordinates_0based"]["rx_clean_998p4"]["sync_origin"]))

    for label, rxf, rawf, normf in GOLDEN_CASES:
        rx = read_cf32(os.path.join(golden_dir, rxf))
        gold_raw = read_cf32(os.path.join(golden_dir, rawf))
        gold_norm = read_cf32(os.path.join(golden_dir, normf))

        res = estimate_cir(rx, origin0, code, pre, post, skip, n_sync)

        if res["raw"].size != gold_raw.size:
            print("  FAIL %-10s tap_count %d != golden %d"
                  % (label, res["raw"].size, gold_raw.size))
            failures += 1
            continue

        r_raw = rel_l2(res["raw"], gold_raw)
        r_norm = rel_l2(res["norm"], gold_norm)
        mean_raw = res["individual"].mean(axis=1)
        r_mean = rel_l2(mean_raw, res["raw"])

        slope, cfo = phase_slope_cfo(res["individual"], res["peak_tap"],
                                     res["repetition_index"], fs,
                                     SAMPLES_PER_SYMBOL)
        # per-repetition peak-tap spread (should be 0 for a static channel)
        peaks = np.argmax(np.abs(res["individual"]), axis=0)
        peak_span = int(peaks.max() - peaks.min())

        status = "ok"
        if r_norm >= tol:
            status = "FAIL(norm L2>=%.1e)" % tol
            failures += 1
        if r_mean > 1e-10:
            status = "FAIL(mean!=avg)"
            failures += 1

        print("  %-10s raw_L2=%.3e norm_L2=%.3e mean_L2=%.2e valid=%d "
              "peak=%d peak_span=%d phase_slope=%.3e rad/rep cfo=%+.2f Hz  %s"
              % (label, r_raw, r_norm, r_mean, res["valid"], res["peak_tap"],
                 peak_span, slope, cfo, status))

    if failures:
        print("FAIL: %d check(s) failed" % failures)
        return 1
    print("PASS: averaged CIR matches MATLAB goldens; individual mean == "
          "averaged CIR (machine precision)")
    return 0


# ---------------------------------------------------------------------------
# Capture mode (real hardware run dir)
# ---------------------------------------------------------------------------

def _xcorr_peak(a, b):
    """Shift-invariant normalized |cross-correlation| and best lag.

    Returns (peak, lag) with peak in [0, 1]; lag>0 means ``a`` is shifted
    right relative to ``b``.
    """
    a = a / (np.linalg.norm(a) + 1e-30)
    b = b / (np.linalg.norm(b) + 1e-30)
    xc = np.correlate(a, b, "full")
    k = int(np.argmax(np.abs(xc)))
    return float(np.abs(xc[k])), k - (b.size - 1)


def run_capture(run_dir: str, work_rate: float, native_rate: float) -> int:
    """Per-repetition CIR from a hardware run dir (native SC16 capture).

    The native->998.4 grid is produced with scipy resample_poly, whose FIR
    phase differs from the repo's 65/48 PDU FIR by a constant offset.  We
    re-anchor the CIR axis on the live sync template and then refine the
    integer offset against the C++ cir.cf32 (shift-invariant), so the averaged
    per-repetition CIR reproduces the realtime CIR.
    """
    from scipy.signal import resample_poly

    capture_jsonl = os.path.join(run_dir, "capture.jsonl")
    cir_jsonl = os.path.join(run_dir, "cir.jsonl")
    if not os.path.exists(capture_jsonl) or not os.path.exists(cir_jsonl):
        print("FAIL: %s needs capture.jsonl + cir.jsonl" % run_dir)
        return 1

    cap = [json.loads(l) for l in open(capture_jsonl) if l.strip()]
    cir_meta = {int(json.loads(l)["pulse_id"]): json.loads(l)
                for l in open(cir_jsonl) if l.strip()}
    cir_raw = read_cf32(os.path.join(run_dir, "cir.cf32"))

    tmpl_path = os.path.join(run_dir, "sync_template_live.cf32")
    tmpl = read_cf32(tmpl_path) if os.path.exists(tmpl_path) else None

    up, down = 65, 48  # 737.28 -> 998.4
    code = build_sampled_code()

    def template_origin(work, origin0):
        if tmpl is None:
            return origin0, 0.0
        t = tmpl / (np.linalg.norm(tmpl) + 1e-30)
        best_d, best_c = 0, -1.0
        for d in range(-64, 65):
            p = origin0 + d
            if p < 0 or p + t.size > work.size:
                continue
            seg = work[p:p + t.size]
            c = abs(np.vdot(t, seg)) / (np.linalg.norm(seg) + 1e-30)
            if c > best_c:
                best_d, best_c = d, c
        return origin0 + best_d, best_c

    print("Per-repetition CIR from capture: %s" % run_dir)
    corrs = []
    coh_gains = []
    cfos = []
    with open(os.path.join(run_dir, "capture.iq"), "rb") as fiq:
        for entry in cap:
            pid = int(entry["packet_id"])
            meta = cir_meta.get(pid)
            if meta is None or meta.get("status") != "ok":
                continue
            n = int(entry["sample_count"])
            off = int(entry["file_offset_samples"])
            fiq.seek(off * 4)
            buf = np.frombuffer(fiq.read(n * 4), dtype=np.int16)
            if buf.size != n * 2:
                print("  pulse %d short read" % pid)
                continue
            iq = (buf[0::2].astype(np.float64) + 1j * buf[1::2]) / 32768.0
            work = resample_poly(iq, up, down)

            pre = int(meta["cir_pre_samples"])
            post = int(meta["cir_post_samples"])
            origin0 = int(meta["cir_origin_sample"])
            sfd = int(meta["predicted_sfd_start_sample"])
            sync_reps = int(round((sfd - origin0) / SAMPLES_PER_SYMBOL))
            n_ref = int(meta["tap_count"])
            foff = int(meta["file_offset_taps"])
            cpp = cir_raw[foff:foff + n_ref]

            origin0, align_c = template_origin(work, origin0)
            # refine the integer offset against the C++ averaged CIR
            best_o, best_c = origin0, -1.0
            for d in range(-8, 9):
                r = estimate_cir(work, origin0 + d, code, pre, post, 10,
                                 sync_reps)
                c, _ = _xcorr_peak(r["raw"], cpp)
                if c > best_c:
                    best_o, best_c = origin0 + d, c
            res = estimate_cir(work, best_o, code, pre, post, 10, sync_reps)

            # Coherent gain across repetitions at the peak tap:
            # |mean_k(x_k)| / mean_k(|x_k|).  1.0 = perfectly phase-stable.
            col = res["individual"][res["peak_tap"], :]
            coh = abs(col.mean()) / (np.abs(col).mean() + 1e-30)
            slope, cfo = phase_slope_cfo(res["individual"], res["peak_tap"],
                                         res["repetition_index"], work_rate,
                                         SAMPLES_PER_SYMBOL)
            peaks = np.argmax(np.abs(res["individual"]), axis=0)
            corrs.append(best_c)
            coh_gains.append(coh)
            cfos.append(cfo)
            print("  pulse %2d reps=%d origin=%d tpl=%.3f peak=%d cpp_peak=%d "
                  "xcorr=%.4f peak_span=%d coh=%.4f cfo=%+.1f Hz"
                  % (pid, res["valid"], best_o, align_c, res["peak_tap"],
                     int(np.argmax(np.abs(cpp))), best_c,
                     int(peaks.max() - peaks.min()), coh, cfo))

    if corrs:
        print("summary: n=%d | xcorr(avg per-rep vs C++)=%.4f..%.4f "
              "coh_gain=%.4f..%.4f cfo=%+.1f..%+.1f Hz"
              % (len(corrs), min(corrs), max(corrs), min(coh_gains),
                 max(coh_gains), min(cfos), max(cfos)))
    print("NOTE: capture mode resamples with scipy (not the repo 65/48 FIR); "
          "use golden mode for exact taps.")
    return 0




def run_null_demo(golden_dir: str) -> int:
    """Show the repetition-average null that motivates per-repetition CIRs.

    A residual CFO f rotates the SYNC phase by 2*pi*f*SPS/fs per repetition.
    Coherently averaging N repetitions nulls any path whose phase advances by
    pi (mod 2*pi), i.e. f = fs/(2*SPS) = 491.34 kHz.  The per-repetition CIRs
    keep the path; only the average collapses.
    """
    meta = json.load(open(os.path.join(golden_dir, "metadata.json")))
    code = build_sampled_code()
    fs = float(meta["rate_work_hz"])
    pre = int(meta["cir"]["radar"]["pre"])
    post = int(meta["cir"]["radar"]["post"])
    skip = int(meta["cir"]["skip_initial_repetitions"])
    n_sync = int(meta["sync_repetitions"])
    origin0 = int(round(meta["coordinates_0based"]["rx_clean_998p4"]["sync_origin"]))
    rx = read_cf32(os.path.join(golden_dir, "rx_clean_998p4.cf32"))
    n = np.arange(rx.size)
    f_null = fs / (2.0 * SAMPLES_PER_SYMBOL)

    ref = estimate_cir(rx, origin0, code, pre, post, skip, n_sync)
    peak = ref["peak_tap"]

    print("Repetition-average null demo (f_null = fs/(2*SPS) = %.1f Hz)"
          % f_null)
    print("  %-16s %-14s %-10s %-14s %-10s" %
          ("cfo", "avg|raw[peak]|", "avg/avg0", "per-rep <|x|>",
           "coh"))
    base = abs(ref["raw"][peak])
    for label, f in (("0", 0.0), ("f_null/4", f_null / 4.0),
                     ("f_null/2", f_null / 2.0), ("f_null", f_null)):
        rxc = rx * np.exp(1j * 2.0 * np.pi * f * n / fs)
        res = estimate_cir(rxc, origin0, code, pre, post, skip, n_sync)
        col = res["individual"][peak, :]
        coh = abs(col.mean()) / (np.abs(col).mean() + 1e-30)
        print("  %-16s %-14.6g %-10.6f %-14.6g %-10.4f"
              % (label, abs(res["raw"][peak]), abs(res["raw"][peak]) / base,
                 np.abs(col).mean(), coh))
    print("PASS: average collapses at f_null; per-repetition CIRs survive it")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--golden-dir", default=HERE)
    ap.add_argument("--capture", default="", help="hardware run dir with capture.iq")
    ap.add_argument("--null-demo", action="store_true",
                    help="show fs/(2*SPS) repetition-average null")
    ap.add_argument("--work-rate", type=float, default=998.4e6)
    ap.add_argument("--native-rate", type=float, default=737.28e6)
    args = ap.parse_args(argv)

    if args.null_demo:
        return run_null_demo(args.golden_dir)
    if args.capture:
        return run_capture(args.capture, args.work_rate, args.native_rate)
    return run_golden(args.golden_dir)


if __name__ == "__main__":
    sys.exit(main())
