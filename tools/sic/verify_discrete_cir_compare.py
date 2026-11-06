#!/usr/bin/env python3
"""Independent offline comparison of the full-packet SIC reconstruction backends.

Old backend: ``--reconstruction full`` (the archived path_fractional reference).
New backend: ``--reconstruction discrete_cir`` (direct discrete FIR estimate).

Nothing here imports the C++ implementation.  CRC16, dumped-IQ checks and the
suppression/CIR/weak-target metrics are re-derived from the public case files
and the offline sidecars.

Gate scope: FINITE-SAMPLE functional admission on identical source windows, not
a general decode-rate proof.  Model IDs, packet origins ``q`` and CFO values are
estimated independently by each backend and are deliberately not required to
match.  Failures are reported per cell and per metric; nothing is averaged away.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

SCHEMA = "sic-discrete-cir-comparison/1"
STAGES = ("estimate", "comm_demod", "validate", "tx", "channel", "subtract",
          "copy", "sic_total", "cir", "sensing_demod", "service")
DISCRETE_TIMES = ("detect", "cfo", "support", "projection", "solve", "fit_check",
                  "estimate_total", "validate", "tx", "scale", "convolution",
                  "cfo_rot", "subtract")
ESTIMATE_CHILDREN = ("detect", "cfo", "support", "projection", "solve", "fit_check")
RECON_CHILDREN = ("scale", "convolution", "cfo_rot")
RECON_PHASES = ("validate", "tx", "subtract")
REJECT_REASONS = {"queue_full", "source_late", "cancelled", "not_submitted"}
TOL_US = 2.0
# Contract constants from docs/sic/DiscreteCIR_接口合同.md.
MAX_TAPS = 256
SUPPORT_TOLERANCE_DB = 1.0
DISCLAIMER = ("finite-sample functional gate on the declared source windows; "
              "it is not a proof of a general decode rate. Model IDs, q and CFO "
              "are independently estimated and are not required to match.")


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------
def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True,
                                    allow_nan=False) + "\n")


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def valid_fcs(text):
    if not isinstance(text, str) or not text:
        return False
    try:
        data = bytes.fromhex(text)
    except (ValueError, TypeError):
        return False
    return len(data) >= 2 and crc16(data[:-2]) == int.from_bytes(data[-2:], "little")


def is_int(value):
    return type(value) is int


def as_uint64(value):
    """Accept a JSON integer or a decimal string carrying a uint64 id."""
    if type(value) is int and 0 <= value < 2 ** 64:
        return value
    if isinstance(value, str) and value.isdigit():
        parsed = int(value)
        if 0 <= parsed < 2 ** 64:
            return parsed
    return None


def is_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def bounds(value):
    if isinstance(value, dict):
        return (int(value.get("lo", value.get("begin", 0))),
                int(value.get("hi", value.get("end", 0))))
    return int(value[0]), int(value[1])


def read_iq(path):
    path = Path(path)
    if path.stat().st_size % 8:
        raise ValueError(f"unaligned CF32: {path}")
    data = np.fromfile(path, dtype="<c8")
    if not np.isfinite(data).all():
        raise ValueError(f"nonfinite CF32: {path}")
    return data


def energy(values):
    values = np.asarray(values, dtype=np.complex128)
    return float(np.vdot(values, values).real)


def db_ratio(numerator, denominator):
    if numerator <= 0 or denominator <= 0:
        return None
    return 10.0 * math.log10(numerator / denominator)


def source_index(value):
    if is_int(value):
        return value
    text = str(value)
    if text.isdigit():
        return int(text)
    name = Path(text).name
    if name.startswith("w") and name[1:].isdigit():
        return int(name[1:])
    raise ValueError(f"invalid source_window: {value}")


def source_dir(case, source):
    return Path(case) / "windows" / f"w{source_index(source):06d}"


def correlate_valid(x, y):
    """Linear cross-correlation, ``mode='valid'`` semantics, via numpy FFT.

    Equivalent to scipy.signal.correlate(x, y, mode='valid') for complex input
    without importing scipy.
    """
    x = np.asarray(x, dtype=np.complex128)
    y = np.asarray(y, dtype=np.complex128)
    if len(x) < len(y) or len(y) == 0:
        return np.zeros(0, dtype=np.complex128)
    size = 1
    while size < len(x) + len(y) - 1:
        size <<= 1
    spectrum = np.fft.fft(x, size) * np.conj(np.fft.fft(y, size))
    full = np.fft.ifft(spectrum)
    return full[:len(x) - len(y) + 1]


def cir_estimate(iq, sync, origin, n_reps=64):
    """Independent matched-filter CIR: reps 9..62, taps -8..64.

    Diagnostic is relative to a clean reference under identical filtering; the
    taps are not claimed to be physical channel gains.
    """
    sps = len(sync)
    if sps <= 0:
        return None
    low = max(0, origin + 9 * sps - 8)
    high = min(len(iq), origin + 63 * sps + 65)
    if high - low < sps:
        return None
    corr = correlate_valid(iq[low:high], sync)
    reference_energy = max(energy(sync), 1e-30)
    taps = []
    for delay in range(-8, 65):
        indices = np.array([origin + r * sps + delay - low
                            for r in range(9, min(n_reps - 1, 63))])
        indices = indices[(indices >= 0) & (indices < len(corr))]
        taps.append(np.mean(corr[indices]) / reference_energy if len(indices) else 0j)
    return np.asarray(taps)


def window_effects(raw, clean, output, public, sidecar, sync):
    """Suppression by region plus independent CIR/weak-target diagnostics."""
    n = len(raw)
    regions = {"all": (0, n),
               "target": (int(public["target_lo"]), int(public["target_hi"]))}
    for name, span in (sidecar.get("fields") or {}).items():
        try:
            low, high = bounds(span)
        except (TypeError, KeyError, IndexError, ValueError):
            continue
        regions["comm_" + str(name).lower()] = (max(0, low), min(n, high))
    region_metrics = {}
    for name, (low, high) in regions.items():
        low, high = max(0, int(low)), min(n, int(high))
        if high <= low:
            continue
        before = raw[low:high] - clean[low:high]
        after = output[low:high] - clean[low:high]
        before_energy, after_energy = energy(before), energy(after)
        region_metrics[name] = {
            "bounds": [low, high],
            "input_interference_energy": before_energy,
            "residual_energy": after_energy,
            "suppression_db": db_ratio(before_energy, after_energy),
        }
    origin = int(sidecar.get("sensing_start",
                             public.get("sensing_seed", public["target_lo"])))
    clean_cir = cir_estimate(clean, sync, origin)
    result = {"regions": region_metrics,
              "target_suppression_db": region_metrics.get(
                  "target", {}).get("suppression_db")}
    if clean_cir is None:
        result["cir"] = None
        return result
    raw_cir = cir_estimate(raw, sync, origin)
    out_cir = cir_estimate(output, sync, origin)
    raw_error = energy(raw_cir - clean_cir)
    out_error = energy(out_cir - clean_cir)
    delay = int(sidecar.get("target_delay_samples", 37))
    tap = max(0, min(len(clean_cir) - 1, delay + 8))
    denominator = max(abs(clean_cir[tap]), 1e-30)
    amp_error = 20.0 * math.log10(max(abs(out_cir[tap]), 1e-30) / denominator)
    raw_amp_error = 20.0 * math.log10(max(abs(raw_cir[tap]), 1e-30) / denominator)
    result["cir"] = {
        "diagnostic": "matched_filter_reps9_62_taps_minus8_64",
        "raw_error_energy": raw_error,
        "output_error_energy": out_error,
        "recovery_db": db_ratio(raw_error, out_error),
        "relative_error": math.sqrt(out_error / max(energy(clean_cir), 1e-30)),
        "weak_target_tap": int(tap),
        "weak_target_amp_error_db": amp_error,
        "weak_target_abs_error_db": abs(amp_error),
        "weak_target_raw_amp_error_db": raw_amp_error,
    }
    return result


# --------------------------------------------------------------------------
# schema checks
# --------------------------------------------------------------------------
def validate_discrete(obj, row, public, sps):
    errors = []
    rejected = row.get("rejected") is True or row.get("reason") in REJECT_REASONS
    if not isinstance(obj, dict):
        return ["discrete_cir is not an object"]
    required = {"model_id", "q", "cfo_hz", "lag_lo", "tap_count",
                "active_count", "condition", "effective_rank", "fit_residual",
                "waveform_scale", "support", "sub", "times_us"}
    missing = sorted(required - set(obj))
    if missing:
        errors.append("discrete_cir missing fields: " + ",".join(missing))
    model_id = as_uint64(obj.get("model_id"))
    if model_id is None:
        errors.append("discrete_cir.model_id must be an unsigned 64-bit integer")
    q = obj.get("q")
    if not is_int(q):
        errors.append("discrete_cir.q must be an integer")
    elif row.get("committed") is True:
        if not 0 <= q <= public["n"]:
            errors.append("discrete_cir.q outside the public context")
    elif not -1 <= q <= public["n"]:
        errors.append("discrete_cir.q invalid")
    cfo = obj.get("cfo_hz")
    if not is_number(cfo):
        errors.append("discrete_cir.cfo_hz must be finite")
    elif abs(cfo) > 0.5 * float(public.get("sample_rate", 998400000.0)) + 1e-6:
        errors.append("discrete_cir.cfo_hz outside Nyquist")
    lag_lo = obj.get("lag_lo")
    if not is_int(lag_lo) or not -int(sps) <= lag_lo <= int(public["n"]):
        errors.append("discrete_cir.lag_lo out of range")
    taps = obj.get("tap_count")
    if not is_int(taps) or not 0 <= taps <= min(MAX_TAPS, int(sps) - 1):
        errors.append("discrete_cir.tap_count outside the finite support contract")
    elif row.get("model_valid") is True and taps < 1:
        errors.append("valid discrete model has an empty continuous hull")
    active = obj.get("active_count")
    if not is_int(active) or active < 0 or (is_int(taps) and active > taps):
        errors.append("discrete_cir.active_count outside [0,tap_count]")
    elif row.get("committed") is True and active < 1:
        errors.append("committed discrete model has no active tap")
    elif row.get("model_valid") is True and active < 1:
        errors.append("valid discrete model has no active tap")
    condition = obj.get("condition")
    if not is_number(condition) or condition < 0:
        errors.append("discrete_cir.condition must be finite and nonnegative")
    elif row.get("committed") is True and condition <= 0:
        errors.append("committed discrete model has a nonpositive condition")
    rank = obj.get("effective_rank")
    if not is_int(rank) or rank < 0 or (is_int(taps) and rank > taps):
        errors.append("discrete_cir.effective_rank outside [0,tap_count]")
    if row.get("committed") is True and rank == 0:
        errors.append("committed discrete model has zero effective rank")
    residual = obj.get("fit_residual")
    if not is_number(residual) or residual < 0:
        errors.append("discrete_cir.fit_residual must be finite and nonnegative")
    scale = obj.get("waveform_scale")
    if not is_number(scale):
        errors.append("discrete_cir.waveform_scale must be finite")
    elif row.get("committed") is True and scale <= 0:
        errors.append("committed discrete model has nonpositive waveform_scale")
    try:
        support_lo, support_hi = bounds(obj["support"])
        sub_lo, sub_hi = bounds(obj["sub"])
        row_lo, row_hi = bounds(row["support"])
        if not 0 <= support_lo <= support_hi <= public["n"]:
            errors.append("discrete_cir.support outside the public context")
        if not 0 <= sub_lo <= sub_hi <= public["n"]:
            errors.append("discrete_cir.sub outside the public context")
        if sub_lo < support_lo or sub_hi > support_hi:
            errors.append("discrete_cir.sub outside the convolution support")
        if sub_lo < row_lo or sub_hi > row_hi:
            errors.append("discrete_cir.sub outside the row support")
        if row.get("committed") is True and sub_hi <= sub_lo:
            errors.append("committed discrete model has an empty sub interval")
    except (KeyError, TypeError, ValueError, IndexError):
        errors.append("discrete_cir support/sub missing or invalid")
    detail = obj.get("times_us")
    if not isinstance(detail, dict):
        errors.append("discrete_cir.times_us missing")
        return errors
    missing_times = [k for k in DISCRETE_TIMES if k not in detail]
    if missing_times:
        errors.append("discrete_cir.times_us missing: " + ",".join(missing_times))
    if not all(is_number(detail.get(k)) and detail[k] >= 0 for k in DISCRETE_TIMES):
        errors.append("discrete_cir.times_us must be finite and nonnegative")
        return errors
    row_times = row.get("times_us", {}) if isinstance(row.get("times_us"), dict) else {}
    estimate_children = sum(detail[k] for k in ESTIMATE_CHILDREN)
    if estimate_children > detail["estimate_total"] + TOL_US:
        errors.append("discrete estimate children exceed estimate_total")
    if detail["estimate_total"] > row_times.get("estimate", 0.0) + TOL_US:
        errors.append("discrete estimate_total exceeds the row estimate parent")
    recon_children = sum(detail[k] for k in RECON_CHILDREN)
    if recon_children > row_times.get("channel", 0.0) + TOL_US:
        errors.append("discrete reconstruction children exceed the row channel parent")
    for name in RECON_PHASES:
        if detail[name] > row_times.get(name, 0.0) + TOL_US:
            errors.append(f"discrete {name} exceeds the row parent")
    if rejected and row.get("committed") is True:
        errors.append("rejected row is committed")
    return errors


def validate_record(row, public, side, sps):
    errors = []

    def check(ok, why):
        if not ok:
            errors.append(why)

    if not isinstance(row, dict):
        return ["row is not an object"]
    rejected = row.get("rejected") is True or row.get("reason") in REJECT_REASONS
    check(row.get("mode") == "full", "mode is not full")
    check(is_int(row.get("window_id")) and row["window_id"] >= 0, "window_id")
    try:
        source_index(row.get("source_window"))
    except (ValueError, TypeError):
        errors.append("source_window")
    check(is_int(row.get("n")) and row.get("n") == public["n"], "n mismatch")
    check(row.get("observed_end") == public["observed_end"], "observed_end mismatch")
    expected = "full" if side == "old" else "discrete_cir"
    check(row.get("reconstruction") == expected, f"reconstruction is not {expected}")
    check(type(row.get("committed")) is bool, "committed type")
    check(type(row.get("model_valid")) is bool, "model_valid type")
    committed = row.get("committed") is True
    model_valid = row.get("model_valid") is True
    check(not committed or model_valid, "committed without a valid model")
    try:
        low, high = bounds(row["support"])
        check(0 <= low <= high <= public["n"], "support outside the public context")
        check(not committed or high > low, "committed row has an empty support")
    except (KeyError, TypeError, ValueError, IndexError):
        errors.append("support missing/invalid")
    check(type(row.get("comm_fcs")) is bool, "comm_fcs type")
    check(not committed or row.get("comm_fcs") is True, "full commit without comm FCS")
    if row.get("comm_fcs") is True:
        check(valid_fcs(row.get("comm_psdu_hex")), "invalid reported comm FCS")
    check(type(row.get("sensing_fcs")) is bool, "sensing_fcs type")
    if row.get("sensing_fcs") is True:
        check(valid_fcs(row.get("sensing_psdu_hex")), "invalid reported sensing FCS")
    times = row.get("times_us")
    if not isinstance(times, dict):
        errors.append("times_us missing")
    else:
        for key in STAGES:
            if rejected and key not in times:
                continue
            check(is_number(times.get(key)) and times[key] >= 0,
                  "time " + key)
        if not rejected and all(is_number(times.get(k)) for k in STAGES):
            children = sum(times[k] for k in ("estimate", "comm_demod", "validate",
                                              "tx", "channel", "subtract", "copy"))
            check(abs(times["sic_total"] - children) <= TOL_US,
                  "SIC stage partition")
            check(abs(times["service"] - times["sic_total"] - times["cir"]
                      - times["sensing_demod"]) <= TOL_US, "service stage partition")
        for field in ("thread_cpu", "thread_cpu_us"):
            if field in times:
                check(is_number(times[field]) and times[field] >= 0,
                      "thread CPU time")
    events = row.get("events")
    if not isinstance(events, dict):
        errors.append("events missing")
    else:
        try:
            values = [float(events[k]) for k in
                      ("submit_us", "start_us", "finish_us", "result_us")]
            check(all(math.isfinite(v) and v >= 0 for v in values),
                  "invalid event time")
            check(values == sorted(values), "event order")
            sample = events["required_sample"]
            check(is_number(sample) and 0 <= sample <= public["n"], "required sample")
            if not rejected:
                check(sample == public["n"], "full context observation policy")
                if "first_clean_us" in events and "sample_ready_us" in events:
                    check(events["first_clean_us"] + 1e-6 >= events["sample_ready_us"],
                          "early clean before samples")
        except (KeyError, TypeError, ValueError):
            errors.append("events missing/invalid")
    if side == "old":
        if isinstance(row.get("discrete_cir"), dict):
            errors.append("old (full) row unexpectedly carries discrete_cir")
    elif not rejected and "discrete_cir" not in row:
        errors.append("discrete_cir object missing")
    elif "discrete_cir" in row:
        errors += validate_discrete(row["discrete_cir"], row, public, sps)
    return errors


# --------------------------------------------------------------------------
# loading and comparison
# --------------------------------------------------------------------------
def load_case(case):
    case = Path(case).resolve()
    public = json.loads((case / "public.json").read_text())
    windows = sorted(p for p in (case / "windows").iterdir() if p.is_dir())
    if not windows:
        raise ValueError(f"no source windows in {case}")
    hashes = {}
    for window in windows:
        for name in ("raw.cf32", "clean.cf32", "sidecar.json"):
            if not (window / name).is_file():
                raise ValueError(f"incomplete source window {window}: missing {name}")
        if (window / "raw.cf32").stat().st_size != int(public["n"]) * 8:
            raise ValueError(f"raw size mismatch in {window}")
        hashes[window.name] = sha256(window / "raw.cf32")
    templates = {}
    for name in ("comm_sync.cf32", "comm_preamble.cf32", "sensing_sync.cf32"):
        path = case / name
        if not path.is_file():
            raise ValueError(f"missing public template {name}")
        templates[name] = sha256(path)
        declared = public.get("template_hashes", {}).get(name)
        if declared is None:
            raise ValueError(f"public.json has no template hash for {name}")
        if declared != templates[name]:
            raise ValueError(f"public template hash mismatch for {name}")
    return dict(path=case, public=public, windows=windows, hashes=hashes,
                templates=templates)


def load_run(run):
    run = Path(run).resolve()
    rows_path = run / "windows.jsonl"
    if not rows_path.is_file():
        raise ValueError(f"missing {rows_path}")
    all_rows = [json.loads(line) for line in rows_path.read_text().splitlines()
                if line.strip()]
    if not all_rows:
        raise ValueError(f"no terminal rows in {run}")
    summary_path = run / "runner_summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.is_file() else None
    return dict(path=run, all_rows=all_rows, summary=summary)


def full_rows(run):
    rows = [row for row in run["all_rows"] if row.get("mode") == "full"]
    invalid = [row for row in run["all_rows"]
               if row.get("mode") not in ("full", "preamble")]
    return rows, invalid


def row_is_rejected(row):
    return row.get("rejected") is True or row.get("reason") in REJECT_REASONS


def aggregate_counts(rows, sidecars):
    counts = {"n": len(rows), "accepted": 0, "committed": 0, "comm_fcs": 0,
              "comm_exact": 0, "sensing_fcs": 0, "sensing_exact": 0,
              "comm_fcs_given_accepted": 0, "sensing_fcs_given_accepted": 0}
    for row in rows:
        if row_is_rejected(row):
            continue
        counts["accepted"] += 1
        try:
            side = sidecars[source_index(row["source_window"])]
        except (KeyError, ValueError, TypeError):
            continue
        comm_exact = (row.get("comm_fcs") is True and
                      row.get("comm_psdu_hex", "").lower() ==
                      str(side.get("comm_psdu_hex", "missing")).lower())
        sense_exact = (row.get("sensing_fcs") is True and
                       row.get("sensing_psdu_hex", "").lower() ==
                       str(side.get("sensing_psdu_hex", "missing")).lower())
        counts["committed"] += row.get("committed") is True
        counts["comm_fcs"] += row.get("comm_fcs") is True
        counts["comm_exact"] += comm_exact
        counts["sensing_fcs"] += row.get("sensing_fcs") is True
        counts["sensing_exact"] += sense_exact
    if counts["accepted"]:
        counts["comm_fcs_given_accepted"] = counts["comm_fcs"] / counts["accepted"]
        counts["sensing_fcs_given_accepted"] = counts["sensing_fcs"] / counts["accepted"]
    return counts


def pooled_suppression(window_metrics, region):
    numerator = denominator = 0.0
    windows = 0
    for metric in window_metrics:
        detail = metric["regions"].get(region)
        if not detail:
            continue
        numerator += detail["input_interference_energy"]
        denominator += detail["residual_energy"]
        windows += 1
    return {"input_interference_energy": numerator,
            "residual_energy": denominator,
            "suppression_db": db_ratio(numerator, denominator),
            "n_windows": windows}


def pooled_cir(window_metrics):
    raw_error = output_error = 0.0
    windows = 0
    for metric in window_metrics:
        cir = metric.get("cir")
        if not cir:
            continue
        raw_error += cir["raw_error_energy"]
        output_error += cir["output_error_energy"]
        windows += 1
    return {"raw_error_energy": raw_error,
            "output_error_energy": output_error,
            "recovery_db": db_ratio(raw_error, output_error),
            "n_windows": windows}


def mean_values(values):
    finite = [float(v) for v in values if v is not None and math.isfinite(v)]
    return sum(finite) / len(finite) if finite else None


def aggregate_metrics(window_metrics):
    regions = sorted({name for metric in window_metrics
                      for name in metric["regions"]})
    suppression = {region: pooled_suppression(window_metrics, region)
                   for region in regions}
    weak_signed = [metric["cir"]["weak_target_amp_error_db"]
                   for metric in window_metrics if metric.get("cir")]
    weak_abs = [metric["cir"]["weak_target_abs_error_db"]
                for metric in window_metrics if metric.get("cir")]
    return {
        "n_windows": len(window_metrics),
        "suppression": suppression,
        "target_suppression_db": suppression.get(
            "target", {}).get("suppression_db"),
        "target_suppression_mean_db": mean_values(
            [metric["regions"].get("target", {}).get("suppression_db")
             for metric in window_metrics]),
        "cir": pooled_cir(window_metrics),
        "cir_recovery_db": pooled_cir(window_metrics)["recovery_db"],
        "cir_recovery_mean_db": mean_values(
            [metric["cir"]["recovery_db"] for metric in window_metrics
             if metric.get("cir")]),
        "weak_target_amp_error_db": mean_values(weak_signed),
        "weak_target_abs_error_db": mean_values(weak_abs),
        "weak_target_abs_error_max_db": (max(weak_abs) if weak_abs else None),
        "weak_target_raw_abs_error_db": mean_values(
            [abs(metric["cir"]["weak_target_raw_amp_error_db"])
             for metric in window_metrics if metric.get("cir")]),
    }


def metric_gate(name, old, new, rule, tolerance, old_perfect=False,
                new_perfect=False):
    if old_perfect or new_perfect:
        if new_perfect and old_perfect:
            detail = "both residuals are exactly zero"
            passed = True
        elif new_perfect:
            detail = "new residual is exactly zero (perfect)"
            passed = True
        else:
            detail = "old residual is exactly zero but new is not"
            passed = False
        return {"name": name, "rule": rule, "old": old, "new": new,
                "tolerance": tolerance, "evaluated": True,
                "delta": None, "limit": None, "passed": passed,
                "detail": detail}
    evaluated = old is not None and new is not None and \
        math.isfinite(float(old)) and math.isfinite(float(new))
    entry = {"name": name, "rule": rule, "old": old, "new": new,
             "tolerance": tolerance, "evaluated": bool(evaluated),
             "delta": (float(new) - float(old)) if evaluated else None}
    if not evaluated:
        entry.update(passed=None, detail="not evaluated (missing finite metric)")
        return entry
    if rule == ">=":
        entry["limit"] = float(old) - float(tolerance)
        entry["passed"] = float(new) >= entry["limit"]
    else:
        entry["limit"] = float(old) + float(tolerance)
        entry["passed"] = float(new) <= entry["limit"]
    return entry


def _perfect_flags(old_aggregate, new_aggregate, kind):
    if kind == "suppression":
        old_detail = (old_aggregate.get("suppression") or {}).get("target") or {}
        new_detail = (new_aggregate.get("suppression") or {}).get("target") or {}
        old_input = old_detail.get("input_interference_energy", 0)
        new_input = new_detail.get("input_interference_energy", 0)
        return {"old_perfect": bool(old_input > 0 and
                                    old_detail.get("residual_energy") == 0),
                "new_perfect": bool(new_input > 0 and
                                    new_detail.get("residual_energy") == 0)}
    old_cir = old_aggregate.get("cir") or {}
    new_cir = new_aggregate.get("cir") or {}
    return {"old_perfect": bool(old_cir.get("raw_error_energy", 0) > 0 and
                                old_cir.get("output_error_energy") == 0),
            "new_perfect": bool(new_cir.get("raw_error_energy", 0) > 0 and
                                new_cir.get("output_error_energy") == 0)}


def count_gate(name, old, new):
    entry = {"name": name, "rule": ">=", "old": int(old), "new": int(new),
             "tolerance": 0, "evaluated": True,
             "delta": int(new) - int(old), "limit": int(old),
             "passed": int(new) >= int(old)}
    return entry


def compare(case, old_run, new_run, require_provenance=False, summary_only=False):
    case_info = load_case(case)
    public = case_info["public"]
    sps = len(read_iq(case_info["path"] / "sensing_sync.cf32"))
    old = load_run(old_run)
    new = load_run(new_run)
    errors, warnings = [], []
    old_rows, old_invalid = full_rows(old)
    new_rows, new_invalid = full_rows(new)
    if old_invalid:
        errors.append("old run has terminal rows with unsupported modes")
    if new_invalid:
        errors.append("new run has terminal rows with unsupported modes")
    if any(row.get("mode") == "preamble" for row in new["all_rows"]):
        errors.append("new run must contain full-mode rows only")
    if not old_rows:
        errors.append("old run has no full-mode rows")
    if not new_rows:
        errors.append("new run has no full-mode rows")

    sidecars = {}
    for window in case_info["windows"]:
        sidecars[int(window.name[1:])] = json.loads(
            (window / "sidecar.json").read_text())

    provenance_errors = []
    expected_sources = sorted(sidecars)
    for side, run, rows in (("old", old, old_rows), ("new", new, new_rows)):
        for position, row in enumerate(rows):
            label = f"{side} row {position} (window {row.get('window_id')})"
            errors.extend(f"{label}: {message}"
                          for message in validate_record(row, public, side, sps))
        keys = [(row.get("window_id"), row.get("mode")) for row in rows]
        if len(keys) != len(set(keys)):
            errors.append(f"{side} run has duplicate terminal rows")
        sources = []
        for row in rows:
            try:
                sources.append(source_index(row.get("source_window")))
            except (ValueError, TypeError):
                continue
        if summary_only:
            if not sources or not set(sources) <= set(expected_sources):
                errors.append(f"{side} run references sources outside the case")
        elif set(sources) != set(expected_sources):
            errors.append(f"{side} run does not cover the case source windows")
        if not summary_only:
            if sorted(sources) != sources:
                errors.append(f"{side} run source windows are not in case order")
            if len(sources) != len(set(sources)):
                errors.append(f"{side} run repeats source windows; "
                              "a functional comparison needs one row per source")
        summary = run["summary"]
        if summary is not None:
            submitted = summary.get("submitted")
            completed = summary.get("completed")
            rejected_count = summary.get("rejected")
            if not all(is_int(v) and v >= 0
                       for v in (submitted, completed, rejected_count)):
                errors.append(f"{side} runner summary count types")
            elif submitted != len(run["all_rows"]):
                errors.append(f"{side} runner summary submitted/row mismatch")
            elif submitted != completed + rejected_count:
                errors.append(f"{side} runner summary terminal accounting")
            expected_recon = "full" if side == "old" else "discrete_cir"
            if summary.get("reconstruction") != expected_recon:
                errors.append(f"{side} runner summary reconstruction mismatch")
        else:
            warnings.append(f"{side} runner_summary.json absent")
        if require_provenance:
            for row in rows:
                source = source_index(row.get("source_window"))
                expected = case_info["hashes"][f"w{source:06d}"]
                if row.get("input_hash") != expected:
                    provenance_errors.append(
                        f"{side} row {row.get('window_id')}: recorded input "
                        "hash missing or mismatched")
    for side, run, rows in (("old", old, old_rows), ("new", new, new_rows)):
        if run["summary"] and run["summary"].get("input_pool") not in (None, len(case_info["windows"])):
            warnings.append(f"{side} runner summary input_pool differs from case")
        for row in rows:
            if row.get("output_hash") and not summary_only:
                output = run["path"] / f"w{int(row['window_id']):06d}_full.cf32"
                if not output.is_file():
                    errors.append(f"{side} row {row['window_id']}: declared output missing")
                elif row["output_hash"] != sha256(output):
                    errors.append(f"{side} row {row['window_id']}: output hash mismatch")
    if require_provenance:
        for window in case_info["windows"]:
            sidecar = json.loads((window / "sidecar.json").read_text())
            for name, expected in (sidecar.get("hashes") or {}).items():
                path = window / name
                if path.is_file() and sha256(path) != expected:
                    provenance_errors.append(
                        f"sidecar hash mismatch: {window.name}/{name}")
    errors.extend(provenance_errors)
    old_rx = {(row.get("rx_pipeline"), row.get("rx_avx2")) for row in old_rows
              if "rx_pipeline" in row or "rx_avx2" in row}
    new_rx = {(row.get("rx_pipeline"), row.get("rx_avx2")) for row in new_rows
              if "rx_pipeline" in row or "rx_avx2" in row}
    if old_rx and new_rx and old_rx != new_rx:
        errors.append("paired runs used different RX pipelines")
    if old_rx and new_rx:
        for row in old_rows + new_rows:
            if type(row.get("rx_avx2")) is not bool:
                errors.append("RX AVX2 metadata type")

    old_by_source = {}
    for row in old_rows:
        if row_is_rejected(row):
            errors.append("old functional run has rejected rows")
            continue
        try:
            source = source_index(row["source_window"])
        except (ValueError, TypeError):
            continue
        old_by_source.setdefault(source, []).append(row)
    new_by_source = {}
    for row in new_rows:
        if row_is_rejected(row):
            errors.append("new functional run has rejected rows")
            continue
        try:
            source = source_index(row["source_window"])
        except (ValueError, TypeError):
            continue
        new_by_source.setdefault(source, []).append(row)
    if not summary_only and any(len(v) > 1 for v in old_by_source.values()):
        errors.append("old run repeats accepted source windows")
    if not summary_only and any(len(v) > 1 for v in new_by_source.values()):
        errors.append("new run repeats accepted source windows")
    paired_sources = sorted((set(old_by_source) & set(new_by_source))
                            & set(expected_sources))
    if summary_only:
        # Timing repeats a finite pool; a short smoke may cover only part of
        # it. Both variants must still describe the same sources.
        if not paired_sources or set(old_by_source) != set(new_by_source) or \
                not set(old_by_source) <= set(expected_sources):
            errors.append("paired source coverage is incomplete")
    elif set(old_by_source) != set(expected_sources) or \
            set(new_by_source) != set(expected_sources):
        errors.append("paired source coverage is incomplete")

    old_metrics, new_metrics, per_window = [], [], []
    for source in paired_sources:
        old_row, new_row = old_by_source[source][0], new_by_source[source][0]
        window = case_info["windows"][source]
        raw = read_iq(window / "raw.cf32")
        clean = read_iq(window / "clean.cf32")
        if len(raw) != int(public["n"]) or len(clean) != int(public["n"]):
            errors.append(f"source window {source}: raw/clean length mismatch")
            continue
        sidecar = sidecars[source]
        sync = read_iq(case_info["path"] / "sensing_sync.cf32")
        if not summary_only:
            outputs = {}
            for side, run, row in (("old", old, old_row), ("new", new, new_row)):
                path = run["path"] / f"w{int(row['window_id']):06d}_full.cf32"
                if not path.is_file():
                    errors.append(f"{side} row {row['window_id']}: dumped output missing")
                    continue
                output = read_iq(path)
                if len(output) != len(raw):
                    errors.append(f"{side} row {row['window_id']}: output size")
                    continue
                outputs[side] = output
                try:
                    low, high = bounds(row["support"])
                except (KeyError, TypeError, ValueError, IndexError):
                    errors.append(f"{side} row {row['window_id']}: "
                                  "support unavailable for invariance check")
                    continue
                same_outside = (output[:low].tobytes() == raw[:low].tobytes() and
                                output[high:].tobytes() == raw[high:].tobytes())
                if not same_outside:
                    errors.append(f"{side} row {row['window_id']}: "
                                  "output changed outside its support")
                if row.get("committed") is not True:
                    if output.tobytes() != raw.tobytes():
                        errors.append(f"{side} row {row['window_id']}: "
                                      "bypass changed IQ")
                elif side == "new":
                    detail = row.get("discrete_cir")
                    try:
                        sub_low, sub_high = bounds(detail["sub"])
                    except (KeyError, TypeError, ValueError, IndexError):
                        errors.append(f"new row {row['window_id']}: "
                                      "discrete_cir.sub unavailable")
                    else:
                        if output[:sub_low].tobytes() != raw[:sub_low].tobytes() or \
                                output[sub_high:].tobytes() != raw[sub_high:].tobytes():
                            errors.append(
                                f"new row {row['window_id']}: output changed "
                                "outside discrete_cir.sub")
            if "old" not in outputs or "new" not in outputs:
                continue
            old_metric = window_effects(raw, clean, outputs["old"], public,
                                        sidecar, sync)
            new_metric = window_effects(raw, clean, outputs["new"], public,
                                        sidecar, sync)
            old_metrics.append(old_metric)
            new_metrics.append(new_metric)
            delta = {
                "target_suppression_db": _delta(
                    new_metric["target_suppression_db"],
                    old_metric["target_suppression_db"]),
                "cir_recovery_db": _delta(
                    (new_metric.get("cir") or {}).get("recovery_db"),
                    (old_metric.get("cir") or {}).get("recovery_db")),
                "weak_target_abs_error_db": _delta(
                    (new_metric.get("cir") or {}).get("weak_target_abs_error_db"),
                    (old_metric.get("cir") or {}).get("weak_target_abs_error_db")),
            }
            per_window.append({
                "window_id": old_row["window_id"],
                "new_window_id": new_row["window_id"],
                "source_window": source,
                "old": old_metric,
                "new": new_metric,
                "delta_new_minus_old": delta,
                "new_discrete": _discrete_summary(new_row.get("discrete_cir")),
            })

    old_counts = aggregate_counts(old_rows, sidecars)
    new_counts = aggregate_counts(new_rows, sidecars)
    for name in ("comm_fcs", "comm_exact", "sensing_fcs", "sensing_exact"):
        if old_counts[name] > old_counts["accepted"] or \
                new_counts[name] > new_counts["accepted"]:
            errors.append(f"count {name} exceeds accepted rows")

    if summary_only:
        old_aggregate = new_aggregate = {}
        gates = []
    else:
        old_aggregate = aggregate_metrics(old_metrics)
        new_aggregate = aggregate_metrics(new_metrics)
        gates = [
            count_gate("comm_fcs", old_counts["comm_fcs"], new_counts["comm_fcs"]),
            count_gate("comm_exact", old_counts["comm_exact"], new_counts["comm_exact"]),
            count_gate("sensing_fcs", old_counts["sensing_fcs"], new_counts["sensing_fcs"]),
            count_gate("sensing_exact", old_counts["sensing_exact"],
                       new_counts["sensing_exact"]),
            metric_gate("target_suppression_db",
                        old_aggregate.get("target_suppression_db"),
                        new_aggregate.get("target_suppression_db"),
                        ">=", SUPPORT_TOLERANCE_DB,
                        **_perfect_flags(old_aggregate, new_aggregate, "suppression")),
            metric_gate("cir_recovery_db",
                        old_aggregate.get("cir_recovery_db"),
                        new_aggregate.get("cir_recovery_db"),
                        ">=", SUPPORT_TOLERANCE_DB,
                        **_perfect_flags(old_aggregate, new_aggregate, "cir")),
            metric_gate("weak_target_abs_error_db",
                        old_aggregate.get("weak_target_abs_error_db"),
                        new_aggregate.get("weak_target_abs_error_db"),
                        "<=", SUPPORT_TOLERANCE_DB),
        ]
    failures = [f"structural: {message}" for message in errors]
    successes = []
    for gate in gates:
        if gate["evaluated"] and gate["passed"]:
            successes.append(gate["name"])
        elif gate["evaluated"] and not gate["passed"]:
            failures.append(gate["name"])
        else:
            failures.append(gate["name"] + " (not evaluated)")
    failed_windows = []
    if not summary_only:
        for entry in per_window:
            delta = entry["delta_new_minus_old"]
            reasons = []
            if delta["target_suppression_db"] is not None and \
                    delta["target_suppression_db"] < -SUPPORT_TOLERANCE_DB:
                reasons.append("target_suppression_db")
            if delta["cir_recovery_db"] is not None and \
                    delta["cir_recovery_db"] < -SUPPORT_TOLERANCE_DB:
                reasons.append("cir_recovery_db")
            if delta["weak_target_abs_error_db"] is not None and \
                    delta["weak_target_abs_error_db"] > SUPPORT_TOLERANCE_DB:
                reasons.append("weak_target_abs_error_db")
            if reasons:
                failed_windows.append({"source_window": entry["source_window"],
                                       "window_id": entry["window_id"],
                                       "metrics": reasons,
                                       "delta": delta})
    structural_pass = not errors
    gate_pass = all(g["passed"] is True for g in gates) if gates else True
    passed = structural_pass and gate_pass and not summary_only
    if summary_only:
        passed = structural_pass
    exact_differences = {
        "comm_fcs": int(new_counts["comm_fcs"] - old_counts["comm_fcs"]),
        "comm_exact": int(new_counts["comm_exact"] - old_counts["comm_exact"]),
        "sensing_fcs": int(new_counts["sensing_fcs"] - old_counts["sensing_fcs"]),
        "sensing_exact": int(new_counts["sensing_exact"] - old_counts["sensing_exact"]),
    }
    if not summary_only:
        old_target = old_aggregate.get("target_suppression_db")
        new_target = new_aggregate.get("target_suppression_db")
        exact_differences["target_suppression_db"] = _delta(new_target, old_target)
        exact_differences["cir_recovery_db"] = _delta(
            new_aggregate.get("cir_recovery_db"),
            old_aggregate.get("cir_recovery_db"))
        exact_differences["weak_target_abs_error_db"] = _delta(
            new_aggregate.get("weak_target_abs_error_db"),
            old_aggregate.get("weak_target_abs_error_db"))
    cell_name = public.get("cell") or case_info["path"].name
    cell = {
        "cell": cell_name,
        "case": str(case_info["path"]),
        "old_run": str(old["path"]),
        "new_run": str(new["path"]),
        "structural": {"passed": structural_pass, "errors": errors,
                       "warnings": warnings},
        "rows": {"old": len(old_rows), "new": len(new_rows),
                 "unique_sources": len(paired_sources),
                 "expected_sources": len(expected_sources)},
        "counts": {"old": old_counts, "new": new_counts},
        "discrete_model": discrete_model_stats(
            [row for row in new_rows if not row_is_rejected(row)]),
        "metrics": {"old": old_aggregate, "new": new_aggregate},
        "gates": gates,
        "per_window": per_window,
        "windows_exceeding_tolerance": failed_windows,
        "successes": successes,
        "failures": failures,
        "exact_differences": exact_differences,
        "passed": passed,
        "notes": [
            "model_id, q and cfo_hz are estimated independently and are not "
            "required to match across backends",
            "counts are over accepted rows; exact requires the PSDU to match "
            "the offline sidecar",
            "suppression and CIR recovery are pooled energy ratios over the "
            "cell's source windows; per-window values are listed separately",
        ],
    }
    return {
        "schema": SCHEMA,
        "created_utc": utc_now(),
        "verification_level": "summary_only" if summary_only else "dumped_iq",
        "iq_effects_verified": not summary_only and not errors,
        "disclaimer": DISCLAIMER,
        "case": str(case_info["path"]),
        "old_run": str(old["path"]),
        "new_run": str(new["path"]),
        "template_hashes": case_info["templates"],
        "require_provenance": bool(require_provenance),
        "provenance_verified": bool(require_provenance and not provenance_errors),
        "cells": [cell],
        "passed": cell["passed"],
        "successes": cell["successes"],
        "failures": cell["failures"],
        "structural_errors": errors,
        "warnings": warnings,
    }


def _discrete_summary(obj):
    if not isinstance(obj, dict):
        return None
    keys = ("model_id", "q", "cfo_hz", "lag_lo", "tap_count", "active_count",
            "condition", "effective_rank", "fit_residual", "waveform_scale")
    result = {key: obj.get(key) for key in keys}
    for name in ("support", "sub"):
        try:
            low, high = bounds(obj[name])
        except (KeyError, TypeError, ValueError, IndexError):
            result[name] = None
            continue
        result[name] = [low, high]
    times = obj.get("times_us")
    if isinstance(times, dict):
        result["times_us"] = {key: times.get(key) for key in DISCRETE_TIMES}
    return result


def discrete_model_stats(rows):
    objects = [_discrete_summary(row.get("discrete_cir")) for row in rows
               if isinstance(row.get("discrete_cir"), dict)]
    if not objects:
        return {"rows": 0}

    def column(name):
        return [obj.get(name) for obj in objects if is_number(obj.get(name))]

    def span(values):
        if not values:
            return None
        return {"min": min(values), "max": max(values), "mean": sum(values) / len(values)}

    support_widths = [obj["support"][1] - obj["support"][0] for obj in objects
                      if obj.get("support")]
    sub_widths = [obj["sub"][1] - obj["sub"][0] for obj in objects
                  if obj.get("sub")]
    return {
        "rows": len(objects),
        "committed": sum(row.get("committed") is True for row in rows),
        "model_ids": sorted({as_uint64(obj.get("model_id")) for obj in objects
                             if as_uint64(obj.get("model_id")) is not None}),
        "lag_lo": span(column("lag_lo")),
        "tap_count": span(column("tap_count")),
        "active_count": span(column("active_count")),
        "condition": span(column("condition")),
        "effective_rank": span(column("effective_rank")),
        "fit_residual": span(column("fit_residual")),
        "waveform_scale": span(column("waveform_scale")),
        "support_width": span(support_widths),
        "sub_width": span(sub_widths),
        "estimate_total_us": span(
            [obj["times_us"]["estimate_total"] for obj in objects
             if isinstance(obj.get("times_us"), dict)
             and is_number(obj["times_us"].get("estimate_total"))]),
    }


def _delta(new, old):
    if not is_number(new) or not is_number(old):
        return None
    return float(new) - float(old)


def write_csv(result, path):
    rows = []
    for cell in result["cells"]:
        for gate in cell["gates"]:
            rows.append({
                "cell": cell["cell"], "scope": "gate", "window_id": "",
                "source_window": "", "metric": gate["name"],
                "old": _cell_text(gate["old"]), "new": _cell_text(gate["new"]),
                "delta": _cell_text(gate["delta"]),
                "tolerance": _cell_text(gate["tolerance"]),
                "passed": "" if gate["passed"] is None else
                ("true" if gate["passed"] else "false"),
                "evaluated": "true" if gate["evaluated"] else "false",
            })
        old_metrics = cell["metrics"].get("old") or {}
        new_metrics = cell["metrics"].get("new") or {}
        regions = sorted(set(old_metrics.get("suppression", {})) |
                         set(new_metrics.get("suppression", {})))
        for region in regions:
            old_detail = old_metrics.get("suppression", {}).get(region, {})
            new_detail = new_metrics.get("suppression", {}).get(region, {})
            rows.append({
                "cell": cell["cell"], "scope": "cell:suppression",
                "window_id": "", "source_window": "",
                "metric": "suppression_db:" + region,
                "old": _cell_text(old_detail.get("suppression_db")),
                "new": _cell_text(new_detail.get("suppression_db")),
                "delta": _cell_text(_delta(new_detail.get("suppression_db"),
                                           old_detail.get("suppression_db"))),
                "tolerance": "", "passed": "", "evaluated": "true"})
        for name in ("target_suppression_db", "cir_recovery_db",
                     "weak_target_abs_error_db"):
            old_value = old_metrics.get(name)
            new_value = new_metrics.get(name)
            rows.append({
                "cell": cell["cell"], "scope": "cell:paired", "window_id": "",
                "source_window": "", "metric": name,
                "old": _cell_text(old_value), "new": _cell_text(new_value),
                "delta": _cell_text(cell["exact_differences"].get(name)),
                "tolerance": _cell_text(SUPPORT_TOLERANCE_DB),
                "passed": "", "evaluated": "true"})
        for entry in cell["per_window"]:
            for name in ("target_suppression_db", "cir_recovery_db",
                         "weak_target_abs_error_db"):
                old_value = _window_value(entry["old"], name)
                new_value = _window_value(entry["new"], name)
                rows.append({
                    "cell": cell["cell"], "scope": "window",
                    "window_id": entry["window_id"],
                    "source_window": entry["source_window"], "metric": name,
                    "old": _cell_text(old_value), "new": _cell_text(new_value),
                    "delta": _cell_text(entry["delta_new_minus_old"].get(name)),
                    "tolerance": _cell_text(SUPPORT_TOLERANCE_DB),
                    "passed": "", "evaluated": "true"})
    fieldnames = ["cell", "scope", "window_id", "source_window", "metric",
                  "old", "new", "delta", "tolerance", "passed", "evaluated"]
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _window_value(metric, name):
    if name == "target_suppression_db":
        return metric.get("target_suppression_db")
    cir = metric.get("cir") or {}
    return cir.get(name)


def _cell_text(value):
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def write_outputs(result, out):
    out = Path(out)
    if out.exists():
        raise FileExistsError(f"comparison output exists; refusing overwrite: {out}")
    out.parent.mkdir(parents=True, exist_ok=True)
    write_json(out, result)
    write_csv(result, out.with_suffix(".csv"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path)
    parser.add_argument("--old-run", type=Path)
    parser.add_argument("--new-run", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--require-provenance", action="store_true")
    parser.add_argument("--summary-only", action="store_true",
                        help="validate records only; dumped IQ checks are skipped")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import qa_discrete_cir_compare
        return qa_discrete_cir_compare.main()
    if not args.case or not args.old_run or not args.new_run:
        parser.error("--case, --old-run and --new-run are required")
    try:
        result = compare(args.case, args.old_run, args.new_run,
                         require_provenance=args.require_provenance,
                         summary_only=args.summary_only)
        out = args.out or (Path(args.new_run) / "comparison.json")
        write_outputs(result, out)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        print(f"comparison failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"passed": result["passed"],
                      "level": result["verification_level"],
                      "failures": result["failures"],
                      "exact_differences": result["cells"][0]["exact_differences"]},
                     allow_nan=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
