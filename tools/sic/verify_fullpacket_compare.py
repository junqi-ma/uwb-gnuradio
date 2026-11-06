#!/usr/bin/env python3
"""Independent artifact verifier and paired statistics for full-packet SIC.

No online implementation is imported. IQ effects require dumped output; explicit
--summary-only validates records/rates only and never certifies IQ effects.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.signal import correlate

MODES = {"preamble": "preamble", "preamble_only": "preamble",
         "full": "full", "full_packet_decode8": "full"}
STAGES = ("estimate", "comm_demod", "validate", "tx", "channel", "subtract",
          "copy", "sic_total", "cir", "sensing_demod", "service")
COMM_STAGES = ("timing", "cfo", "sfd", "cir", "ns_sfd", "phr", "payload")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def crc16(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def valid_fcs(text):
    try:
        data = bytes.fromhex(text)
    except (ValueError, TypeError):
        return False
    return len(data) >= 2 and crc16(data[:-2]) == int.from_bytes(data[-2:], "little")


def rate(successes, total):
    """95% Wilson interval, including finite uncertainty for zero failures."""
    if total == 0:
        return {"successes": successes, "n": 0, "rate": None, "ci95": None}
    p = successes / total
    z2 = 1.959963984540054 ** 2
    center = (p + z2 / (2 * total)) / (1 + z2 / total)
    half = math.sqrt(z2 * (p * (1-p) / total + z2 / (4*total**2))) / (1+z2/total)
    return {"successes": successes, "n": total, "rate": p,
            "ci95": [max(0., center-half), min(1., center+half)]}


def distribution(values):
    if not values:
        return {"n": 0}
    a = np.asarray(values, dtype=float)
    return dict(n=int(a.size), mean=float(a.mean()),
                **{f"p{p}": float(np.percentile(a, p)) for p in (50, 95, 99)})


def source_dir(case, source):
    if isinstance(source, int) or str(source).isdigit():
        return case / "windows" / f"w{int(source):06d}"
    name = Path(str(source)).name
    if not name.startswith("w") or not name[1:].isdigit():
        raise ValueError(f"invalid source_window: {source}")
    return case / "windows" / name


def read_iq(path):
    if path.stat().st_size % 8:
        raise ValueError(f"unaligned CF32: {path}")
    result = np.fromfile(path, dtype="<c8")
    if not np.isfinite(result).all():
        raise ValueError(f"nonfinite CF32: {path}")
    return result


def bounds(value):
    if isinstance(value, dict):
        return int(value.get("lo", value.get("begin", 0))), int(value.get("hi", value.get("end", 0)))
    return int(value[0]), int(value[1])


def energy(a):
    return float(np.vdot(a.astype(np.complex128), a.astype(np.complex128)).real)


def db_ratio(a, b):
    if a <= 0 or b <= 0:
        return None
    return 10 * math.log10(a / b)


def cir_estimate(iq, sync, origin, n_reps=64):
    """Independent matched filter average, reps 9..62, taps -8..64.

    Diagnostic is relative to clean under identical filtering, rather than an
    uncalibrated claim that taps equal physical channel gain.
    """
    sps = len(sync)
    lo, hi = max(0, origin + 9*sps - 8), min(len(iq), origin + 63*sps + 65)
    if hi - lo < sps or sps == 0:
        return None
    corr = correlate(iq[lo:hi], sync, mode="valid", method="fft")
    result = []
    for delay in range(-8, 65):
        indices = np.array([origin+r*sps+delay-lo for r in range(9, min(n_reps-1, 63))])
        indices = indices[(indices >= 0) & (indices < len(corr))]
        result.append(np.mean(corr[indices]) / max(energy(sync), 1e-30) if len(indices) else 0j)
    return np.asarray(result)


def effect_metrics(raw, clean, out, public, sidecar, sync):
    n = len(raw)
    regions = {"all": (0, n), "target": (public["target_lo"], public["target_hi"])}
    for name, span in sidecar.get("fields", {}).items():
        try:
            lo, hi = bounds(span)
            regions["comm_"+name.lower()] = (max(0, lo), min(n, hi))
        except (TypeError, KeyError, IndexError, ValueError):
            continue
    result = {}
    for name, (lo, hi) in regions.items():
        lo, hi = max(0, int(lo)), min(n, int(hi))
        if hi <= lo:
            continue
        before, after = raw[lo:hi]-clean[lo:hi], out[lo:hi]-clean[lo:hi]
        ei, er = energy(before), energy(after)
        result[name] = {"bounds": [lo, hi], "input_interference_energy": ei,
                        "residual_energy": er, "suppression_db": db_ratio(ei, er)}
    origin = int(sidecar.get("sensing_start", public.get("sensing_seed", public["target_lo"])))
    c = cir_estimate(clean, sync, origin)
    r, q = cir_estimate(raw, sync, origin), cir_estimate(out, sync, origin)
    if c is not None and r is not None and q is not None:
        raw_err, out_err = energy(r-c), energy(q-c)
        result["cir"] = {"diagnostic": "matched_filter_reps9_62_taps_minus8_64",
                         "raw_error_energy": raw_err, "output_error_energy": out_err,
                         "recovery_db": db_ratio(raw_err, out_err),
                         "relative_error": math.sqrt(out_err/max(energy(c), 1e-30)),
                         "peak_shift_samples": int(np.argmax(abs(q))-np.argmax(abs(c)))}
        # Known configured weak echo at +37 work samples, evaluated identically
        # in clean/raw/sic; no target location is fed to the online path.
        delay = int(sidecar.get("target_delay_samples", 37))
        tap = max(0, min(len(c)-1, delay + 8))
        amp = lambda v: 20*math.log10(max(abs(v[tap]), 1e-30)/max(abs(c[tap]), 1e-30))
        result["cir"]["weak_target_amp_error_db"] = amp(q)
        result["cir"]["weak_target_raw_amp_error_db"] = amp(r)
    return result


def validate_record(row, public):
    errors = []
    mode = MODES.get(row.get("mode"))
    check = lambda ok, why: errors.append(why) if not ok else None
    check(mode is not None, "mode")
    check(type(row.get("committed")) is bool, "committed type")
    check(type(row.get("model_valid")) is bool, "model_valid type")
    check(row.get("n") == public["n"], "n mismatch")
    check(row.get("observed_end") == public["observed_end"], "observed_end mismatch")
    try:
        lo, hi = bounds(row["support"])
        check(0 <= lo <= hi <= public["n"], "support bounds")
        check(not row["committed"] or (row["model_valid"] and hi > lo), "commit without model/support")
    except (KeyError, ValueError, TypeError, IndexError):
        errors.append("support missing/invalid")
    if mode == "preamble":
        check(row.get("comm_fcs") is None, "preamble decoded comm")
    elif mode == "full":
        check(type(row.get("comm_fcs")) is bool, "full comm_fcs type")
        check(not row.get("committed") or row.get("comm_fcs") is True, "full commit without FCS")
        if row.get("comm_fcs"):
            check(valid_fcs(row.get("comm_psdu_hex")), "invalid reported comm FCS")
    if row.get("sensing_fcs"):
        check(valid_fcs(row.get("sensing_psdu_hex")), "invalid reported sensing FCS")
    times = row.get("times_us", {})
    rejected = row.get("reason") in ("queue_full", "source_late", "cancelled", "not_submitted")
    for key in STAGES:
        if rejected and key not in times:
            continue
        value = times.get(key)
        check(isinstance(value, (int, float)) and math.isfinite(value) and value >= 0, "time "+key)
    if all(isinstance(times.get(k), (int, float)) for k in ("service", "sic_total")):
        check(times["service"] + 2 >= times["sic_total"], "service below SIC time")
    if not rejected and all(isinstance(times.get(k), (int,float)) for k in STAGES):
        sic_children = sum(times[k] for k in ("estimate", "comm_demod", "validate", "tx", "channel", "subtract", "copy"))
        check(abs(times["sic_total"] - sic_children) <= 2., "SIC stage partition")
        check(abs(times["service"] - times["sic_total"] - times["cir"] - times["sensing_demod"]) <= 2., "service stage partition")
    for field in ("thread_cpu", "thread_cpu_us"):
        if field in times:
            check(isinstance(times[field], (int,float)) and math.isfinite(times[field]) and times[field] >= 0, "thread CPU time")
    # New RX metadata is optional for archived runs, but complete when emitted.
    if "rx_pipeline" in row or "rx_avx2" in row:
        check(isinstance(row.get("rx_pipeline"), str) and bool(row.get("rx_pipeline")), "RX pipeline metadata")
        check(type(row.get("rx_avx2")) is bool, "RX AVX2 metadata")
    if "comm_stage_us" in row:
        detail = row["comm_stage_us"]
        check(isinstance(detail, dict), "comm stage counters type")
        if isinstance(detail, dict) and detail:
            check(mode == "full" and not rejected, "comm counters without decoding")
            check(set(detail) == set(COMM_STAGES), "comm stage counter names")
            valid_detail = all(type(v) in (int, float) and math.isfinite(v) and v >= 0
                               for v in detail.values())
            check(valid_detail, "comm stage counter value")
            parent = times.get("comm_demod")
            if valid_detail and type(parent) in (int, float) and math.isfinite(parent):
                # Existing counters round each lap to integer microseconds and
                # exclude wrapper overhead. They are diagnostics, never added
                # again to SIC service time.
                check(sum(detail.values()) <= parent + 10., "comm stage counters exceed parent")
    events = row.get("events", {})
    try:
        values = [float(events[k]) for k in ("submit_us", "start_us", "finish_us", "result_us")]
        check(all(math.isfinite(v) and v >= 0 for v in values), "invalid event time")
        check(values == sorted(values), "event order")
        sample = events["required_sample"]
        check(isinstance(sample, (int, float)) and 0 <= sample <= public["n"], "required sample")
        if mode == "full" and not rejected:
            check(sample == public["n"], "full context observation policy")
        # Rejection publishes only immediate status: first_clean_us is then a
        # placeholder, not an IQ output event. Accepted rows retain causality.
        if not rejected and "first_clean_us" in events and "sample_ready_us" in events:
            check(events["first_clean_us"] + 1e-6 >= events["sample_ready_us"], "early clean before samples")
    except (KeyError, TypeError, ValueError):
        errors.append("events missing/invalid")
    return errors


def verify(case, run, summary_only=False, require_provenance=False):
    case, run = Path(case).resolve(), Path(run).resolve()
    public = json.loads((case/"public.json").read_text())
    rows = [json.loads(line) for line in (run/"windows.jsonl").read_text().splitlines() if line.strip()]
    errors, warnings, seen, by_mode, metrics, paired = [], [], set(), defaultdict(list), [], defaultdict(list)
    runner_summary_path = run/"runner_summary.json"
    runner_summary = json.loads(runner_summary_path.read_text()) if runner_summary_path.exists() else None
    paced = bool(runner_summary and runner_summary.get("paced_rate", 0) > 0)
    provenance_verified = True
    cache = {}
    template_hashes = {}
    for name in ("comm_sync.cf32", "comm_preamble.cf32", "sensing_sync.cf32"):
        p = case/name
        if not p.exists():
            errors.append("missing template "+name)
            continue
        template_hashes[name] = sha256(p)
        expected = public.get("template_hashes", {}).get(name)
        if not expected:
            provenance_verified = False
            warnings.append("missing expected template hash "+name)
        if expected and expected != template_hashes[name]:
            errors.append("template hash mismatch "+name)
    sync = read_iq(case/"sensing_sync.cf32") if not summary_only else None
    for index, row in enumerate(rows):
        label = f"row {index}"
        errors += [label+": "+e for e in validate_record(row, public)]
        mode = MODES.get(row.get("mode"), "invalid")
        key = (row.get("window_id"), mode)
        if key in seen:
            errors.append(label+": duplicate terminal")
        seen.add(key)
        by_mode[mode].append(row)
        source = source_dir(case, row["source_window"])
        if source not in cache:
            side = json.loads((source/"sidecar.json").read_text())
            raw_hash = sha256(source/"raw.cf32")
            hashes = side.get("hashes", {})
            for name, expected in hashes.items():
                p = source/name
                if p.exists() and sha256(p) != expected:
                    errors.append(f"source hash mismatch: {source.name}/{name}")
            cache[source] = (side, raw_hash)
        side, raw_hash = cache[source]
        if not row.get("input_hash"):
            provenance_verified = False
            warnings.append(label+": missing recorded input hash; current source hash computed only")
        if row.get("input_hash") and row["input_hash"] != raw_hash:
            errors.append(label+": input hash mismatch")
        paired[(row.get("window_id"), str(row["source_window"]))].append((mode, raw_hash, row.get("model_id"), row.get("origin"), row.get("model_valid"), row.get("rx_pipeline"), row.get("rx_avx2")))
        if summary_only:
            continue
        raw, clean = read_iq(source/"raw.cf32"), read_iq(source/"clean.cf32")
        if len(raw) != public["n"] or len(clean) != public["n"]:
            errors.append(label+": source size")
            continue
        output = run/f"w{int(row['window_id']):06d}_{row['mode']}.cf32"
        if not output.exists():
            errors.append(label+": output IQ missing (use --summary-only for nondumped timing runs)")
            continue
        out = read_iq(output)
        if len(out) != len(raw):
            errors.append(label+": output size")
            continue
        lo, hi = bounds(row["support"])
        # Equality is bytes, including signed zero, rather than approximate IQ.
        def same(a, b):
            return a.tobytes() == b.tobytes()
        if not same(out[:lo], raw[:lo]) or not same(out[hi:], raw[hi:]):
            errors.append(label+": output changed outside support")
        if not row["committed"] and not same(out, raw):
            errors.append(label+": bypass changed IQ")
        if row.get("output_hash") and row["output_hash"] != sha256(output):
            errors.append(label+": output hash mismatch")
        metrics.append({"window_id": row["window_id"], "mode": row["mode"],
                        "source_window": row["source_window"],
                        "output_sha256": sha256(output),
                        "effect": effect_metrics(raw, clean, out, public, side, sync)})
    # In both mode runs all source samples must be paired. Single-mode paced
    # runs are valid and explicitly identified by the summary's mode list.
    modes = set(by_mode)-{"invalid"}
    if len(modes) > 1:
        for key, values in paired.items():
            if {v[0] for v in values} != {"preamble", "full"}:
                errors.append(f"missing paired mode: {key}")
            if len({v[1] for v in values}) > 1:
                errors.append(f"unpaired input hashes: {key}")
            if len({(v[2],v[3],v[4]) for v in values}) > 1:
                errors.append(f"unpaired estimation model: {key}")
            if len({(v[5],v[6]) for v in values}) > 1:
                errors.append(f"unpaired RX pipeline: {key}")
    if runner_summary is not None:
        submitted, completed, rejected = (runner_summary.get(k) for k in ("submitted", "completed", "rejected"))
        actual_rejected = sum(bool(r.get("rejected", r.get("reason") == "queue_full")) for r in rows)
        if not all(type(v) is int and v >= 0 for v in (submitted,completed,rejected)):
            errors.append("runner summary count types")
        elif submitted != completed + rejected or submitted != len(rows) or rejected != actual_rejected:
            errors.append("runner summary terminal accounting")
        for field in ("rx_pipeline", "rx_avx2"):
            if field in runner_summary and any(row.get(field) != runner_summary[field] for row in rows):
                errors.append("runner summary "+field+" mismatch")
    else:
        warnings.append("runner_summary.json absent; submitted/completed/rejected accounting unavailable")
    if require_provenance and not provenance_verified:
        errors.append("required input/template provenance unavailable")
    summaries = {}
    for mode, group in by_mode.items():
        n, cf, ce, sf, se = len(group), 0, 0, 0, 0
        accepted = [r for r in group if not r.get("rejected", r.get("reason") == "queue_full")]
        rejected_rows = [r for r in group if r.get("rejected", r.get("reason") == "queue_full")]
        for row in group:
            side = cache[source_dir(case, row["source_window"])][0]
            cf += row.get("comm_fcs") is True
            ce += row.get("comm_fcs") is True and row.get("comm_psdu_hex", "").lower() == side.get("comm_psdu_hex", "missing").lower()
            sf += row.get("sensing_fcs") is True
            se += row.get("sensing_fcs") is True and row.get("sensing_psdu_hex", "").lower() == side.get("sensing_psdu_hex", "missing").lower()
        unique = {}
        for row in group:
            source = str(row["source_window"])
            if source not in unique or unique[source].get("reason") == "queue_full":
                unique[source] = row
        stats = {"n": n, "unique_sources": len(unique),
                 "exposure_rates_are_independent_trials": len(unique) == n, "comm_fcs": rate(cf, n) if mode == "full" else None,
                 "comm_exact": rate(ce, n) if mode == "full" else None,
                 "comm_exact_given_fcs": rate(ce, cf) if mode == "full" else None,
                 "sensing_fcs": rate(sf, n), "sensing_exact": rate(se, n),
                 "committed": rate(sum(r["committed"] is True for r in group), n),
                 "reasons": dict(Counter(r.get("reason", "missing") for r in group)),
                 "processing_n": len(accepted), "rejected_n": len(rejected_rows),
                 "times_us": {k: distribution([r["times_us"][k] for r in accepted if isinstance(r.get("times_us", {}).get(k), (int,float))]) for k in STAGES},
                 "sic_success_us": distribution([r["times_us"]["sic_total"] for r in group if r.get("committed") and "sic_total" in r.get("times_us", {})])}
        detailed = [r["comm_stage_us"] for r in accepted
                    if isinstance(r.get("comm_stage_us"), dict) and r["comm_stage_us"]]
        stats["comm_stage_us"] = {
            k: distribution([d[k] for d in detailed if type(d.get(k)) in (int,float)
                             and math.isfinite(d[k]) and d[k] >= 0])
            for k in COMM_STAGES} if detailed else {}
        stats["comm_stage_note"] = "diagnostic children of comm_demod; not additive to SIC total"
        rx = {(r.get("rx_pipeline"), r.get("rx_avx2")) for r in group
              if "rx_pipeline" in r or "rx_avx2" in r}
        if rx:
            stats["rx_configurations"] = [dict(rx_pipeline=p, rx_avx2=a)
                                          for p,a in sorted(rx, key=repr)]
        latency = defaultdict(list)
        for row in accepted:
            event = row.get("events", {})
            if not all(isinstance(event.get(k),(int,float)) for k in ("submit_us","start_us","finish_us","result_us")):
                continue
            latency["queue_us"].append(event["start_us"]-event["submit_us"])
            latency["sic_complete_from_submit_us"].append(event["finish_us"]-event["submit_us"])
            latency["result_from_submit_us"].append(event["result_us"]-event["submit_us"])
            if "first_clean_us" in event:
                latency["first_clean_from_submit_us"].append(event["first_clean_us"]-event["submit_us"])
                if paced and "scheduled_us" in event:
                    origin = event["scheduled_us"] - row["observed_end"]*1e6/public["sample_rate"]
                    latency["first_clean_from_sample_origin_us"].append(event["first_clean_us"]-origin)
                    latency["result_from_sample_origin_us"].append(event["result_us"]-origin)
            latency["logical_required_sample_us"].append(event["required_sample"]*1e6/public["sample_rate"])
        stats["latency_reference"] = "real_paced_sample_origin" if paced else "prebuffered_submit_no_arrival_claim"
        stats["latency_us"] = {k: distribution(v) for k,v in latency.items()}
        stats["rejection_response_us"] = distribution([r["events"]["result_us"]-r["events"]["submit_us"] for r in rejected_rows])
        for field in ("thread_cpu", "thread_cpu_us"):
            cpu = [r["times_us"][field] for r in accepted if field in r.get("times_us",{})]
            if cpu:
                stats["times_us"][field] = distribution(cpu)
        if len(unique) < n:
            # Repeated exposures describe machine processing, not new random packets.
            for name in ("comm_fcs", "comm_exact", "comm_exact_given_fcs", "sensing_fcs", "sensing_exact", "committed"):
                if stats.get(name):
                    stats[name]["ci95"] = None
                    stats[name]["ci_note"] = "repeated source exposures; use unique_source_rates"
        counts = dict(comm_fcs=0, comm_exact=0, sensing_fcs=0, sensing_exact=0)
        for row in unique.values():
            side = cache[source_dir(case, row["source_window"])][0]
            counts["comm_fcs"] += row.get("comm_fcs") is True
            counts["sensing_fcs"] += row.get("sensing_fcs") is True
            counts["comm_exact"] += row.get("comm_fcs") is True and row.get("comm_psdu_hex", "").lower() == side.get("comm_psdu_hex", "missing").lower()
            counts["sensing_exact"] += row.get("sensing_fcs") is True and row.get("sensing_psdu_hex", "").lower() == side.get("sensing_psdu_hex", "missing").lower()
        stats["unique_source_rates"] = {name: rate(count, len(unique)) for name, count in counts.items() if mode == "full" or not name.startswith("comm")}
        stats["effects"] = {}
        group_metrics = [m["effect"] for m in metrics if MODES.get(m["mode"]) == mode]
        for region in sorted({region for m in group_metrics for region in m}):
            region_rows = [m[region] for m in group_metrics if region in m]
            stats["effects"][region] = {}
            for field in sorted({field for m in region_rows for field in m}):
                values = [m[field] for m in region_rows if isinstance(m.get(field), (int,float)) and math.isfinite(m[field])]
                if values:
                    stats["effects"][region][field] = distribution(values)
        summaries[mode] = stats
    if not rows:
        errors.append("no terminal rows")
    return {"schema": "sic-fullpacket-verification/1", "passed": not errors,
            "provenance_verified": provenance_verified,
            "provenance_scope": "recorded_input_and_public_template_hashes_only; build provenance handled by driver",
            "warnings": warnings, "runner_summary": runner_summary,
            "verification_level": "summary_only" if summary_only else "dumped_iq",
            "iq_effects_verified": not summary_only and not errors,
            "case": str(case), "run": str(run), "rows": len(rows),
            "template_hashes": template_hashes, "modes": summaries,
            "errors": errors, "windows": metrics}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path)
    parser.add_argument("--run", type=Path)
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--require-provenance", action="store_true")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)
    if args.self_test:
        import qa_fullpacket_compare
        return qa_fullpacket_compare.main()
    if not args.case or not args.run:
        parser.error("--case and --run are required")
    try:
        result = verify(args.case, args.run, args.summary_only, args.require_provenance)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        print(f"verification failed: {exc}", file=sys.stderr)
        return 2
    target = args.out or args.run/"verification.json"
    target.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"passed": result["passed"], "rows": result["rows"],
                      "level": result["verification_level"], "provenance_verified": result["provenance_verified"], "errors": result["errors"]}))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
