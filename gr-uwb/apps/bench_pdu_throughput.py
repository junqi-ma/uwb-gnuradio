#!/usr/bin/env python3
"""M0 PDU-throughput benchmark for ``x410_cg400_hrp_echo_cir.py``.

The app writes three artifacts that this tool consumes:

  <out>/echo_timing.jsonl   one JSON record per burst (``--timing-detail``)
  <out>/summary.json        aggregate counters and timing min/max/mean
  <out>.stdout.log          live lines, per-pulse ``pub pulse=`` lines and
                            the ``[timing]`` block printed by --timing-detail

From those it reports count/mean/p50/p95/p99/max for the per-burst host
steps (get_time_ms, issue_ms, send_ms, recv_ms, uhd_ms, enqueue_ms,
sc16_ms), the publisher stages (contig_ms, tolist_ms, pmt_ms, pub_ms,
total_ms), resampler/estimator service times, late/est_drop counters,
schedule_wall_s and the effective CIR/s.

The publisher per-pulse lines are only emitted for ``pulse_id < 3`` and
``pulse_id % 100 == 99`` (see ``TimedUhdEcho._publisher``); the reported
publisher count therefore reflects those samples, not every pulse.  When
only the aggregate ``[timing] publisher`` block is available (no per-pulse
lines) the mean/min/max are taken from it and the percentiles are null.

Modes
-----
``--parse-only``
    Report on existing logs (no hardware).  The path may be a single run
    directory or a "tag" directory whose immediate subdirectories are runs.

``--run``
    Drive the app through ``subprocess`` over the requested rates
    (default 100/200/500 Hz) with fixed parameters: preamble 128,
    minphase pulse shaping, gain 50/60, ``--no-udp`` and the same
    ``--pulses``.  Each run's stdout is teed to
    ``<tag>/<rate>hz.stdout.log`` and its timing to
    ``<tag>/<rate>hz/echo_timing.jsonl``; a Markdown + JSON A/B table is
    written to ``<tag>/bench_summary.{md,json}``.

Only the Python standard library is used, so ``--parse-only`` works with a
bare ``python3``.  Hardware is only ever touched by the child process; this
tool never runs two device jobs concurrently.
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import re
import subprocess
import sys
import time

# Per-burst host steps written by ``TimedUhdEcho._one_burst``.
ECHO_STEP_KEYS = (
    "get_time_ms",
    "issue_ms",
    "send_ms",
    "recv_ms",
    "uhd_ms",
    "enqueue_ms",
    "sc16_ms",
)
# Publisher stages recorded by ``TimedUhdEcho._publisher``.
PUB_KEYS = ("contig_ms", "tolist_ms", "pmt_ms", "pub_ms", "total_ms")
# ``[timing]`` stdout labels -> internal key.
TIMING_LABELS = {
    "usrp get_time_now": "get_time_ms",
    "rx issue_stream_cmd": "issue_ms",
    "tx send (fifo)": "send_ms",
    "rx recv (rf+wait)": "recv_ms",
    "sc16 dump": "sc16_ms",
    "enqueue to publisher": "enqueue_ms",
    "one burst uhd total": "uhd_ms",
    "ascontiguousarray": "contig_ms",
    "rx.tolist()": "tolist_ms",
    "init_c32vector": "pmt_ms",
    "message_port_pub": "pub_ms",
    "publisher total": "total_ms",
}
# Optional summary keys a future app revision may expose for the 65/32
# resampler (see UwbPduRationalResamplerCcf65_32 accessors).
RES_SERVICE_SUMMARY_KEYS = (
    "res_service_us_mean",
    "res_service_us_max",
    "resample_us_mean",
    "resample_us_max",
    "res_total_us",
    "res_max_us",
    "resample_total_us",
    "resample_max_us",
)
# Per-pulse meta keys the CIR writer may expose (via lineage passthrough).
CIR_SERVICE_KEYS = ("resample_us", "estimator_us", "queue_delay_us", "queue_us")

DEFAULT_APP = os.path.join("gr-uwb", "apps", "x410_cg400_hrp_echo_cir.py")


# ---------------------------------------------------------------------------
# statistics
# ---------------------------------------------------------------------------

def _percentile(sorted_vals, p):
    """Linear-interpolation percentile matching numpy's default."""
    n = len(sorted_vals)
    if n == 0:
        return None
    if n == 1:
        return float(sorted_vals[0])
    idx = (n - 1) * p
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return float(sorted_vals[lo])
    frac = idx - lo
    return (float(sorted_vals[lo]) * (1.0 - frac)
            + float(sorted_vals[hi]) * frac)


def stat_block(values):
    """count/mean/min/p50/p95/p99/max over the finite values."""
    vals = []
    for v in values:
        if v is None:
            continue
        try:
            fv = float(v)
        except (TypeError, ValueError):
            continue
        if math.isfinite(fv):
            vals.append(fv)
    if not vals:
        return {"count": 0, "mean": None, "min": None, "p50": None,
                "p95": None, "p99": None, "max": None}
    s = sorted(vals)
    return {
        "count": len(vals),
        "mean": sum(vals) / len(vals),
        "min": s[0],
        "p50": _percentile(s, 0.50),
        "p95": _percentile(s, 0.95),
        "p99": _percentile(s, 0.99),
        "max": s[-1],
    }


def _stat_from_triplet(triplet):
    """Build a stat block from a (mean, min, max, n) ``[timing]`` row."""
    if not triplet:
        return {"count": 0, "mean": None, "min": None, "p50": None,
                "p95": None, "p99": None, "max": None}
    mean, lo, hi, n = triplet
    return {"count": int(n), "mean": mean, "min": lo, "p50": None,
            "p95": None, "p99": None, "max": hi}


# ---------------------------------------------------------------------------
# artifact parsing
# ---------------------------------------------------------------------------

def _read_json(path):
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def parse_timing_jsonl(path):
    """Parse the per-burst echo timing JSONL into step stat blocks."""
    out = {
        "file": path,
        "n": 0,
        "ok": 0,
        "late": 0,
        "status_hist": {},
        "steps": {},
        "lead_s": stat_block([]),
        "dt_tx_s": stat_block([]),
        "span_s": None,
    }
    if not path or not os.path.isfile(path):
        return out
    recs = []
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                try:
                    recs.append(json.loads(ln))
                except ValueError:
                    continue
    out["n"] = len(recs)
    for r in recs:
        st = r.get("status", "")
        out["status_hist"][st] = out["status_hist"].get(st, 0) + 1
    ok = [r for r in recs if r.get("status") == "ok"]
    out["ok"] = len(ok)
    out["late"] = out["status_hist"].get("late", 0)
    # Step stats over successful bursts (mirrors print_timing_detail).
    for key in ECHO_STEP_KEYS:
        out["steps"][key] = stat_block(
            [r.get(key) for r in ok if key in r])
    out["lead_s"] = stat_block(
        [r.get("lead_s") for r in recs if r.get("lead_s") is not None])
    out["dt_tx_s"] = stat_block(
        [b.get("t_tx", 0) - a.get("t_tx", 0)
         for a, b in zip(ok, ok[1:])])
    if len(ok) >= 2:
        out["span_s"] = ok[-1].get("t_tx", 0.0) - ok[0].get("t_tx", 0.0)
    return out


_PUB_KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([-+0-9.eE]+)")


def parse_stdout(path):
    """Parse stdout for publisher samples, [timing] rows and key=value lines."""
    out = {
        "file": path,
        "exists": os.path.isfile(path),
        "pub": {k: [] for k in PUB_KEYS},
        "pub_samples": [],
        "pub_n_lines": 0,
        "timing_rows": {},
        "est_service": None,
        "schedule_wall_s": None,
        "sched_last": {},
        "live_last": {},
        "radio_ok": {},
        "cir_jsonl_lines": None,
    }
    if not out["exists"]:
        return out
    timing_re = re.compile(
        r"^\[timing\]\s+(.*?)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+"
        r"([-+0-9.eE]+)\s+(\d+)\s*$")
    pub_map = {
        "contig": "contig_ms",
        "tolist": "tolist_ms",
        "pmt": "pmt_ms",
        "pmt_ms": "pmt_ms",
        "pub": "pub_ms",
        "pub_ms": "pub_ms",
        "total": "total_ms",
        "total_ms": "total_ms",
    }
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            # The publisher thread can glue its line onto a preceding
            # ``sched ...`` line with no newline, so scan for embedded
            # records instead of relying on line starts.
            chunks = line.split("pub pulse=")
            prefix = chunks[0]
            for chunk in chunks[1:]:
                out["pub_n_lines"] += 1
                kvs = dict(_PUB_KV_RE.findall(chunk))
                if "samples" in kvs:
                    try:
                        out["pub_samples"].append(int(float(kvs["samples"])))
                    except ValueError:
                        pass
                for src, dst in pub_map.items():
                    if src in kvs:
                        try:
                            out["pub"][dst].append(float(kvs[src]))
                        except ValueError:
                            pass
            if chunks[1:]:
                line = prefix
            if not line:
                continue
            if line.startswith("[timing]"):
                m = timing_re.match(line)
                if m:
                    label = m.group(1).strip()
                    key = TIMING_LABELS.get(label, label)
                    out["timing_rows"][key] = (
                        float(m.group(2)), float(m.group(3)),
                        float(m.group(4)), int(m.group(5)))
                elif "mean=" in line and "max=" in line:
                    mm = re.search(r"mean=([-+0-9.eE]+)\s*us", line)
                    xm = re.search(r"max=([-+0-9.eE]+)\s*us", line)
                    if mm and xm:
                        out["est_service"] = {
                            "mean_us": float(mm.group(1)),
                            "max_us": float(xm.group(1)),
                        }
            elif line.startswith("schedule_wall_s="):
                out["schedule_wall_s"] = _kv_float(line.split("=", 1)[1])
            elif line.startswith("sched "):
                out["sched_last"] = _kv_float_dict(line)
            elif line.startswith("live "):
                out["live_last"] = _kv_float_dict(line)
            elif line.startswith("radio_ok="):
                out["radio_ok"] = _kv_float_dict(line)
            elif line.startswith("cir.jsonl_lines="):
                out["cir_jsonl_lines"] = _kv_float(line.split("=", 1)[1])
    return out


def _kv_float(tok):
    try:
        return float(tok)
    except (TypeError, ValueError):
        return None


def _kv_float_dict(line):
    d = {}
    for k, v in _PUB_KV_RE.findall(line):
        try:
            d[k] = float(v)
        except ValueError:
            pass
    return d


def _parse_cir_service(cir_path):
    """Per-pulse service microseconds from the CIR JSONL, when present."""
    out = {}
    if not os.path.isfile(cir_path):
        return out
    cols = {k: [] for k in CIR_SERVICE_KEYS}
    with open(cir_path, "r", encoding="utf-8", errors="replace") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                rec = json.loads(ln)
            except ValueError:
                continue
            if rec.get("status") not in (None, "ok"):
                continue
            for k in CIR_SERVICE_KEYS:
                if k in rec:
                    try:
                        cols[k].append(float(rec[k]))
                    except (TypeError, ValueError):
                        pass
    for k, vals in cols.items():
        if vals:
            out[k] = stat_block(vals)
    return out


# ---------------------------------------------------------------------------
# case assembly
# ---------------------------------------------------------------------------

def _first_existing(*paths):
    for p in paths:
        if p and os.path.isfile(p):
            return p
    return ""


def _discover_stdout(run_dir):
    base = os.path.basename(os.path.normpath(run_dir))
    parent = os.path.dirname(os.path.normpath(run_dir))
    return _first_existing(
        os.path.join(run_dir, "stdout.log"),
        run_dir.rstrip("/") + ".stdout.log",
        os.path.join(parent, base + ".stdout.log"),
    )


def build_case(run_dir, stdout_path=None, label=None):
    run_dir = os.path.abspath(run_dir)
    timing_path = _first_existing(
        os.path.join(run_dir, "echo_timing.jsonl"),
        os.path.join(run_dir, "timing.jsonl"),
    )
    summary = _read_json(os.path.join(run_dir, "summary.json"))
    stdout_path = stdout_path or _discover_stdout(run_dir)
    stdout = parse_stdout(stdout_path)
    timing = parse_timing_jsonl(timing_path)
    cir_service = _parse_cir_service(os.path.join(run_dir, "cir.jsonl"))

    rate = summary.get("rate_hz")
    if label is None:
        if rate:
            label = "%ghz" % float(rate)
        else:
            label = os.path.basename(run_dir)

    # Echo step stats: prefer the JSONL; fall back to the printed [timing] block.
    steps = {}
    for key in ECHO_STEP_KEYS:
        if timing["steps"].get(key, {}).get("count"):
            steps[key] = timing["steps"][key]
        elif key in stdout["timing_rows"]:
            steps[key] = _stat_from_triplet(stdout["timing_rows"][key])
        else:
            steps[key] = {"count": 0, "mean": None, "min": None,
                          "p50": None, "p95": None, "p99": None, "max": None}

    # Publisher stats: per-pulse lines give a distribution; the [timing]
    # publisher block gives mean/min/max only.
    publisher = {}
    pub_sources = {}
    for key in PUB_KEYS:
        if stdout["pub"][key]:
            publisher[key] = stat_block(stdout["pub"][key])
            pub_sources[key] = "stdout_pub_lines"
        elif key in stdout["timing_rows"]:
            publisher[key] = _stat_from_triplet(stdout["timing_rows"][key])
            pub_sources[key] = "stdout_timing_block"
        else:
            publisher[key] = {"count": 0, "mean": None, "min": None,
                              "p50": None, "p95": None, "p99": None,
                              "max": None}
            pub_sources[key] = "missing"
    pub_samples = stat_block(stdout["pub_samples"])

    counts = {
        "pulses": summary.get("pulses"),
        "echo_ok": summary.get("echo_ok"),
        "echo_fail": summary.get("echo_fail"),
        "echo_late": summary.get("echo_late"),
        "echo_pub": summary.get("echo_pub"),
        "res_rx": summary.get("res_rx"),
        "res_tx": summary.get("res_tx"),
        "res_drop": summary.get("res_drop"),
        "est_rx": summary.get("est_rx"),
        "est_enq": summary.get("est_enq"),
        "est_done": summary.get("est_done"),
        "est_fail": summary.get("est_fail"),
        "est_drop": summary.get("est_drop"),
        "wr_ok": summary.get("wr_ok"),
        "wr_fail": summary.get("wr_fail"),
        "cir_ok": (summary.get("cir") or {}).get(
            "ok", summary.get("cir_ok")),
        "cir_fail": (summary.get("cir") or {}).get(
            "fail", summary.get("cir_fail")),
        "tx_send_error": summary.get("tx_send_error"),
    }

    # Resampler service: explicit summary keys first, else per-pulse CIR meta.
    res_service = {}
    for k in RES_SERVICE_SUMMARY_KEYS:
        if k in summary:
            res_service[k] = summary[k]
    res_per_pulse = cir_service.get("resample_us")

    # Estimator service: per-pulse estimator_us percentiles (preferred) plus
    # the summary mean/max as a stable aggregate.
    est_service = {
        "summary_mean_us": summary.get("est_service_us_mean"),
        "summary_max_us": summary.get("est_service_us_max"),
        "stdout_mean_us": (stdout["est_service"] or {}).get("mean_us"),
        "stdout_max_us": (stdout["est_service"] or {}).get("max_us"),
        "per_pulse_estimator_us": cir_service.get("estimator_us"),
        "per_pulse_queue_delay_us": cir_service.get("queue_delay_us"),
    }

    schedule_wall = summary.get("schedule_wall_s")
    if schedule_wall is None:
        schedule_wall = stdout["schedule_wall_s"]

    def _rate_per_s(count):
        if count is None or not schedule_wall:
            return None
        return float(count) / float(schedule_wall)

    throughput = {
        "echo_ok_per_s": _rate_per_s(counts.get("echo_ok")),
        "cir_ok_per_s": _rate_per_s(counts.get("cir_ok")),
        "wr_ok_per_s": _rate_per_s(counts.get("wr_ok")),
        "est_done_per_s": _rate_per_s(counts.get("est_done")),
    }

    return {
        "label": label,
        "run_dir": run_dir,
        "timing_file": timing_path,
        "stdout_file": stdout_path,
        "summary_file": os.path.join(run_dir, "summary.json"),
        "rate_hz": rate,
        "steps": steps,
        "publisher": publisher,
        "publisher_source": pub_sources,
        "publisher_n_lines": stdout["pub_n_lines"],
        "publisher_samples": pub_samples,
        "service": {
            "resampler": res_service or None,
            "resampler_per_pulse_us": res_per_pulse,
            "estimator": est_service,
        },
        "counts": counts,
        "schedule_wall_s": schedule_wall,
        "throughput": throughput,
        "timing": {
            "n": timing["n"],
            "ok": timing["ok"],
            "late": timing["late"],
            "status_hist": timing["status_hist"],
            "span_s": timing["span_s"],
            "lead_s": timing["lead_s"],
            "dt_tx_s": timing["dt_tx_s"],
        },
        "stdout_live_last": stdout["live_last"],
        "stdout_sched_last": stdout["sched_last"],
        "stdout_radio_ok": stdout["radio_ok"],
        "summary": summary,
    }


def discover_cases(path):
    """Return ``[(run_dir, stdout_path), ...]`` for a run or tag directory."""
    path = os.path.abspath(path)
    if not os.path.isdir(path):
        raise SystemExit("not a directory: %s" % path)
    has_artifacts = (
        os.path.isfile(os.path.join(path, "echo_timing.jsonl"))
        or os.path.isfile(os.path.join(path, "timing.jsonl"))
        or os.path.isfile(os.path.join(path, "summary.json")))
    if has_artifacts:
        return [path]
    cases = []
    for name in sorted(os.listdir(path)):
        sub = os.path.join(path, name)
        if not os.path.isdir(sub):
            continue
        if (os.path.isfile(os.path.join(sub, "echo_timing.jsonl"))
                or os.path.isfile(os.path.join(sub, "summary.json"))):
            cases.append(sub)
    if not cases:
        raise SystemExit("no run artifacts under %s" % path)
    return cases


# ---------------------------------------------------------------------------
# rendering
# ---------------------------------------------------------------------------

def _fmt(v, nd=3):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if not math.isfinite(v):
            return "-"
        return ("%%.%df" % nd) % v
    return str(v)


def flat_metrics(case):
    """Flatten a case into an ordered metric -> value mapping."""
    m = {}
    c = case["counts"]
    m["rate_hz"] = case["rate_hz"]
    m["pulses"] = c.get("pulses")
    m["schedule_wall_s"] = case["schedule_wall_s"]
    m["echo_ok"] = c.get("echo_ok")
    m["echo_fail"] = c.get("echo_fail")
    m["echo_late"] = c.get("echo_late")
    m["est_drop"] = c.get("est_drop")
    m["est_fail"] = c.get("est_fail")
    m["res_drop"] = c.get("res_drop")
    m["cir_ok"] = c.get("cir_ok")
    m["cir_fail"] = c.get("cir_fail")
    m["wr_ok"] = c.get("wr_ok")
    tp = case["throughput"]
    m["echo_ok_per_s"] = tp.get("echo_ok_per_s")
    m["cir_ok_per_s"] = tp.get("cir_ok_per_s")
    m["wr_ok_per_s"] = tp.get("wr_ok_per_s")
    for key in ECHO_STEP_KEYS:
        blk = case["steps"].get(key, {})
        m["%s.mean" % key] = blk.get("mean")
        m["%s.p95" % key] = blk.get("p95")
        m["%s.p99" % key] = blk.get("p99")
        m["%s.max" % key] = blk.get("max")
    for key in PUB_KEYS:
        blk = case["publisher"].get(key, {})
        m["pub.%s.mean" % key] = blk.get("mean")
        m["pub.%s.p95" % key] = blk.get("p95")
        m["pub.%s.p99" % key] = blk.get("p99")
        m["pub.%s.max" % key] = blk.get("max")
    m["pub.n"] = case["publisher_n_lines"]
    est = case["service"]["estimator"]
    m["est_service_us_mean"] = est.get("summary_mean_us")
    m["est_service_us_max"] = est.get("summary_max_us")
    pp = est.get("per_pulse_estimator_us")
    m["est_p95_us"] = pp.get("p95") if pp else None
    m["est_p99_us"] = pp.get("p99") if pp else None
    res = case["service"].get("resampler_per_pulse_us")
    m["resample_us.mean"] = res.get("mean") if res else None
    m["resample_us.p99"] = res.get("p99") if res else None
    return m


def render_markdown(cases, title="UWB PDU throughput", params=None):
    lines = []
    lines.append("# %s" % title)
    lines.append("")
    lines.append("Generated: %s" % datetime.datetime.now().isoformat(
        timespec="seconds"))
    if params:
        lines.append("")
        lines.append("Fixed parameters: %s" % params)
    lines.append("")
    if not cases:
        lines.append("_no cases_")
        return "\n".join(lines) + "\n"

    # Summary table.
    lines.append("## Per-case summary")
    lines.append("")
    header = ["metric"] + [c["label"] for c in cases]
    rows = [
        ("rate_hz", "rate (Hz)"),
        ("pulses", "pulses"),
        ("schedule_wall_s", "schedule_wall_s"),
        ("echo_ok", "echo_ok"),
        ("echo_late", "echo_late"),
        ("est_drop", "est_drop"),
        ("cir_ok", "cir_ok"),
        ("cir_ok_per_s", "CIR/s"),
        ("echo_ok_per_s", "echo_ok/s"),
        ("est_service_us_mean", "est service mean (us)"),
        ("est_service_us_max", "est service max (us)"),
        ("est_p95_us", "est service p95 (us)"),
        ("pub.n", "publisher samples"),
    ]
    flats = [flat_metrics(c) for c in cases]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "---|" * len(header))
    for key, label in rows:
        vals = [_fmt(f.get(key)) for f in flats]
        lines.append("| %s | %s |" % (label, " | ".join(vals)))
    lines.append("")

    # Per-step mean/p99 tables.
    def step_table(keys, heading, prefix=""):
        lines.append("## %s" % heading)
        lines.append("")
        lines.append("| metric | " + " | ".join(c["label"] for c in cases)
                     + " |")
        lines.append("|" + "---|" * (len(cases) + 1))
        for key in keys:
            for agg in ("mean", "p95", "p99", "max"):
                name = "%s%s.%s" % (prefix, key, agg)
                vals = [_fmt(f.get(name)) for f in flats]
                if all(v == "-" for v in vals):
                    continue
                lines.append("| %s | %s |" % (name, " | ".join(vals)))
        lines.append("")

    step_table(ECHO_STEP_KEYS, "Echo per-burst host steps (ms)")
    step_table(PUB_KEYS, "Publisher stages (ms)", prefix="pub.")

    # Notes.
    lines.append("## Notes")
    lines.append("")
    lines.append("- Echo step stats are over successful bursts in "
                 "`echo_timing.jsonl`.")
    lines.append("- Publisher per-pulse lines are emitted only for "
                 "`pulse_id < 3` or every 100th pulse; `pub.n` is the "
                 "number of parsed lines.")
    lines.append("- `est service` comes from `summary.json` "
                 "(`est_service_us_mean/max`); percentiles come from "
                 "`cir.jsonl` `estimator_us` when present.")
    lines.append("- Resampler `resample_us` is reported only when the app "
                 "or CIR meta exposes it; otherwise it is `-`.")
    lines.append("")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# run mode
# ---------------------------------------------------------------------------

def repo_root():
    cur = os.path.abspath(os.path.dirname(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(cur, "testdata")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    return os.getcwd()


def git_commit():
    try:
        out = subprocess.check_output(
            ["git", "-C", repo_root(), "log", "--oneline", "-1"],
            text=True, stderr=subprocess.DEVNULL).strip()
        full = subprocess.check_output(
            ["git", "-C", repo_root(), "rev-parse", "HEAD"],
            text=True, stderr=subprocess.DEVNULL).strip()
        return full, out
    except (OSError, subprocess.CalledProcessError):
        return "", ""


def child_env():
    repo = repo_root()
    env = os.environ.copy()
    build_lib = os.path.join(repo, "gr-uwb", "build", "lib")
    build_test = os.path.join(repo, "gr-uwb", "build", "test_modules")
    ld = [build_lib]
    py = ["/usr/local/lib/python3.10/site-packages", "/tmp/opencode/uwbshim",
          build_test]
    for key, paths in (("LD_LIBRARY_PATH", ld), ("PYTHONPATH", py)):
        cur = [p for p in env.get(key, "").split(":") if p]
        env[key] = ":".join(paths + [p for p in cur if p not in paths])
    return env


def build_command(args, rate, out_dir):
    cmd = [
        sys.executable, os.path.join(repo_root(), args.app),
        "--args", args.device_args,
        "--output", out_dir,
        "--pri-s", repr(1.0 / float(rate)),
        "--pulses", str(args.pulses),
        "--preamble-length", str(args.preamble_length),
        "--pulse-shape", args.pulse_shape,
        "--gain-tx", str(args.gain_tx),
        "--gain-rx", str(args.gain_rx),
        "--tx-channel", str(args.tx_channel),
        "--rx-channel", str(args.rx_channel),
        "--tx-antenna", args.tx_antenna,
        "--rx-antenna", args.rx_antenna,
        "--cal-delay-native", str(args.cal_delay_native),
        "--timing-detail",
    ]
    if args.no_udp:
        cmd.append("--no-udp")
    if args.extra_args:
        cmd.extend(args.extra_args)
    return cmd


def run_one(args, rate, tag_dir):
    out_dir = os.path.join(tag_dir, "%ghz" % float(rate))
    stdout_path = os.path.join(tag_dir, "%ghz.stdout.log" % float(rate))
    os.makedirs(out_dir, exist_ok=True)
    cmd = build_command(args, rate, out_dir)
    print("[bench] run %g Hz: %s" % (float(rate), " ".join(cmd)), flush=True)
    env = child_env()
    env["UWB_UHD_BOOTSTRAPPED"] = ""  # let the app bootstrap fresh
    env.pop("UWB_UHD_BOOTSTRAPPED", None)
    t0 = time.monotonic()
    rc = None
    with open(stdout_path, "w", encoding="utf-8") as logf:
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, env=env, cwd=repo_root())
        assert proc.stdout is not None
        for line in proc.stdout:
            logf.write(line)
            logf.flush()
            sys.stdout.write(line)
            if args.timeout and time.monotonic() - t0 > args.timeout:
                print("[bench] timeout after %ss; terminating" % args.timeout,
                      file=sys.stderr, flush=True)
                proc.terminate()
                break
        rc = proc.wait()
    dt = time.monotonic() - t0
    print("[bench] %g Hz done rc=%s wall=%.1fs stdout=%s" % (
        float(rate), rc, dt, stdout_path), flush=True)
    return {"rate_hz": rate, "out_dir": out_dir, "stdout": stdout_path,
            "cmd": cmd, "rc": rc, "wall_s": dt}


def do_run(args):
    tag_dir = os.path.abspath(args.out_root)
    os.makedirs(tag_dir, exist_ok=True)
    rates = [float(x) for x in args.rates.split(",") if x.strip()]
    runs = []
    for rate in rates:
        runs.append(run_one(args, rate, tag_dir))
    full, oneline = git_commit()
    meta = {
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "git_commit": full,
        "git_commit_oneline": oneline,
        "python": sys.version,
        "device_args": args.device_args,
        "tx_channel": args.tx_channel,
        "rx_channel": args.rx_channel,
        "tx_antenna": args.tx_antenna,
        "rx_antenna": args.rx_antenna,
        "pulses": args.pulses,
        "preamble_length": args.preamble_length,
        "pulse_shape": args.pulse_shape,
        "gain_tx": args.gain_tx,
        "gain_rx": args.gain_rx,
        "cal_delay_native": args.cal_delay_native,
        "no_udp": args.no_udp,
        "rates": rates,
        "runs": runs,
    }
    with open(os.path.join(tag_dir, "run_meta.json"), "w",
              encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
        f.write("\n")
    write_tag_outputs(tag_dir, runs, meta)
    print("[bench] wrote %s" % os.path.join(tag_dir, "bench_summary.md"),
          flush=True)
    return 0


def do_parse_only(args):
    tag_dir = os.path.abspath(args.out_dir)
    run_dirs = discover_cases(tag_dir)
    # Keep the provenance written by --run (run_meta.json); only fall back to
    # a parse-only marker when there is none.
    meta = _read_json(os.path.join(tag_dir, "run_meta.json"))
    if not meta:
        meta = {"parse_only": True}
    write_tag_outputs(tag_dir, run_dirs, meta, cases=run_dirs)
    print("[bench] wrote %s" % os.path.join(tag_dir, "bench_summary.md"),
          flush=True)
    return 0


def write_tag_outputs(tag_dir, runs, meta, cases=None):
    if cases is None:
        cases = []
        for r in runs:
            run_dir = r["out_dir"] if isinstance(r, dict) else r
            stdout = r.get("stdout") if isinstance(r, dict) else None
            cases.append(build_case(run_dir, stdout_path=stdout))
    else:
        cases = [build_case(p) for p in cases]
    params = None
    if meta and not meta.get("parse_only"):
        params = ("preamble=%s, pulse-shape=%s, gain_tx=%s, gain_rx=%s, "
                  "no_udp=%s, pulses=%s, device=%s, tx_ch=%s, rx_ch=%s, "
                  "git=%s" % (
                      meta.get("preamble_length"), meta.get("pulse_shape"),
                      meta.get("gain_tx"), meta.get("gain_rx"),
                      meta.get("no_udp"), meta.get("pulses"),
                      meta.get("device_args"), meta.get("tx_channel"),
                      meta.get("rx_channel"),
                      meta.get("git_commit_oneline")))
    md = render_markdown(cases, params=params)
    with open(os.path.join(tag_dir, "bench_summary.md"), "w",
              encoding="utf-8") as f:
        f.write(md)
    payload = {
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "meta": meta,
        "cases": cases,
    }
    with open(os.path.join(tag_dir, "bench_summary.json"), "w",
              encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")
    return payload


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(
        description="M0 PDU-throughput benchmark for "
                    "x410_cg400_hrp_echo_cir.py")
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--parse-only", action="store_true",
                      help="report on existing logs; --out-dir may be a run "
                           "or tag directory")
    mode.add_argument("--run", action="store_true",
                      help="drive the app over --rates with fixed parameters")
    p.add_argument("--out-dir", default="",
                   help="--parse-only: run or tag directory")
    p.add_argument("--out-root", default=os.path.join("analysis_outputs",
                                                      "m0_baseline"),
                   help="--run: tag directory to create")
    p.add_argument("--app", default=DEFAULT_APP,
                   help="app path relative to the repo root")
    p.add_argument("--rates", default="100,200,500",
                   help="--run: comma-separated rates in Hz")
    p.add_argument("--pulses", type=int, default=1000,
                   help="--run: pulses per rate (same for every rate)")
    p.add_argument("--preamble-length", type=int, default=128)
    p.add_argument("--pulse-shape", default="minphase")
    p.add_argument("--gain-tx", type=float, default=50.0)
    p.add_argument("--gain-rx", type=float, default=60.0)
    p.add_argument("--cal-delay-native", type=float, default=334.0)
    p.add_argument("--device-args", default="addr=192.168.10.2")
    p.add_argument("--tx-channel", type=int, default=0)
    p.add_argument("--rx-channel", type=int, default=3)
    p.add_argument("--tx-antenna", default="TX/RX0")
    p.add_argument("--rx-antenna", default="RX1")
    p.add_argument("--no-udp", action="store_true", default=True)
    p.add_argument("--udp", dest="no_udp", action="store_false",
                   help="re-enable UDP (default is --no-udp)")
    p.add_argument("--timeout", type=float, default=900.0,
                   help="--run: per-rate wall-clock guard (s)")
    p.add_argument("--extra-args", nargs=argparse.REMAINDER, default=[],
                   help="--run: extra args appended to the app command")
    return p


def main():
    args = build_parser().parse_args()
    if args.run:
        return do_run(args)
    if not args.out_dir:
        raise SystemExit("--parse-only requires --out-dir")
    return do_parse_only(args)


if __name__ == "__main__":
    raise SystemExit(main())
