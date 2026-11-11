#!/usr/bin/env python3
"""Turn a run_sc16_round2_matrix.py output directory into report tables.

Ownership: main agent.  Reads only runs.csv (and therefore the raw per-run JSON
via the paths it records) so every number in the report can be re-derived.

Noise handling
--------------
The task book forbids calling a delta an improvement if it is inside the
measured noise.  For each metric we report the median of the valid runs and the
interquartile range (IQR) of each variant.  A delta is labelled "beyond noise"
only when the two IQRs do not overlap; otherwise it is "inside noise".
"""

import argparse
import csv
import statistics as st
import sys


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def num(row, key):
    v = row.get(key)
    if v in (None, "", "None"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def iqr(values):
    v = sorted(values)
    if not v:
        return (None, None)
    n = len(v)
    return (v[int(0.25 * (n - 1))], v[int(0.75 * (n - 1))])


def summarize(rows, config, control_tag, candidate_tag, metrics):
    out = []
    for metric in metrics:
        vals = {}
        for tag in (control_tag, candidate_tag):
            v = [
                num(r, metric)
                for r in rows
                if r["config"] == config
                and r["variant"] == tag
                and r.get("valid") == "True"
            ]
            vals[tag] = [x for x in v if x is not None]
        c, a = vals[control_tag], vals[candidate_tag]
        if not c or not a:
            out.append((metric, None, None, None, None, None, "no valid runs"))
            continue
        mc, ma = st.median(c), st.median(a)
        delta = 100.0 * (ma - mc) / mc if mc else 0.0
        lc, hc = iqr(c)
        la, ha = iqr(a)
        if hc is not None and la is not None and lc is not None and ha is not None:
            beyond = (hc < la) or (ha < lc)
            verdict = "beyond noise" if beyond else "inside noise"
        else:
            verdict = "n/a"
        out.append((metric, mc, ma, delta, (lc, hc), (la, ha), verdict))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--control-tag", default="control")
    ap.add_argument("--candidate-tag", default="candidate")
    ap.add_argument("--metrics", default="")
    args = ap.parse_args()

    rows = load(f"{args.outdir}/runs.csv")
    metrics = (
        [m.strip() for m in args.metrics.split(",") if m.strip()]
        if args.metrics
        else [
            "steady_window_us",
            "resample_us_mean",
            "resample_us_p95",
            "sic_us_mean",
            "e2e_us_mean",
            "startup_ready_ms",
            "cold_total_s",
            "drain_tail_ms",
            "rx_read_msps",
        ]
    )

    configs = []
    for r in rows:
        if r["config"] not in configs:
            configs.append(r["config"])

    print(f"# {args.outdir}: {args.control_tag} -> {args.candidate_tag}\n")
    for cfg in configs:
        n_valid = sum(1 for r in rows if r["config"] == cfg and r.get("valid") == "True")
        print(f"## {cfg}  (valid runs: {n_valid})\n")
        print("| metric | control median | candidate median | delta | control IQR | candidate IQR | verdict |")
        print("| --- | ---: | ---: | ---: | --- | --- | --- |")
        for metric, mc, ma, delta, ci, ai, verdict in summarize(
            rows, cfg, args.control_tag, args.candidate_tag, metrics
        ):
            if mc is None:
                print(f"| {metric} | - | - | - | - | - | {verdict} |")
                continue
            ci_s = f"{ci[0]:.6g}..{ci[1]:.6g}" if ci[0] is not None else "-"
            ai_s = f"{ai[0]:.6g}..{ai[1]:.6g}" if ai[0] is not None else "-"
            print(
                f"| {metric} | {mc:.6g} | {ma:.6g} | {delta:+.1f}% | {ci_s} | {ai_s} | {verdict} |"
            )
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
