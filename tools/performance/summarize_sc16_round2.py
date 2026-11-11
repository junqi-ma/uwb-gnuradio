#!/usr/bin/env python3
"""Turn a run_sc16_round2_matrix.py output directory into report tables.

Ownership: main agent.  Reads only runs.csv (and therefore the raw per-run JSON
via the paths it records) so every number in the report can be re-derived.

Noise handling
--------------
The task book forbids calling a delta an improvement if it is inside the
measured noise.  Two descriptive tables are printed per configuration:

1. the pooled table: median of every valid run per variant, the delta of the
   medians, the per-variant interquartile range (IQR), and whether the two IQRs
   overlap.  This is a description of the marginal distributions only.

2. the paired ABBA table: the runner executes every pair in the order
   `[control, candidate, candidate, control]` (slots 0..3), so each pair yields
   one control mean `(slot0+slot3)/2` and one candidate mean
   `(slot1+slot2)/2` measured in the same time window on the same machine.
   The paired delta is `100*(cand-ctrl)/ctrl`; the table reports the number of
   complete groups, the median/min/max paired delta and a paired verdict.

Neither table is a statistical test: there is no confidence interval, no
significance level and no correction for multiple configurations.  In
particular "IQRs do not overlap" is not statistical significance, and a paired
median is a median of a handful of groups.
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


def as_int(row, key):
    """Read an integer CSV cell.  runs.csv stores pair/slot as strings."""
    v = row.get(key)
    if v in (None, ""):
        return None
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return None


# ABBA slot order written by run_sc16_round2_matrix.py: control, candidate,
# candidate, control.  Index into this list is the slot number.
ABBA_ORDER = ("control", "candidate", "candidate", "control")


def paired_groups(rows, config, control_tag, candidate_tag, metric):
    """Group valid rows by ABBA pair and return (groups, incomplete, excluded).

    `groups` is a list of `(pair, delta_pct)`, one per complete group.
    `incomplete` lists `(pair, reason)` for every pair that exists in runs.csv
    but does not provide slots 0..3 with the expected variants and a numeric
    metric value; such pairs are reported, never silently dropped.
    `excluded` counts valid rows in this config that could not be assigned to
    any pair (missing/garbled pair or slot number).
    """
    order = [control_tag if v == "control" else candidate_tag for v in ABBA_ORDER]
    by_pair = {}
    excluded = 0
    for r in rows:
        if r.get("config") != config:
            continue
        pair = as_int(r, "pair")
        slot = as_int(r, "slot")
        if pair is None or slot is None:
            if r.get("valid") == "True":
                excluded += 1
            continue
        by_pair.setdefault(pair, []).append(r)

    groups = []
    incomplete = []
    for pair in sorted(by_pair):
        slots = {}
        problems = []
        for r in by_pair[pair]:
            if r.get("valid") != "True":
                continue
            slot = as_int(r, "slot")
            if slot is None or not 0 <= slot < len(ABBA_ORDER):
                problems.append(f"invalid slot {r.get('slot')!r}")
                continue
            if slot in slots:
                problems.append(f"duplicate slot {slot}")
                continue
            slots[slot] = r
        if not problems:
            missing = [s for s in range(len(ABBA_ORDER)) if s not in slots]
            if missing:
                problems.append(
                    "missing valid slot(s) " + ",".join(str(s) for s in missing)
                )
            else:
                wrong = [
                    s
                    for s in range(len(ABBA_ORDER))
                    if slots[s].get("variant") != order[s]
                ]
                if wrong:
                    problems.append(
                        "unexpected variant at slot " + ",".join(str(s) for s in wrong)
                    )
        if problems:
            incomplete.append((pair, "; ".join(problems)))
            continue
        vals = [num(slots[s], metric) for s in range(len(ABBA_ORDER))]
        if any(v is None for v in vals):
            incomplete.append((pair, f"missing {metric} value"))
            continue
        ctrl = 0.5 * (vals[0] + vals[3])
        cand = 0.5 * (vals[1] + vals[2])
        if ctrl == 0:
            if all(v == 0.0 for v in vals):
                incomplete.append((pair, "metric is zero in every run"))
            else:
                incomplete.append((pair, "control mean is zero"))
            continue
        groups.append((pair, 100.0 * (cand - ctrl) / ctrl))
    return groups, incomplete, excluded


def paired_verdict(deltas):
    """Paired ABBA verdict over complete groups.

    Deliberately conservative and purely threshold based: a >=5% directional
    verdict needs the median to reach the threshold AND every complete group to
    move in the same direction.  No significance test is implied.
    """
    if not deltas:
        return "incomplete"
    med = st.median(deltas)
    if med <= -5.0 and all(d < 0.0 for d in deltas):
        return "paired decrease >=5%"
    if med >= 5.0 and all(d > 0.0 for d in deltas):
        return "paired increase >=5%"
    return "inside 5%"


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
    ap.add_argument(
        "--show-pairs",
        action="store_true",
        help="also print every per-pair ABBA delta in the paired table",
    )
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
    print(
        "Pairing: runs.csv slots 0..3 = [control, candidate, candidate, control];\n"
        "ctrl = mean(slot0, slot3), cand = mean(slot1, slot2), "
        "delta = 100*(cand-ctrl)/ctrl.\n"
        "Descriptive only: no confidence interval, no significance test, and a "
        "non-overlapping IQR is not significance.\n"
    )
    for cfg in configs:
        n_valid = sum(1 for r in rows if r["config"] == cfg and r.get("valid") == "True")
        print(f"## {cfg}  (valid runs: {n_valid})\n")
        print("### Pooled descriptive table\n")
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

        print("### Paired ABBA table (per-pair control vs candidate, then median)\n")
        print(
            "| metric | complete groups | excluded | paired median | paired min | paired max | per-group deltas | paired verdict |"
        )
        print("| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |")
        notes = {}
        for metric in metrics:
            groups, incomplete, excluded = paired_groups(
                rows, cfg, args.control_tag, args.candidate_tag, metric
            )
            for pair, reason in incomplete:
                notes.setdefault(reason, set()).add(pair)
            n_excl = len(incomplete) + (1 if excluded else 0)
            reasons = {reason for _, reason in incomplete}
            inc_s = str(n_excl)
            if excluded:
                inc_s += f" (+{excluded} unpaired row(s))"
            if n_excl and len(reasons) == 1:
                inc_s += f" ({reasons.pop()})"
            if not groups:
                print(f"| {metric} | 0 | {inc_s} | - | - | - | - | incomplete |")
                continue
            deltas = [d for _, d in groups]
            med = st.median(deltas)
            groups_s = (
                ", ".join(f"p{p}:{d:+.1f}%" for p, d in groups)
                if args.show_pairs
                else "-"
            )
            print(
                f"| {metric} | {len(deltas)} | {inc_s} | {med:+.2f}% | {min(deltas):+.2f}% "
                f"| {max(deltas):+.2f}% | {groups_s} | {paired_verdict(deltas)} |"
            )
        for reason, pairs in sorted(notes.items()):
            plist = ", ".join(str(p) for p in sorted(pairs))
            print(
                f"- excluded group(s) {plist}: {reason} "
                "(not used by the paired median)"
            )
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
