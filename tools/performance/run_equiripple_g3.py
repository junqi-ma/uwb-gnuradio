#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G3: pipeline-level A/B of two 65/48 tap sets on the native-grid fixture.

What this can and cannot answer
-------------------------------
CAN:  run the production SC16 pipeline end to end with each tap set, compare
      every per-window decision field, and measure the waveform distortion the
      new filter actually introduces (group-delay shift, EVM, CIR peak
      position and amplitude).

CANNOT: measure SIC suppression, PSDU/FCS or packet error rate.  The
      repository's SIC scenario fixtures are generated on the 998.4 MS/s
      *work* grid (gr-uwb/apps/sic_preamble_sim.py), so using them would
      bypass the very resampler under test -- which the task book forbids.
      The only native-grid fixture does not trigger detection, so SIC
      suppression has no measurable value here.  Those items are reported as
      unverified rather than guessed.

Usage
-----
    python3 tools/performance/run_equiripple_g3.py \
        --binary gr-uwb/build-sc16-equiripple/apps/uwb_uhd_sc16_pipeline_runner \
        --out docs/performance/data_65_48_equiripple_g3
"""
import argparse
import csv
import json
import os
import subprocess
import sys

FIXTURE = "testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin"

# profile name -> (tap file that the pipeline resolves, T)
PAIRS = [
    dict(name="realtime", base="realtime_minorder", cand="realtime_eq16",
         t_base=1319, t_cand=1039),
    dict(name="quality", base="quality_minorder", cand="quality_eq32",
         t_base=2707, t_cand=2079),
]

DECISION_FIELDS = ["state", "sic_executed", "sic_committed", "sic_status",
                   "radar_executed", "radar_sfd_found", "radar_detected_start"]


def run(binary, profile, dump_dir, out_json, args, env):
    cmd = [binary, "--file", FIXTURE,
           "--native-rate", "737280000", "--work-rate", "998400000",
           "--period", str(args.period), "--capture", str(args.capture),
           "--max-windows", str(args.windows), "--consumer", "both",
           "--resampler-profile", profile,
           "--iq-dump-dir", dump_dir,
           "--iq-dump-windows", str(args.windows),
           "--json-out", out_json]
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if p.returncode != 0:
        raise SystemExit(f"pipeline run failed: {p.stderr[-1500:]}")
    return json.load(open(out_json))


def compare_waveforms(ref, test, t_ref, t_test, tmp_json):
    cmd = [sys.executable, "testdata/compare_resampler_waveforms.py",
           "--ref", ref, "--test", test,
           "--taps-ref", str(t_ref), "--taps-test", str(t_test),
           "--json", tmp_json]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0 or not os.path.exists(tmp_json):
        return None
    d = json.load(open(tmp_json))
    al = d["aligned"]
    x = d["xcorr_aligned"]
    return {"rel_l2": al["rel_l2"],
            "rel_l2_after_gain": al["rel_l2_after_gain"],
            "evm_pct_after_gain": al["evm_pct_after_gain"],
            "gain_db": al["fit_gain_db"],
            "xcorr_lag": x["peak_lag_samples"],
            "xcorr_corr": x["peak_corr"],
            "amp_db": x["amplitude_ratio_db"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--period", type=int, default=100000)
    ap.add_argument("--capture", type=int, default=56000)
    ap.add_argument("--windows", type=int, default=4)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    env = dict(os.environ)
    env.pop("LD_LIBRARY_PATH", None)

    rows, wave = [], []
    for pair in PAIRS:
        dirs = {}
        metas = {}
        for side in ("base", "cand"):
            prof = pair[side]
            d = os.path.join(args.out, f"{pair['name']}_{side}")
            os.makedirs(d, exist_ok=True)
            j = os.path.join(args.out, f"{pair['name']}_{side}.json")
            metas[side] = run(args.binary, prof, d, j, args, env)
            dirs[side] = d
            print(f"  ran {pair['name']} {side} ({prof})")

        wb = metas["base"]["windows"]
        wc = metas["cand"]["windows"]
        if len(wb) != len(wc):
            print(f"  WINDOW COUNT MISMATCH {len(wb)} vs {len(wc)}")
        for i in range(min(len(wb), len(wc))):
            diff = {f: (wb[i].get(f), wc[i].get(f))
                    for f in DECISION_FIELDS
                    if wb[i].get(f) != wc[i].get(f)}
            rows.append(dict(pair=pair["name"], window=i,
                             identical=not diff,
                             differing=json.dumps(diff) if diff else "",
                             sic_status_base=wb[i].get("sic_status"),
                             sic_status_cand=wc[i].get("sic_status"),
                             suppression_base=wb[i].get("actual_suppression_db"),
                             suppression_cand=wc[i].get("actual_suppression_db"),
                             detected_start_base=wb[i].get("radar_detected_start"),
                             detected_start_cand=wc[i].get("radar_detected_start")))

            rb = os.path.join(dirs["base"], f"win_{i}_dump.cf32")
            rc = os.path.join(dirs["cand"], f"win_{i}_dump.cf32")
            if os.path.exists(rb) and os.path.exists(rc):
                m = compare_waveforms(rb, rc, pair["t_base"], pair["t_cand"],
                                      os.path.join(args.out, f"cmp_{pair['name']}_{i}.json"))
                if m:
                    m.update(pair=pair["name"], window=i,
                             theoretical_delay=(pair["t_base"] - pair["t_cand"])
                             / (2.0 * 48.0))
                    wave.append(m)

    dp = os.path.join(args.out, "g3_decisions.csv")
    with open(dp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {dp} ({len(rows)} windows)")

    wp = os.path.join(args.out, "g3_waveform.csv")
    with open(wp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(wave[0].keys()))
        w.writeheader()
        w.writerows(wave)
    print(f"wrote {wp} ({len(wave)} windows)")

    n_id = sum(1 for r in rows if r["identical"])
    print(f"\ndecisions identical: {n_id}/{len(rows)}")
    for p in {r["pair"] for r in rows}:
        sub = [r for r in wave if r["pair"] == p]
        if not sub:
            continue
        print(f"  {p}: EVM {sum(x['evm_pct_after_gain'] for x in sub)/len(sub):.3f}% "
              f"amp {sum(x['amp_db'] for x in sub)/len(sub):+.4f} dB "
              f"lag {sorted({x['xcorr_lag'] for x in sub})} "
              f"corr {min(x['xcorr_corr'] for x in sub):.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
