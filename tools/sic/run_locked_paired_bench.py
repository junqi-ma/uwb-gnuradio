#!/usr/bin/env python3
import fcntl
import os
import re
import subprocess
import sys
import numpy as np

LOCK_PATH = "/tmp/uwb-sic-benchmark.lock"
OLD_BUILD = "/home/junqima/workspace/uwb-gnuradio/.worktrees/sic-unified/gr-uwb/build-sic-unified"
NEW_BUILD = "/home/junqima/workspace/uwb-gnuradio/.worktrees/sic-unified/gr-uwb/build-sic-unified-round2"

def run_bench(build_dir):
    exe = os.path.join(build_dir, "apps/bench_unified_timing")
    lib = os.path.join(build_dir, "lib")
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = lib
    res = subprocess.run([exe], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
    return res.stdout

def parse_output(text):
    data = {}
    sections = text.split("=======================================================")
    for sec in sections:
        if "Benchmark:" not in sec:
            continue
        policy_match = re.search(r"Benchmark:\s+([^\n]+)", sec)
        if not policy_match:
            continue
        policy = policy_match.group(1).strip()
        data[policy] = {}

        # Committed & Suppression
        comm_match = re.search(r"Committed:\s+(\w+)\s+\|\s+Suppression:\s+([0-9\.]+)\s+dB", sec)
        if comm_match:
            data[policy]["committed"] = comm_match.group(1)
            data[policy]["suppression_db"] = float(comm_match.group(2))

        # Stages
        for line in sec.splitlines():
            line = line.strip()
            # Stage lines like: 1. estimate_channel         105.27      102.02      129.30
            m = re.match(r"([0-9]+\.\s+[a-zA-Z0-9_\(\)/\s-]+)\s+([0-9\.]+)\s+([0-9\.]+)\s+([0-9\.]+)", line)
            if m:
                name = m.group(1).strip()
                data[policy][name] = {
                    "mean": float(m.group(2)),
                    "p50": float(m.group(3)),
                    "p95": float(m.group(4))
                }
            # TOTAL CORE TIME
            m_tot = re.match(r"TOTAL CORE TIME\s+([0-9\.]+)\s+([0-9\.]+)\s+([0-9\.]+)", line)
            if m_tot:
                data[policy]["TOTAL CORE TIME"] = {
                    "mean": float(m_tot.group(1)),
                    "p50": float(m_tot.group(2)),
                    "p95": float(m_tot.group(3))
                }
            # Sub-stages like: - timing                  99.62
            m_sub = re.match(r"-\s+([a-zA-Z0-9_\(\)/\s-]+)\s+([0-9\.]+)", line)
            if m_sub:
                name = "-" + m_sub.group(1).strip()
                data[policy][name] = {
                    "mean": float(m_sub.group(2))
                }
    return data

def main():
    print(f"Acquiring lock {LOCK_PATH}...")
    with open(LOCK_PATH, "w") as lock_file:
        fcntl.flock(lock_file, fcntl.LOCK_EX)
        print("Lock acquired. Running old -> new -> new -> old paired benchmark...")

        print("1/4: Running Old baseline (run 1)...")
        out_old1 = parse_output(run_bench(OLD_BUILD))

        print("2/4: Running New optimized (run 1)...")
        out_new1 = parse_output(run_bench(NEW_BUILD))

        print("3/4: Running New optimized (run 2)...")
        out_new2 = parse_output(run_bench(NEW_BUILD))

        print("4/4: Running Old baseline (run 2)...")
        out_old2 = parse_output(run_bench(OLD_BUILD))

        print("Benchmark runs complete. Aggregating results...")

        # Policies
        policies = list(out_old1.keys())

        for p in policies:
            print(f"\n=======================================================")
            print(f"  Aggregated Results: {p}")
            print(f"  Old Suppression: {out_old1[p]['suppression_db']:.2f} dB, New Suppression: {out_new1[p]['suppression_db']:.2f} dB")
            print(f"-------------------------------------------------------")
            print(f"  {'STAGE':<26} | {'OLD MEAN':>10} | {'NEW MEAN':>10} | {'SPEEDUP':>8} | {'OLD P95':>10} | {'NEW P95':>10}")
            print(f"  {'-'*26}-+-{'-'*10}-+-{'-'*10}-+-{'-'*8}-+-{'-'*10}-+-{'-'*10}")

            stages = [
                "1. estimate_channel",
                "-timing",
                "-cfo",
                "-sfd (tail refit)",
                "-cir (equalizer)",
                "-phr",
                "-payload",
                "2. comm_rx (demod/sfd)",
                "3. reconstruct (FIR)",
                "4. subtract & validate",
                "TOTAL CORE TIME"
            ]

            for s in stages:
                if s not in out_old1[p] or s not in out_new1[p]:
                    continue
                old_m = (out_old1[p][s]["mean"] + out_old2[p][s]["mean"]) / 2.0
                new_m = (out_new1[p][s]["mean"] + out_new2[p][s]["mean"]) / 2.0
                speedup = (old_m / new_m) if new_m > 0 else 1.0

                old_p95_str = f"{((out_old1[p][s]['p95'] + out_old2[p][s]['p95'])/2.0):.2f}" if "p95" in out_old1[p][s] else "-"
                new_p95_str = f"{((out_new1[p][s]['p95'] + out_new2[p][s]['p95'])/2.0):.2f}" if "p95" in out_new1[p][s] else "-"

                print(f"  {s:<26} | {old_m:>10.2f} | {new_m:>10.2f} | {speedup:>7.2f}x | {old_p95_str:>10} | {new_p95_str:>10}")

if __name__ == "__main__":
    main()
