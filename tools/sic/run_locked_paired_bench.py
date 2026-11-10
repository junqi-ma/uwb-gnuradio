#!/usr/bin/env python3
import fcntl
import os
import re
import shutil
import subprocess
import sys
import numpy as np

LOCK_PATH = "/tmp/uwb-sic-benchmark.lock"
OLD_BUILD = "/home/junqima/workspace/uwb-gnuradio/.worktrees/sic-unified/gr-uwb/build-sic-unified-round2"
NEW_BUILD = "/home/junqima/workspace/uwb-gnuradio/.worktrees/sic-unified/gr-uwb/build-sic-unified-round3"

def run_bench(build_dir, dump_dir=None):
    exe = os.path.join(build_dir, "apps/bench_unified_timing")
    lib = os.path.join(build_dir, "lib")
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = lib
    if dump_dir:
        os.makedirs(dump_dir, exist_ok=True)
        env["SIC_DUMP_OUTPUT_DIR"] = dump_dir
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

        raw_match = re.search(r"RAW_TOTALS:\s+([0-9\.\s]+)", sec)
        if raw_match:
            data[policy]["raw_totals"] = [float(x) for x in raw_match.group(1).strip().split()]

        # Committed & Suppression
        comm_match = re.search(r"Committed:\s+(\w+)\s+\|\s+Suppression:\s+([0-9\.]+)\s+dB", sec)
        if comm_match:
            data[policy]["committed"] = comm_match.group(1)
            data[policy]["suppression_db"] = float(comm_match.group(2))

        # Sample finiteness & Output L2
        finite_match = re.search(r"OUTPUT_FINITE:\s+(\w+)", sec)
        if finite_match:
            data[policy]["output_finite"] = finite_match.group(1)
        l2_match = re.search(r"OUTPUT_L2:\s+([0-9\.]+)", sec)
        if l2_match:
            data[policy]["output_l2"] = float(l2_match.group(1))

        # Stages
        for line in sec.splitlines():
            line = line.strip()
            # Stage lines like: 1. estimate_channel         105.27      102.02      129.30
            m = re.match(r"([0-9]+\.\s+[a-zA-Z0-9_&/()\s-]+)\s+([0-9\.]+)\s+([0-9\.]+)\s+([0-9\.]+)", line)
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
            m_sub = re.match(r"-\s+([a-zA-Z0-9_&/()\s-]+)\s+([0-9\.]+)", line)
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

        old_dump = "/tmp/sic_bench_dump_old"
        new_dump = "/tmp/sic_bench_dump_new"
        shutil.rmtree(old_dump, ignore_errors=True)
        shutil.rmtree(new_dump, ignore_errors=True)

        print("1/4: Running Old baseline (run 1)...")
        out_old1 = parse_output(run_bench(OLD_BUILD, old_dump))

        print("2/4: Running New optimized (run 1)...")
        out_new1 = parse_output(run_bench(NEW_BUILD, new_dump))

        print("3/4: Running New optimized (run 2)...")
        out_new2 = parse_output(run_bench(NEW_BUILD, new_dump))

        print("4/4: Running Old baseline (run 2)...")
        out_old2 = parse_output(run_bench(OLD_BUILD, old_dump))

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

            # Pooled and batch P95 breakdown for TOTAL CORE TIME
            if "raw_totals" in out_old1[p] and "raw_totals" in out_old2[p] and "raw_totals" in out_new1[p] and "raw_totals" in out_new2[p]:
                merged_old = np.array(out_old1[p]["raw_totals"] + out_old2[p]["raw_totals"])
                merged_new = np.array(out_new1[p]["raw_totals"] + out_new2[p]["raw_totals"])
                old_batch_p95 = (out_old1[p]["TOTAL CORE TIME"]["p95"] + out_old2[p]["TOTAL CORE TIME"]["p95"]) / 2.0
                new_batch_p95 = (out_new1[p]["TOTAL CORE TIME"]["p95"] + out_new2[p]["TOTAL CORE TIME"]["p95"]) / 2.0
                old_pooled_p95 = np.percentile(merged_old, 95)
                new_pooled_p95 = np.percentile(merged_new, 95)

                print(f"  * OLD TOTAL: mean={np.mean(merged_old):.2f} us, P50={np.percentile(merged_old, 50):.2f} us, Batch P95={old_batch_p95:.2f} us, Pooled 100 P95={old_pooled_p95:.2f} us")
                print(f"  * NEW TOTAL: mean={np.mean(merged_new):.2f} us, P50={np.percentile(merged_new, 50):.2f} us, Batch P95={new_batch_p95:.2f} us, Pooled 100 P95={new_pooled_p95:.2f} us")

            # Check output waveform dump and sample-by-sample L2 diff
            pol_file = "preamble" if "Preamble" in p else "fullpacket"
            old_bin = os.path.join(old_dump, f"sic_out_{pol_file}.bin")
            new_bin = os.path.join(new_dump, f"sic_out_{pol_file}.bin")
            if os.path.exists(old_bin) and os.path.exists(new_bin):
                old_iq = np.fromfile(old_bin, dtype=np.complex64)
                new_iq = np.fromfile(new_bin, dtype=np.complex64)
                all_finite = bool(np.all(np.isfinite(new_iq)) and np.all(np.isfinite(old_iq)))
                diff = new_iq - old_iq
                old_norm = np.linalg.norm(old_iq)
                diff_norm = np.linalg.norm(diff)
                rel_diff = diff_norm / old_norm if old_norm > 0 else 0.0
                max_abs = np.max(np.abs(diff))
                print(f"  * Output Finiteness: {'100% PASS (all samples finite)' if all_finite else 'FAIL (nonfinite samples detected)'}")
                print(f"  * Output L2 Norm: Old={old_norm:.6f}, New={np.linalg.norm(new_iq):.6f}")
                print(f"  * Output Sample L2 RelDiff: {rel_diff:.2e} (max abs sample diff = {max_abs:.2e})")

if __name__ == "__main__":
    main()
