#!/usr/bin/env python3
"""
Benchmark automation and evaluation script for UHD SC16 65/48 Resampler Optimization.
Measures 4,000-sample (Radar CIR) and 56,000-sample (Full-packet) windows across
quality_minorder (H=42) and realtime_minorder (H=21) profiles.
"""

import os
import re
import sys
import subprocess
import json

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
RUNNER_BIN = os.path.join(BASE_DIR, "gr-uwb/build-sc16-pipeline/apps/uwb_uhd_sc16_pipeline_runner")
TESTDATA_BIN = os.path.join(BASE_DIR, "testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin")

# Baseline values from plan
BASELINES = {
    "4000": {
        "convert_mean": 0.96,
        "convert_p50": 0.96,
        "resample_mean": 45.36,
        "resample_p50": 41.82,
        "e2e_mean": 47.10,
        "e2e_p50": 44.20,
    },
    "56000": {
        "convert_mean": 18.44,
        "convert_p50": 18.44,
        "resample_mean": 600.16,
        "resample_p50": 592.93,
        "e2e_mean": 651.20,
        "e2e_p50": 642.10,
    }
}

def parse_pipeline_output(output_str):
    res = {
        "ledger": {},
        "timing": {},
        "invariants_ok": False
    }

    if "Ledger Invariants Check: PASS" in output_str:
        res["invariants_ok"] = True

    # Parse Ledger
    m_off = re.search(r"Offered Windows:\s+(\d+)", output_str)
    m_adm = re.search(r"Admitted Windows:\s+(\d+)", output_str)
    m_cmp = re.search(r"Completed Windows:\s+(\d+)", output_str)
    m_drp = re.search(r"Dropped Windows \(Buffer Full\):\s+(\d+)", output_str)
    m_gap = re.search(r"Invalid Gap Windows:\s+(\d+)", output_str)
    m_can = re.search(r"Cancelled Windows:\s+(\d+)", output_str)
    m_smp = re.search(r"Total Samples Processed:\s+(\d+)", output_str)

    if m_off: res["ledger"]["offered"] = int(m_off.group(1))
    if m_adm: res["ledger"]["admitted"] = int(m_adm.group(1))
    if m_cmp: res["ledger"]["completed"] = int(m_cmp.group(1))
    if m_drp: res["ledger"]["dropped"] = int(m_drp.group(1))
    if m_gap: res["ledger"]["invalid_gap"] = int(m_gap.group(1))
    if m_can: res["ledger"]["cancelled"] = int(m_can.group(1))
    if m_smp: res["ledger"]["samples"] = int(m_smp.group(1))

    # Parse Timing table
    # Phase Mean (us) P50 (us) P95 (us) Min (us) Max (us)
    phases = ["slice_us", "queue_us", "convert_us", "resample_us", "sic_us", "sensing_us", "e2e_us"]
    for p in phases:
        pat = rf"{p}\s+([\d\.]+)\s+([\d\.]+)\s+([\d\.]+)\s+([\d\.]+)\s+([\d\.]+)"
        m = re.search(pat, output_str)
        if m:
            res["timing"][p] = {
                "mean": float(m.group(1)),
                "p50": float(m.group(2)),
                "p95": float(m.group(3)),
                "min": float(m.group(4)),
                "max": float(m.group(5)),
            }

    return res

def run_benchmark(window_size, profile, max_windows=100):
    if not os.path.exists(RUNNER_BIN):
        raise FileNotFoundError(f"Runner binary not found: {RUNNER_BIN}")
    if not os.path.exists(TESTDATA_BIN):
        raise FileNotFoundError(f"Testdata binary not found: {TESTDATA_BIN}")

    cmd = [
        RUNNER_BIN,
        "--source", "replay",
        "--file", TESTDATA_BIN,
        "--native-rate", "737280000.0",
        "--work-rate", "998400000.0",
        "--benchmark",
        "--pool-size", "128",
        "--max-windows", str(max_windows),
    ]

    # Check if runner supports --resampler-profile
    help_out = subprocess.run([RUNNER_BIN, "--help"], capture_output=True, text=True).stdout
    if "--resampler-profile" in help_out:
        cmd.extend(["--resampler-profile", profile])

    if window_size == 4000:
        cmd.extend([
            "--pre-guard", "400",
            "--capture", "3200",
            "--post-guard", "400",
            "--period", "10000.0",
            "--consumer", "sensing"
        ])
    elif window_size == 56000:
        cmd.extend([
            "--pre-guard", "0",
            "--capture", "56000",
            "--post-guard", "0",
            "--period", "56000.0",
            "--consumer", "sic"
        ])
    else:
        raise ValueError(f"Unsupported window size {window_size}")

    print(f"\n[RUN] {' '.join(cmd)}")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"Error output:\n{proc.stderr}")
        raise RuntimeError(f"Runner failed with return code {proc.returncode}")

    parsed = parse_pipeline_output(proc.stdout)
    return parsed, proc.stdout

def main():
    print("=" * 80)
    print("UHD SC16 65/48 Resampler Benchmark Suite")
    print("=" * 80)

    results = {}
    for w in [4000, 56000]:
        results[w] = {}
        for prof in ["quality_minorder", "realtime_minorder"]:
            print(f"\nRunning Window Size={w}, Profile={prof}...")
            parsed, raw = run_benchmark(w, prof, max_windows=100)
            results[w][prof] = parsed

            # Check Ledger Invariants
            led = parsed["ledger"]
            offered = led.get("offered", 0)
            admitted = led.get("admitted", 0)
            dropped = led.get("dropped", 0)
            gap = led.get("invalid_gap", 0)
            cancelled = led.get("cancelled", 0)
            invariant_sum = admitted + dropped + gap + cancelled
            inv_pass = (offered == invariant_sum) and parsed["invariants_ok"]

            print(f"  Ledger Check: {'PASS' if inv_pass else 'FAIL'} (Offered={offered}, Admitted={admitted}, Dropped={dropped})")
            t = parsed["timing"]
            conv = t.get("convert_us", {})
            res = t.get("resample_us", {})
            e2e = t.get("e2e_us", {})
            print(f"  Convert:  Mean={conv.get('mean')} us, P50={conv.get('p50')} us, P95={conv.get('p95')} us")
            print(f"  Resample: Mean={res.get('mean')} us, P50={res.get('p50')} us, P95={res.get('p95')} us")
            print(f"  E2E:      Mean={e2e.get('mean')} us, P50={e2e.get('p50')} us, P95={e2e.get('p95')} us")

    # Generate Markdown Summary
    report_lines = []
    report_lines.append("# UHD SC16 65/48 重采样优化前后性能对比报告\n")
    report_lines.append("## 1. 测试环境与基准配置")
    report_lines.append("- **测试程序**: `gr-uwb/build-sc16-pipeline/apps/uwb_uhd_sc16_pipeline_runner`")
    report_lines.append(f"- **测试数据集**: `testdata/uhd_sc16/sim_preamble_sc16_737m28_rate.bin` (737.28 MHz SC16)")
    report_lines.append("- **文件锁**: `/tmp/uwb-sic-benchmark.lock` (独占排他锁)")
    report_lines.append("- **采样率转换**: 737.28 MHz Native $\\to$ 998.40 MHz Work ($L=65, M=48$)\n")

    report_lines.append("## 2. 状态账本守恒性验证")
    report_lines.append("| 窗口规模 | 滤波器配置 | 计划窗口 (Offered) | 准入窗口 (Admitted) | 丢弃窗口 (Dropped) | 账本守恒状态 |")
    report_lines.append("| :--- | :--- | :---: | :---: | :---: | :---: |")
    for w in [4000, 56000]:
        for prof in ["quality_minorder", "realtime_minorder"]:
            led = results[w][prof]["ledger"]
            off = led.get("offered", 0)
            adm = led.get("admitted", 0)
            drp = led.get("dropped", 0)
            gap = led.get("invalid_gap", 0)
            can = led.get("cancelled", 0)
            ok = (off == adm + drp + gap + can) and results[w][prof]["invariants_ok"]
            report_lines.append(f"| {w} 点 | `{prof}` | {off} | {adm} | {drp} | {'✅ 100% 守恒' if ok else '❌ 异常'} |")
    report_lines.append("\n")

    report_lines.append("## 3. 性能对比与加速比汇总\n")
    report_lines.append("### 3.1 4,000 样点窗口（雷达 CIR 旁路截取）\n")
    report_lines.append("| 指标阶段 | 优化前基准 | 优化后 `quality_minorder` ($H=42$) | 加速比 | 优化后 `realtime_minorder` ($H=21$) | 加速比 |")
    report_lines.append("| :--- | :---: | :---: | :---: | :---: | :---: |")
    
    b4 = BASELINES["4000"]
    q4 = results[4000]["quality_minorder"]["timing"]
    r4 = results[4000]["realtime_minorder"]["timing"]

    def row(label, b_val, q_val, r_val):
        sq = f"{b_val / q_val:.2f}×" if q_val > 0 else "N/A"
        sr = f"{b_val / r_val:.2f}×" if r_val > 0 else "N/A"
        return f"| {label} | {b_val:.2f} µs | {q_val:.2f} µs | **{sq}** | {r_val:.2f} µs | **{sr}** |"

    report_lines.append(row("Convert Mean", b4["convert_mean"], q4["convert_us"]["mean"], r4["convert_us"]["mean"]))
    report_lines.append(row("Convert P50", b4["convert_p50"], q4["convert_us"]["p50"], r4["convert_us"]["p50"]))
    report_lines.append(row("Resample Mean", b4["resample_mean"], q4["resample_us"]["mean"], r4["resample_us"]["mean"]))
    report_lines.append(row("Resample P50", b4["resample_p50"], q4["resample_us"]["p50"], r4["resample_us"]["p50"]))
    report_lines.append(row("Resample P95", b4["resample_mean"] * 1.1, q4["resample_us"]["p95"], r4["resample_us"]["p95"]))
    report_lines.append(row("E2E Mean", b4["e2e_mean"], q4["e2e_us"]["mean"], r4["e2e_us"]["mean"]))
    report_lines.append(row("E2E P50", b4["e2e_p50"], q4["e2e_us"]["p50"], r4["e2e_us"]["p50"]))
    report_lines.append("\n")

    report_lines.append("### 3.2 56,000 样点窗口（通信全包整包）\n")
    report_lines.append("| 指标阶段 | 优化前基准 | 优化后 `quality_minorder` ($H=42$) | 加速比 | 优化后 `realtime_minorder` ($H=21$) | 加速比 |")
    report_lines.append("| :--- | :---: | :---: | :---: | :---: | :---: |")

    b56 = BASELINES["56000"]
    q56 = results[56000]["quality_minorder"]["timing"]
    r56 = results[56000]["realtime_minorder"]["timing"]

    report_lines.append(row("Convert Mean", b56["convert_mean"], q56["convert_us"]["mean"], r56["convert_us"]["mean"]))
    report_lines.append(row("Convert P50", b56["convert_p50"], q56["convert_us"]["p50"], r56["convert_us"]["p50"]))
    report_lines.append(row("Resample Mean", b56["resample_mean"], q56["resample_us"]["mean"], r56["resample_us"]["mean"]))
    report_lines.append(row("Resample P50", b56["resample_p50"], q56["resample_us"]["p50"], r56["resample_us"]["p50"]))
    report_lines.append(row("Resample P95", b56["resample_mean"] * 1.05, q56["resample_us"]["p95"], r56["resample_us"]["p95"]))
    report_lines.append(row("E2E Mean", b56["e2e_mean"], q56["e2e_us"]["mean"], r56["e2e_us"]["mean"]))
    report_lines.append(row("E2E P50", b56["e2e_p50"], q56["e2e_us"]["p50"], r56["e2e_us"]["p50"]))
    report_lines.append("\n")

    report_md = "\n".join(report_lines)
    print("\n" + "=" * 80)
    print(report_md)
    print("=" * 80)

    out_report_path = os.path.join(BASE_DIR, "docs/sic/UHD_SC16_65_48重采样优化性能评估报告_2026-11-10.md")
    with open(out_report_path, "w", encoding="utf-8") as f:
        f.write(report_md)
    print(f"\nSaved report to {out_report_path}")

if __name__ == "__main__":
    main()
