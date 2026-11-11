#!/usr/bin/env python3
"""
plot_demod_and_sic_iterations.py
Visualizes the execution latency reduction of UWB Demodulation and SIC
across successive optimization iterations and milestones.
"""

import os
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter, LogLocator, NullFormatter

# Set style
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.linewidth'] = 0.8

# Iteration labels and metadata
iterations = [
    "Iter 0\nBaseline",
    "Iter 1\nDiscrete CIR\n& VOLK",
    "Iter 2\nCMF/PHR\nFusion",
    "Iter 3\nSparse Pulse\n& Zero-Scan",
    "Iter 4\nVector Phase I\n(4-Lag DotProd)",
    "Iter 5\nVector Phase II\n(CIR/CFO SIMD)",
    "Iter 6\nVector Phase III\n(SFD Pre-Fold)"
]
x = np.arange(len(iterations))

# 1. Total SIC & Demod Latency (Full-Packet, in ms)
# Data source: docs/sic/ reports & AGENTS.md
sic_total_full_ms = np.array([21.833, 5.500, 2.782, 1.822, 1.142, 1.025, 0.798])
sic_total_preamble_ms = np.array([5.000, 2.400, 1.820, 1.250, 0.828, 0.635, 0.436])
uwb_demod_full_ms = np.array([7.035, 2.765, 1.657, 1.161, 0.780, 0.632, 0.472])
uwb_demod_preamble_ms = np.array([3.800, 1.650, 1.120, 0.720, 0.321, 0.303, 0.140])

# 2. Full-Packet SIC Component Breakdown (in ms)
# Breakdown: Comm Demod, Channel Est, Reconstruction & Mod, Subtraction & Validation
sic_demod_ms = uwb_demod_full_ms
sic_channel_est_ms = np.array([2.070, 0.892, 0.747, 0.215, 0.185, 0.105, 0.053])
sic_reconstruct_ms = np.array([12.133, 1.480, 0.250, 0.228, 0.110, 0.226, 0.064])
sic_subtract_val_ms = np.array([0.595, 0.363, 0.128, 0.218, 0.067, 0.062, 0.063])

# 3. UWB Demodulation Component Breakdown (in us)
# Breakdown: Timing, SFD & Tail, CFO, CIR/Softchip, PHR, Payload
demod_timing_us = np.array([1850.0, 900.0, 500.0, 380.0, 97.7, 95.5, 96.6])
demod_sfd_us    = np.array([1650.0, 750.0, 300.0, 340.0, 212.0, 210.0, 70.0])
demod_cfo_us    = np.array([35.0,   25.0,  15.0,  12.0,  8.5,   8.0,   8.1])
demod_cir_us    = np.array([2500.0, 700.0, 450.0, 160.0, 157.0, 156.5, 156.5])
demod_phr_us    = np.array([200.0,  85.0,  18.0,  20.0,  24.0,  24.3,  27.8])
demod_payload_us= np.array([800.0,  305.0, 208.0, 101.0, 100.0, 99.7,  102.4])

# Speedup ratios compared to Iter 0 baseline
speedup_sic = sic_total_full_ms[0] / sic_total_full_ms
speedup_demod = uwb_demod_full_ms[0] / uwb_demod_full_ms

# ==================== Create Figure ====================
fig = plt.figure(figsize=(16, 12), dpi=300)
gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.22, top=0.92, bottom=0.08, left=0.08, right=0.95)

# Color palettes
c_sic = '#1f77b4'       # Blue
c_sic_pre = '#17becf'   # Cyan
c_demod = '#ff7f0e'     # Orange
c_demod_pre = '#d62728' # Red
c_guide = '#7f7f7f'

# ----------------- Subplot (a): Overall Latency vs Iterations (Log Scale) -----------------
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(x, sic_total_full_ms, 'o-', color=c_sic, linewidth=2.4, markersize=8, label='Full-Packet SIC Total', zorder=5)
ax1.plot(x, sic_total_preamble_ms, 's--', color=c_sic_pre, linewidth=2.0, markersize=7, label='Preamble-Only SIC Total', zorder=4)
ax1.plot(x, uwb_demod_full_ms, '^-.', color=c_demod, linewidth=2.2, markersize=8, label='UWB Demodulation (Full-Packet)', zorder=5)
ax1.plot(x, uwb_demod_preamble_ms, 'd:', color=c_demod_pre, linewidth=1.8, markersize=6, label='UWB Demodulation (Preamble-Only)', zorder=4)

# Real-time reference lines
ax1.axhline(y=1.0, color='#d62728', linestyle='--', linewidth=1.3, alpha=0.8, label='1.0 ms Real-Time Threshold')
ax1.axhline(y=0.5, color='#2ca02c', linestyle=':', linewidth=1.4, alpha=0.9, label='0.5 ms Preamble-Only Goal')

ax1.set_yscale('log')
ax1.set_ylabel('Execution Time [ms] (Log Scale)', fontsize=12, fontweight='bold')
ax1.set_title('(a) Latency Reduction Trend across Iterations', fontsize=13, fontweight='bold', pad=10)
ax1.set_xticks(x)
ax1.set_xticklabels(iterations, fontsize=9.5)
ax1.set_ylim(0.1, 35)
ax1.yaxis.set_major_formatter(ScalarFormatter())
ax1.grid(True, which='both', linestyle='--', alpha=0.5)

# Value callouts on key milestones
ax1.annotate(f'{sic_total_full_ms[0]:.2f} ms', xy=(0, sic_total_full_ms[0]), xytext=(0.15, 23.5),
             arrowprops=dict(arrowstyle='->', color=c_sic, lw=1.2), fontsize=9.5, fontweight='bold', color=c_sic)
ax1.annotate(f'{sic_total_full_ms[-1]:.3f} ms\n(27.4x Speedup)', xy=(6, sic_total_full_ms[-1]), xytext=(5.1, 1.55),
             arrowprops=dict(arrowstyle='->', color=c_sic, lw=1.2), fontsize=9.5, fontweight='bold', color=c_sic)
ax1.annotate(f'{uwb_demod_full_ms[-1]:.3f} ms\n(14.9x Speedup)', xy=(6, uwb_demod_full_ms[-1]), xytext=(5.1, 0.22),
             arrowprops=dict(arrowstyle='->', color=c_demod, lw=1.2), fontsize=9.5, fontweight='bold', color=c_demod)

ax1.legend(loc='lower left', fontsize=9, frameon=True, framealpha=0.92)

# ----------------- Subplot (b): Speedup Factors across Iterations -----------------
ax2 = fig.add_subplot(gs[0, 1])
width = 0.36
rects1 = ax2.bar(x - width/2, speedup_sic, width, label='SIC Total Speedup (×)', color='#2b5c8f', edgecolor='black', linewidth=0.6, alpha=0.9)
rects2 = ax2.bar(x + width/2, speedup_demod, width, label='UWB Demod Speedup (×)', color='#e67e22', edgecolor='black', linewidth=0.6, alpha=0.9)

ax2.set_ylabel('Speedup Factor vs. Iter 0 Baseline', fontsize=12, fontweight='bold')
ax2.set_title('(b) Cumulative Acceleration Factor across Iterations', fontsize=13, fontweight='bold', pad=10)
ax2.set_xticks(x)
ax2.set_xticklabels(iterations, fontsize=9.5)
ax2.set_ylim(0, 32)
ax2.grid(True, axis='y', linestyle='--', alpha=0.6)

# Add values above bars
for r in rects1:
    h = r.get_height()
    ax2.text(r.get_x() + r.get_width()/2., h + 0.4, f'{h:.1f}×', ha='center', va='bottom', fontsize=8.5, fontweight='bold', color='#1d3f63')
for r in rects2:
    h = r.get_height()
    ax2.text(r.get_x() + r.get_width()/2., h + 0.4, f'{h:.1f}×', ha='center', va='bottom', fontsize=8.5, fontweight='bold', color='#b35400')

ax2.legend(loc='upper left', fontsize=10, frameon=True, framealpha=0.92)

# ----------------- Subplot (c): Full-Packet SIC Stage Breakdown -----------------
ax3 = fig.add_subplot(gs[1, 0])
bar_width = 0.55

p1 = ax3.bar(x, sic_demod_ms, bar_width, label='Comm Demod (comm_rx)', color='#ff9f43', edgecolor='white', linewidth=0.5)
p2 = ax3.bar(x, sic_channel_est_ms, bar_width, bottom=sic_demod_ms, label='Channel Est (estimate_channel)', color='#54a0ff', edgecolor='white', linewidth=0.5)
p3 = ax3.bar(x, sic_reconstruct_ms, bar_width, bottom=sic_demod_ms + sic_channel_est_ms, label='Reconstruction & Mod', color='#ee5253', edgecolor='white', linewidth=0.5)
p4 = ax3.bar(x, sic_subtract_val_ms, bar_width, bottom=sic_demod_ms + sic_channel_est_ms + sic_reconstruct_ms, label='Subtract & Validation', color='#10ac84', edgecolor='white', linewidth=0.5)

ax3.set_ylabel('SIC Stage Execution Time [ms]', fontsize=12, fontweight='bold')
ax3.set_title('(c) Full-Packet SIC Stage Composition Breakdown', fontsize=13, fontweight='bold', pad=10)
ax3.set_xticks(x)
ax3.set_xticklabels(iterations, fontsize=9.5)
ax3.set_ylim(0, 24)
ax3.grid(True, axis='y', linestyle='--', alpha=0.6)

# Annotate total on top of bars
for i in range(len(x)):
    total = sic_total_full_ms[i]
    ax3.text(x[i], total + 0.4, f'{total:.2f} ms', ha='center', va='bottom', fontsize=8.5, fontweight='bold')

ax3.legend(loc='upper right', fontsize=9.5, frameon=True, framealpha=0.92)

# ----------------- Subplot (d): UWB Demodulation Component Breakdown -----------------
ax4 = fig.add_subplot(gs[1, 1])

d1 = ax4.bar(x, demod_timing_us, bar_width, label='Preamble Timing (stage_timing)', color='#5f27cd', edgecolor='white', linewidth=0.5)
d2 = ax4.bar(x, demod_sfd_us, bar_width, bottom=demod_timing_us, label='SFD & Tail (stage_sfd)', color='#ff6b6b', edgecolor='white', linewidth=0.5)
d3 = ax4.bar(x, demod_cir_us, bar_width, bottom=demod_timing_us + demod_sfd_us, label='CIR Equalization / Soft-chip', color='#48dbfb', edgecolor='white', linewidth=0.5)
d4 = ax4.bar(x, demod_phr_us, bar_width, bottom=demod_timing_us + demod_sfd_us + demod_cir_us, label='PHR Decoding (stage_phr)', color='#1dd1a1', edgecolor='white', linewidth=0.5)
d5 = ax4.bar(x, demod_payload_us, bar_width, bottom=demod_timing_us + demod_sfd_us + demod_cir_us + demod_phr_us, label='Payload Demod (stage_payload)', color='#feca57', edgecolor='white', linewidth=0.5)
d6 = ax4.bar(x, demod_cfo_us, bar_width, bottom=demod_timing_us + demod_sfd_us + demod_cir_us + demod_phr_us + demod_payload_us, label='CFO Fitting (stage_cfo)', color='#8395a7', edgecolor='white', linewidth=0.5)

ax4.set_ylabel('Demod Execution Time [$\mu$s]', fontsize=12, fontweight='bold')
ax4.set_title('(d) UWB Demodulation Internal Stage Breakdown', fontsize=13, fontweight='bold', pad=10)
ax4.set_xticks(x)
ax4.set_xticklabels(iterations, fontsize=9.5)
ax4.set_ylim(0, 7800)
ax4.grid(True, axis='y', linestyle='--', alpha=0.6)

# Annotate total on top of bars
for i in range(len(x)):
    demod_tot = (demod_timing_us + demod_sfd_us + demod_cir_us + demod_phr_us + demod_payload_us + demod_cfo_us)[i]
    ax4.text(x[i], demod_tot + 120, f'{demod_tot:.0f} $\mu$s', ha='center', va='bottom', fontsize=8.5, fontweight='bold')

ax4.legend(loc='upper right', fontsize=9.0, frameon=True, framealpha=0.92)

# Figure super title
fig.suptitle('UWB Demodulation & SIC Latency Reduction Across Iterations and Optimization Milestones',
             fontsize=15, fontweight='bold', y=0.97)

# Save figure in docs/sic/figures/
out_png = 'docs/sic/figures/uwb_demod_and_sic_iteration_breakdown.png'
out_pdf = 'docs/sic/figures/uwb_demod_and_sic_iteration_breakdown.pdf'
plt.savefig(out_png, dpi=300, bbox_inches='tight')
plt.savefig(out_pdf, bbox_inches='tight')
plt.close(fig)

print(f"Generated successfully:\n  {out_png}\n  {out_pdf}")
