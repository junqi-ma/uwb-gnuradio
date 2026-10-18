"""Build the Obsidian research note's diagrams and evidence-based plots.

The plotted numbers come from the frozen A/B worktrees, not from a nested
`.worktrees/` directory inside whatever checkout contains this file. Pass
`--a-root` and `--b-root` to point elsewhere. `--out-dir` defaults to this
note's `assets/` and overwrites the committed figures; use another directory
when regenerating without replacing the frozen note.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle
import numpy as np

HISTORICAL_REPO = Path("/home/junqima/workspace/uwb-gnuradio")
NOTE_ASSETS = Path(__file__).resolve().parent / "assets"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--a-root", type=Path,
                    default=HISTORICAL_REPO / ".worktrees/sic-sensing-accum")
    ap.add_argument("--b-root", type=Path,
                    default=HISTORICAL_REPO / ".worktrees/sic-payload-collision")
    ap.add_argument("--out-dir", type=Path, default=NOTE_ASSETS)
    args = ap.parse_args(argv)
    for label, path in (("A", args.a_root), ("B", args.b_root)):
        if not path.is_dir():
            raise SystemExit(
                f"{label} source root does not exist: {path}\n"
                "Pass --a-root and --b-root. This script does not look for a "
                "nested .worktrees directory in the current checkout.")
    build(args.a_root.resolve(), args.b_root.resolve(), args.out_dir.resolve())


def build(A, B, OUT):
    OUT.mkdir(parents=True, exist_ok=True)
    ASRC = A / "gr-uwb/build-sic-accum/results/s2a/acceptance_rectification/summary.json"
    BSRC = B / "gr-uwb/build-sic-payload/results/s2b_matrix_rectification/matrix_cells.csv"
    ad = json.loads(ASRC.read_text())
    with BSRC.open() as f:
        br = list(csv.DictReader(f))
    plt.rcParams.update({"font.family": "Microsoft YaHei", "font.size": 11,
                         "axes.unicode_minus": False, "svg.fonttype": "path",
                         "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": "#f8fafc", "axes.facecolor": "#f8fafc"})
    BLUE, TEAL, RED, GOLD = "#2563eb", "#0f766e", "#dc2626", "#d97706"


    def save(fig, name):
        fig.savefig(OUT / (name + ".png"), dpi=180, bbox_inches="tight")
        fig.savefig(OUT / (name + ".svg"), bbox_inches="tight")
        plt.close(fig)


    fig, ax = plt.subplots(figsize=(13, 6))
    ax.set(xlim=(0, 13), ylim=(0, 6)); ax.axis("off")
    def box(x, y, w, h, text, color=BLUE):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.08",
                                   facecolor="white", edgecolor=color, linewidth=1.8))
        ax.text(x+w/2, y+h/2, text, ha="center", va="center", color="#0f172a")
    def arrow(xy, end):
        ax.annotate("", xy=end, xytext=xy,
                    arrowprops={"arrowstyle": "->", "color": "#64748b", "lw": 1.8})
    ax.text(0, 5.65, "同一 SIC 核心，两个接收方向", fontsize=20, weight="bold")
    box(.1, 3.25, 2.25, 1.1, "混合 IQ 窗口\ny = s + i + w")
    box(3.05, 3.25, 2.6, 1.1, "检测 + 前 8 符号估计\n时延 / CFO / 复数 CIR")
    box(6.35, 3.25, 2.3, 1.1, "冻结干扰模型\n重构 64 符号前导", TEAL)
    box(9.35, 3.25, 2.9, 1.1, "对齐后在 IQ 域相减\nz = y − î", TEAL)
    for start, end in [((2.35,3.8),(3.05,3.8)),((5.65,3.8),(6.35,3.8)),((8.65,3.8),(9.35,3.8))]:
        arrow(start,end)
    box(3.05, 1.05, 2.6, 1.1, "公开信息\ncode / 模板 / 粗 ROI")
    arrow((4.35,2.15),(4.35,3.25))
    box(7.4, .95, 2.25, 1.25, "A：恢复 sensing\nCIR / 累加 / 弱目标", TEAL)
    box(10.15, .95, 2.55, 1.25, "B：恢复通信\n完整解调 / PSDU / FCS", GOLD)
    arrow((10.8,3.25),(8.55,2.2)); arrow((10.8,3.25),(11.4,2.2))
    ax.text(.1,.15,"示意图｜估计失败或不满足门限时旁路；真值仅供离线评分，未知 payload 不参与重构。",fontsize=10,color="#475569")
    save(fig,"01_sic_method")

    fig, axs = plt.subplots(2, 1, figsize=(13, 6.7))
    def segment(ax,x,w,y,label,color):
        ax.add_patch(Rectangle((x,y),w,.52,facecolor=color,edgecolor="white",alpha=.9))
        ax.text(x+w/2,y+.26,label,ha="center",va="center",color="white",fontsize=10)
    for ax in axs:
        ax.set(ylim=(-.2,2.1)); ax.set_yticks([])
        ax.spines[['left','bottom']].set_visible(False)
    axs[0].set_title("A｜通信前导与 sensing 前导时间重叠",loc="left",weight="bold")
    segment(axs[0],0,64,1.15,"sensing code 9：64 个 SYNC",BLUE)
    segment(axs[0],0,8,.35,"估计 8",GOLD)
    segment(axs[0],8,56,.35,"comm code 10：冻结模型延伸到剩余 56 个符号",TEAL)
    axs[0].set(xlim=(-1,65),xticks=[0,8,64],xlabel="Ipatov/SYNC 符号序号（示意：忽略成形尾部）")
    axs[1].set_title("B｜完整感知前导落在通信 payload 内，训练区也被 payload 覆盖",loc="left",weight="bold")
    fs=998.4; sync=65024/fs; sfd=8128/fs; phr=21504/fs; pay=116736/fs
    pstart=sync+sfd+phr; istart=pstart+(pay-65024/fs)/2
    segment(axs[1],0,sync,1.15,"comm SYNC",BLUE)
    segment(axs[1],sync,sfd,1.15,"",BLUE)
    segment(axs[1],sync+sfd,phr,1.15,"PHR",BLUE)
    segment(axs[1],pstart,pay,1.15,"未知通信 payload（含 FCS）",BLUE)
    segment(axs[1],istart,8128/fs,.35,"8",GOLD)
    segment(axs[1],istart+8128/fs,56896/fs,.35,"sensing code 9：后 56 符号",TEAL)
    axs[1].text(sync+sfd/2,1.85,"SFD",ha="center",fontsize=9)
    axs[1].set(xlim=(-2,215),xlabel="相对通信包起点 / μs（默认 center，未画 guard 与滤波拖尾）")
    fig.tight_layout(h_pad=2)
    save(fig,"02_collision_timeline")

    ns=[1,2,4,8,16,32,54]
    fig,axs=plt.subplots(1,2,figsize=(13,5))
    plot_rows=[]
    for cell,label,color,style in [("isr10_cfo0_single","ISR10 / CFO=0",TEAL,"-"),
                                  ("isr10_cfo-50k_single","ISR10 / CFO=−50 kHz",GOLD,"-"),
                                  ("isr0_cfo-50k_single","ISR0 / CFO=−50 kHz",RED,"--")]:
        vals=[ad['per_cell'][cell]['accum'][f'indep/{n}']['full']['mean'] for n in ns]
        axs[0].plot(ns,vals,style,marker='o',label=label,color=color,lw=2)
        plot_rows.extend(dict(cell=cell,N=n,cir_suppression_mean_db=v) for n,v in zip(ns,vals))
    axs[0].set(title="A：累加后的 CIR 抑制",xlabel="独立评价符号数 N",ylabel="C_CIR / dB（越高越好）")
    axs[0].legend(fontsize=9);axs[0].grid(alpha=.18)
    xs=np.arange(3); vals=[ad['per_cell'][f'isr{i}_cfo0_single']['accum']['indep/54']['full']['mean'] for i in (0,10,20)]
    axs[1].bar(xs,vals,color=[BLUE,TEAL,GOLD],width=.6)
    for x,v in zip(xs,vals):axs[1].text(x,v+.7,f'{v:.2f} dB',ha='center')
    axs[1].set(xticks=xs,xticklabels=['ISR 0 dB','ISR 10 dB','ISR 20 dB'],ylim=(0,44),
               title="零 CFO：54 符号评价",ylabel="C_CIR / dB")
    axs[1].grid(axis='y',alpha=.18)
    fig.suptitle("A / 感知恢复：每条件 100 窗；固定算子、独立支持区",fontsize=16)
    fig.tight_layout(rect=(0,0,1,.94));save(fig,'03_sensing_results')

    fig,axs=plt.subplots(1,2,figsize=(13,5))
    snrs=[-5.,-3.5,0.,4.,8.]
    for ax,isr in zip(axs,(10,20)):
        for path,label,col,marker in [('raw','raw：有干扰',RED,'x'),('clean','clean：无干扰',BLUE,'o'),('sic','SIC 后',TEAL,'s')]:
            rows=[next(r for r in br if r['cell']==f'b2_snr{s:.1f}_isr{isr}' and r['path']==path) for s in snrs]
            yy=np.array([float(r['per']) for r in rows]);lo=np.array([float(r['per_ci_lo']) for r in rows]);hi=np.array([float(r['per_ci_hi']) for r in rows])
            ax.errorbar(snrs,100*yy,yerr=100*np.maximum(0,np.array([yy-lo,hi-yy])),label=label,color=col,
                        marker=marker,linestyle='--' if path=='sic' else '-',markersize=8 if path=='clean' else 5,
                        markerfacecolor='none' if path=='clean' else col,capsize=3,lw=1.7)
        ax.set(title=f'ISR = {isr} dB',xlabel='样点 SNR / dB',ylabel='PER_PSDU / %',ylim=(-4,105),xticks=snrs)
        ax.grid(alpha=.18);ax.legend(fontsize=9)
    fig.suptitle('B / 通信恢复：单径、CFO=0；每点 1000 包；误差棒为 95% Wilson 区间',fontsize=14)
    fig.tight_layout(rect=(0,0,1,.94));save(fig,'04_communication_per')

    def source_path(path):
        resolved = path.resolve()
        try:
            return str(resolved.relative_to(HISTORICAL_REPO))
        except ValueError:
            return str(resolved)

    sources=[ASRC,BSRC,A/'docs/sic/S2A_整改开发报告_2026-10-18.md',B/'docs/sic/S2B_整改开发报告_2026-10-18.md']
    manifest={'date':'2026-10-18','description':'Derived plots, no new simulation; diagrams are explanatory.',
              'A_commit':subprocess.check_output(['git','-C',str(A),'rev-parse','HEAD'],text=True).strip(),
              'B_commit':subprocess.check_output(['git','-C',str(B),'rev-parse','HEAD'],text=True).strip(),
              'sources':[{'path':source_path(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sources],
              'A_plotted_points':plot_rows,'B_plotted_points':[r for r in br if r['cell'].startswith('b2_') and (r['cell'].endswith('isr10') or r['cell'].endswith('isr20'))]}
    (OUT/'figure_sources.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print('Created four figures in PNG + SVG, and figure_sources.json:',OUT)


if __name__ == "__main__":
    main()
