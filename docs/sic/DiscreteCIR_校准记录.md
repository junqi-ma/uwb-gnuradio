# Discrete CIR 参数校准记录（独立 seed 3000000）

任务书禁止用 10 格验收窗口选择 λ/支持长度。本记录使用独立生成 seed
`3000000`、3 个校准格（`sync_base`、`payload_base`、`payload_four_frac`）各 2 个
源窗，离线 Python 原型（与 C++ 合同同一数学）扫描参数。评价量是整包支持区内真实
残余抑制（相对 `raw-clean`），仅用于固定参数，不是验收。

## 1. 初版连续支持（superseded）

初版连续支持 `[first,last]`（15 dB 阈值）+ ridge 在单径格可用，但会把 sensing
前导拟合进 h，部分 `sync_base` 窗口比旧路径低 2–8 dB；连续宽支持不是可接受方案。

## 2. 峰值组 active 支持（采用）

粗 CIR 找局部极大值，保留 `|cir| >= peak_floor*peak`，按幅度贪心选择、峰间距
`>= peak_min_spacing`，每个峰激活 `peak ± active_radius`。hull 连续，未激活 tap
为零。扫描：

- `peak_floor` ∈ {0.10, 0.15, 0.20, 0.25}
- `peak_min_spacing` ∈ {4,5,6,8}
- `active_radius` ∈ {1,2}

| 参数 | sync_base mean | payload_base mean | payload_four_frac mean |
|---|---:|---:|---:|
| floor 0.10/0.15, radius 1 | 40.84 | 55.58 | 25.79 |
| floor 0.10/0.15, radius 2 | 38.67 | 55.54 | 27.22 |
| floor 0.20/0.25, radius 1 | 40.84 | 55.58 | 15.05 |
| floor 0.20/0.25, radius 2 | 38.67 | 55.54 | 15.15 |

`peak_min_spacing` 在 4–8 内对本校准集无影响（真实峰间距大于 7）。floor≥0.20
会漏掉 `payload_four_frac` 的 51.2 样本弱路径（0.2 增益，约 -14 dB），four 格
从 25.8 降到 15.1 dB。radius=2 略改善四径，但把单径格从 40.8 降到 38.7；
radius=1 已足以表示 PCHIP 分数时延的局部核，并对弱目标更保守。

固定选择：`lag_lo=-8`、最大 `tap_count=64`、`peak_floor=0.15`、
`peak_min_spacing=5`、`active_radius=1`、`lambda=1e-6`。C++ 实现完成后用同一 seed
复核一次，不再根据 10 格验收结果改参数。

## 3. λ

`lambda` 在 `1e-8..1e-4` 范围内对 active 集合基本无差别；`1e-2` 明显偏置。
固定 `1e-6`（相对 Gram 平均对角线），只用于数值稳定和抑制高频过拟合。

校准脚本为一次性离线原型，不进入生产路径；复现命令与实现见
`DiscreteCIR_接口合同.md` 和最终开发报告。
