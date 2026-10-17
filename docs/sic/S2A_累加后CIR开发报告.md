# S2-A 累加后 CIR 开发报告

日期：2026-10-18（运行时间 2026-10-17/18）。分支 `feature/sic-sensing-accum`，
代码基线提交 `93c80b3e77c2caf53cdfab25e09683871806fad8`（本分支新增文件见第 12 节）。
设计/QA 契约（先写、冻结）见 **`docs/sic/S2A_累加后CIR评价设计与QA契约.md`**。
共同约定见 **`docs/sic/S2_并行开发共同约定.md`**，任务书见
**`docs/sic/OpenCode开发指示_S2A_累加后CIR评价.md`**。

本报告只写**真实跑出来的东西**。计划/命令标注“未跑”。效果不达标、结论为负、
以及只有诊断意义的部分都单独列出，不做修饰。

---

## 0. 结论摘要（TL;DR）

| 问题 | 结论 | 证据 |
|---|---|---|
| 同 N 的 SIC CIR 收益 | CFO=0：等于原始 IQ 抑制并随 ISR 线性增长，**28.4 dB @ISR10**、38.4 @ISR20、18.4 @ISR0，且与 N 无关 | 6.1 |
| N 增大的累加收益 | **不能假设额外的 10log10(N)**。CFO=0 时 SIC 残余完全相干（eta≈0.999）→ 累加收益 ≈0；CFO≠0 时 raw 得到 ~23 dB、SIC 只得到 ~10–11 dB | 6.2 |
| 残余是否相干 | 是。CFO=0：相干地板（不随 N 衰减）；CFO≠0：确定性相位斜坡（eta≈0.007 但 centered lag-1≈0.999，斜率 ±44 kHz），**不是随机噪声** | 6.3 |
| 弱目标是否受损 | SIC 后 |amp err| P95 ≤ 0.32 dB、峰移 0，168/168 组合全通过；raw 在 ISR20/CFO0 下 P95 5.2 dB、ISR10 four-path 2.1 dB | 6.4 |
| CFO 困难场景累加是否有效 | 部分有效但不能依赖：C_CIR 在 N≤8 略升（16.5→19.1），N=64 反而降到 3.6/14.7/25.1 dB；ISR0 远端独立区甚至 C_CIR=−1.06 dB（SIC 比 raw 差） | 6.5 |
| 经过实际接收机验证的部分 | receiver 层：SIC 把 target-to-background proxy 从 raw 的 8.7（ISR10/CFO0）恢复到 1.5e3（≈+22 dB），ISR20 的 raw 直接 **timing_failed 0/20**；receiver 层仍与固定算子分列 | 8 |

实现/QA/verifier/reject-QA 全部通过（第 9 节）；效果类验收**不设“必须额外 18 dB”**，
收益为 0 或变差的结论按原样保留。

---

## 1. 交付物与入口

| 交付物 | 路径 |
|---|---|
| 新评价驱动 app（真实 CLI，`--help/--smoke/--self-test`） | `gr-uwb/apps/uwb_sim_sic_accum.py` |
| 固定算子数学参考 + QA | `tools/sic/accum_metrics.py`（`--self-test`） |
| 独立 verifier + 拒绝 QA | `tools/sic/verify_s2a.py`（`--self-test`、`--reject-qa`） |
| 冻结契约 | `testdata/sic_accum/operator.json`、`testdata/sic_accum/cells.json` |
| README（命令、口径、归档策略） | `testdata/sic_accum/README.md` |
| 一键 QA | `testdata/sic_accum/qa_s2a.py`（三个 `--self-test` + 纯 NumPy 两径算子/ROI 端到端 + 可选 verifier/reject-QA） |
| MATLAB 对照（**未运行**） | `testdata/sic_accum/matlab/verify_s2a_cir.m`、`testdata/sic_accum/export_matlab_fixture.py`、`.../matlab/README.md` |
| 设计/QA 契约 | `docs/sic/S2A_累加后CIR评价设计与QA契约.md` |
| 图 | `docs/sic/figures/s2a/*.png`（同时在本方向 results） |
| 结果目录 | `gr-uwb/build-sic-accum/results/s2a/{smoke,calibration,acceptance,bypass,qa}` |

可复制命令（已跑）：

```bash
cd /home/junqima/workspace/uwb-gnuradio/.worktrees/sic-sensing-accum
env -u LD_LIBRARY_PATH cmake -S gr-uwb -B gr-uwb/build-sic-accum -DCMAKE_BUILD_TYPE=Release -DENABLE_DOXYGEN=OFF
env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-sic-accum -j2
python3 tools/sic/accum_metrics.py --self-test
env -u LD_LIBRARY_PATH python3 gr-uwb/apps/uwb_sim_sic_accum.py --self-test
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2a.py --self-test
env -u LD_LIBRARY_PATH python3 -u gr-uwb/apps/uwb_sim_sic_accum.py --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/smoke --smoke
env -u LD_LIBRARY_PATH python3 -u gr-uwb/apps/uwb_sim_sic_accum.py --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/calibration --seed-set calibration \
  --i-know-this-is-calibration --windows-per-cell 20 --archive-windows 8
env -u LD_LIBRARY_PATH python3 -u gr-uwb/apps/uwb_sim_sic_accum.py --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/acceptance --seed-set acceptance \
  --windows-per-cell 100 --archive-windows 8 --receiver --receiver-windows 20 --plots
env -u LD_LIBRARY_PATH python3 -u gr-uwb/apps/uwb_sim_sic_accum.py --build-dir gr-uwb/build-sic-accum \
  --output gr-uwb/build-sic-accum/results/s2a/bypass --bypass --windows-per-cell 1000 --archive-windows 8 --plots
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2a.py --run <run_dir> --verbose
env -u LD_LIBRARY_PATH python3 tools/sic/verify_s2a.py --run gr-uwb/build-sic-accum/results/s2a/acceptance --reject-qa
```

未跑：MATLAB 对照（`testdata/sic_accum/matlab/verify_s2a_cir.m`，本机无 MATLAB/Octave）；
native 491.52/UHD/FPGA；完整 Pd/Pfa 曲线；S2-B 的完整包解调/PER。

## 2. 环境与 provenance（实测）

* 库：`gr-uwb/build-sic-accum/lib/libgnuradio-uwb.so.1.0.0`，
  `uwb_python.cpython-310-x86_64-linux-gnu.so`；
  `ldd` 该模块实测：`libgnuradio-uwb.so.1.0.0 => <本 worktree>/gr-uwb/build-sic-accum/lib/...`
  （verifier 每条 run 都校验 `expected_lib` 在 worktree 内且出现在 ldd 中）。
* 几何：`n=157376`，`pre_guard=1997`，`sensing_roi=[1997,67021)`（校准/验收含
  `visibility_guard=1` 的 cell 时为 `[1997,67022)`），`observed_end=10126`。
* 冻结 SIC 参数实测进入 adapter：
  `support_mode=configured_nominal, online_validation_reps=0, fit_reps=8,
  validation_reps=8, k_paths=4, coarse_stride=14, density 0.40/0.25, post_tail=100`。
* 输入 hash（`run_manifest.json.inputs_hashed`，sha256 前 16 位）：
  `sic_core_h=997408bfb83e8132`、`sic_adapter_cc=bc44d232d9b750b8`、
  `sic_generator=f8d4645bb819bd12`、`sic_app=13f6ea6be16780d5`、
  `accum_metrics=645386b245969525`、`app=0739a4bc614df442`、
  `operator_contract=18ae992ff20d9bb9`、`cells_contract=caf1e0102b47eebd`、
  `target_roi_contract=7baf516fe185e4a5`。**未修改** SIC 共享核心/adapter/旧 QA。
* 忙闲：运行期间另一个 session（`sic-payload-collision`）在跑自己的矩阵，机器同载。
  因此本报告的 wall 时间是诊断值（smoke 2.7 s、calibration 46.9 s、
  acceptance 222.0 s、bypass 62.5 s），**不是性能验收**：未持
  `/tmp/uwb-sic-benchmark.lock`，`summary.timing.mode="diagnostic (no benchmark lock held)"`。
  本任务不宣称吞吐能力。

## 3. 固定算子与冻结坐标（实测验证）

算子：code-9 裸采样码，`origin=1997`，tap `[-32,128)`，`H[r,t]=Σ_m y[origin+r·1016+t+m]·conj(code[m])/64`，
三路同一算子、identity 相位、average 约定。

从公开调制器恢复 code 的实测证据（每 run 都跑，fail-closed）：

```
rank=127, resid_rel=2.69e-08, max_int_dev=8.43e-08, 非零 chip=64, chip_energy=64, ternary={-1,0,1}
```

弱目标 ROI 校准（校准专用 seed 11020000..11020004，每 run 都验证）：

```
target_peak_taps = [39,39,39,39,39]     (frozen target_tap=39)
direct_peak_taps = [2,2,2,2,2]          (公开因果脉冲中心 +2)
|H[39]|/|H[2]|   = 0.25006/0.25015/0.25001/0.24967/0.24986   (profile gain 0.25)
```

即路径增益比 0.25 与校准值一致到 0.13%，证明固定算子在幅度上也是线性的、未做任何
逐路缩放。receiver tap 域关系 `receiver_tap = fixed_offset + cir_pre(16)` 由实测
`zero_delay_tap=16, peak_tap=18` 确认（第 8 节）。

## 4. A1 smoke（nominal ISR10/SNR30/offset0/CFO0/single，8 窗，全归档）

| N | C_CIR full | C_CIR target | G_sic | G_raw | sic amp err P95 | raw amp err P95 |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 28.378 | 28.045 | 0.00 | 0.00 | 0.152 dB | 1.351 dB |
| 8 | 28.377 | 28.045 | −0.064 | −0.063 | 0.152 dB | 1.343 dB |
| 64 | 28.373 | 28.042 | −0.075 | −0.070 | 0.152 dB | 1.340 dB |

原始 IQ 实测抑制（sensing 前导区间）：28.30 dB（p05 28.28）。`eta(sic/all)=0.9989`，
`centered lag-1=0.399`。8 窗全部 committed、非 bypass、`uses_truth=False`。

## 5. A2 acceptance（12 cells × 100 窗，acceptance seed）

总计 1200/1200 committed、0 bypass、`uses_truth=0`、96 窗归档、wall 222.0 s（诊断；同期另一 session 满载）。
calibration（12 cells × 20 窗）与 acceptance 的 `C_CIR(N=64,full)` 逐 cell 差 ≤0.05 dB，
说明冻结口径稳定、无 acceptance 调参。

### 5.1 同 N 的 SIC 收益 `C_CIR(N)`（full 网格，全前导集 `[0,N)`）

| cell | N=1 | N=2 | N=4 | N=8 | N=16 | N=32 | N=64 | 独立区 I=1/I=8/I=56 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| isr0_cfo0_single | 18.39 | 18.39 | 18.39 | 18.39 | 18.39 | 18.39 | 18.38 | 18.39 / 18.39 / 18.38 |
| isr0_cfo−50k_single | 16.48 | 16.94 | 17.87 | 19.12 | 10.30 | 13.14 | **3.65** | 27.81 / 20.98 / **−1.06** |
| isr0_cfo+50k_single | 16.49 | 16.95 | 17.87 | 19.12 | 10.31 | 13.15 | **3.66** | 27.80 / 21.00 / **−1.05** |
| isr10_cfo0_single | 28.39 | 28.39 | 28.39 | 28.39 | 28.39 | 28.39 | 28.38 | 28.39 / 28.39 / 28.38 |
| isr10_cfo−50k_single | 26.83 | 27.29 | 28.22 | 29.56 | 21.02 | 23.22 | 14.66 | 38.45 / 31.87 / 9.42 |
| isr10_cfo+50k_single | 26.82 | 27.29 | 28.22 | 29.56 | 21.00 | 23.20 | 14.64 | 38.46 / 31.85 / 9.40 |
| isr20_cfo0_single | 38.39 | 38.39 | 38.39 | 38.39 | 38.39 | 38.39 | 38.38 | 38.39 / 38.39 / 38.38 |
| isr20_cfo−50k_single | 36.92 | 37.38 | 38.31 | 39.67 | 31.25 | 33.29 | 25.05 | 48.54 / 42.16 / 19.66 |
| isr20_cfo+50k_single | 36.91 | 37.38 | 38.31 | 39.66 | 31.24 | 33.27 | 25.04 | 48.54 / 42.14 / 19.65 |
| isr0_cfo+50k_four | 16.48 | 16.84 | 17.50 | 17.76 | 8.50 | 10.25 | **2.08** | 19.21 / 15.40 / **−2.72** |
| isr10_cfo0_four | 26.59 | 26.55 | 26.52 | 26.51 | 26.50 | 26.50 | 26.50 | 26.50 / 26.50 / 26.50 |
| isr10_cfo0_off0.4_single | 23.19 | 23.19 | 23.19 | 23.19 | 23.19 | 23.19 | 23.19 | 23.19 / 23.19 / 23.19 |

（图：`docs/sic/figures/s2a/ccir_vs_N_full.png`、`ccir_vs_N_target.png`）

要点：

* **CFO=0 的三档 ISR**：C_CIR ≈ 18.4/28.4/38.4 dB，且**与 N 完全无关**（差 <0.01 dB）。
  SIC 残余不随累加变化 → 相干地板（6.3）。ISR 提高 10 dB，C_CIR 提高约 10 dB，说明
  残余是一个**与 ISR 无关的固定误差项**，而不是按比例缩小的干扰。
* **CFO≠0**：C_CIR 在 N≤8 略微上升（16.5→19.1），随后**下降**，N=64 时只剩
  3.6/14.7/25.1 dB（ISR 0/10/20）。原因见 6.2：raw 自己随 N 降很多。
* **ISR0 + CFO 的远端独立区出现负收益**（−1.06 / −2.72 dB）：累加 56 个符号后
  SIC 输出的 CIR 残余比 raw 更大。如实保留为**负结果**。
* four-path（ISR10/CFO0）26.5 dB，比 single-path 28.4 dB 低约 1.9 dB；
  `offset=0.4 work sample` 的 cell 23.2 dB（可见性 guard=1 生效）。

### 5.2 累加收益 `G(N)`（**不是 SIC 增益**；同 start 嵌套窗）

| cell | G_raw(64) | G_sic(64) | 10log10(64)=18.06 |
|---|---:|---:|---|
| isr0/10/20_cfo0_single | −0.07 dB | −0.08 dB | 相干 → 无累加收益 |
| isr0_cfo±50k_single | 22.97 | 10.13 | 只有一半 |
| isr10_cfo±50k_single | 22.97 | 10.80 | 只有一半 |
| isr20_cfo±50k_single | 22.97 | 11.10 | 只有一半 |
| isr0_cfo+50k_four | 22.95 | 8.54 | 更差 |
| isr10_cfo0_four | −0.07 | −0.16 | 相干 |
| isr10_cfo0_off0.4_single | −0.07 | −0.07 | 相干 |

（图：`residual_vs_N.png`，含 1/N 参考线）

结论：**SIC 残余不会自动获得 10log10(N)**。CFO=0 时残余完全相干，累加毫无帮助；
CFO≠0 时 raw 因相位斜坡自相消得到 ~23 dB，SIC 残余只降 ~10–11 dB，因此**SIC 相对
收益随 N 缩小**。

### 5.3 残余相干性（逐符号 `E_sic[r,tap]`）

| cell | 分支 | eta (all) | eta 95% CI | centered lag-1 | lag-1 | 相位斜率 | 单窗逐符号 \|E_sic\|² 分布 |
|---|---|---:|---|---:|---:|---:|---|
| isr10_cfo0_single | sic | 0.9988 | [0.9986,0.9989] | 0.469 | 0.9996 | −15 Hz | r8=1.25e-3 … r63=1.26e-3（基本平坦，r0/median=0.98） |
| isr10_cfo0_single | raw | 0.9995 | [0.9995,0.9995] | 0.000 | 0.9997 | 0 Hz | — |
| isr10_cfo−50k_single | sic | **0.0074** | [0.0073,0.0073] | **0.9989** | 0.9989 | **−44144 Hz** | min 1.2e-4(r8) / median 1.2e-2 / max 6.7e-2(r63)：**跨符号不均匀，远端最差** |
| isr10_cfo−50k_single | raw | 0.0050 | [0.0051,0.0051] | 0.9997 | 0.9997 | −50004 Hz | — |
| isr10_cfo0_four | sic | 0.9760 | — | 0.094 | 0.9878 | −11 Hz | — |

关键点（对应任务书 §5 的警告）：

* CFO=0：`eta≈0.999` = **相干**（不是随机）；累加只会把固定误差原样保留，不衰减。
* CFO≠0：`eta≈0.007` 容易被误读成“随机”，但 **centered lag-1 = 0.999**、
  相位斜率 = ±44 kHz（raw 分支实测 −50004 Hz，正是注入 −50 kHz）→ 残余是
  **确定性相位斜坡**，不是噪声。**不能只看 eta 判定随机**。
* raw 分支的相位斜率实测 **−50004 Hz**，精确复现注入的 −50 kHz；SIC 分支的
  “表观斜率”实测 **−44144 Hz**，与注入不同——它是“真实干扰 − 模型重建”的差拍，
  不能单独当 CFO 误差读。
* 逐符号残余**不是 64 个等价独立符号**：CFO≠0 时 r=8 处最小（1.2e-4），到 r=63
  涨到 6.7e-2（幅度比 ~24×）。N=1 只取 1 个符号会得到误导性结论，故本报告同时给出
  全 64 个单符号的分布与独立区 `[8,8+N)`。

### 5.4 弱目标保护（delay 37、gain 0.25；ROI 冻结，不在 SIC 结果上重搜）

* 全部 **168/168**（12 cells × (7+7) 个 N×集合 组合）组合通过目标门：
  `|amp err| P95 ≤ 1 dB` 且 `|peak shift| ≤ 1 work sample`。
* N=64 实测（sic）：|amp err| P95 = 0.15 dB（CFO0 cells）/ 0.001–0.006 dB（CFO cells）/
  0.08 dB（four）/ 0.32 dB（off0.4）；**峰移全部为 0**。
* N=64 实测（raw，对照）：ISR20/CFO0 = 5.23 dB、ISR10/four = 2.10 dB、
  ISR10/off0.4 = 1.74 dB、ISR10/CFO0 = 1.34 dB → 干扰确实在损坏弱目标，SIC 把它压回
  ≤0.32 dB。
* 背景假峰比与 `target-to-background proxy`（明确分子/分母）逐 cell 记录在
  `summary.json.per_cell.<cell>.target`，**不是检测 SINR**。
（图：`target_vs_N.png`）

### 5.5 CFO 困难场景

以 ISR10/CFO±50k 为例（full 网格）：

| | N=1 | N=8 | N=16 | N=32 | N=64 |
|---|---:|---:|---:|---:|---:|
| C_CIR | 26.83 | 29.56 | 21.02 | 23.22 | 14.66 |
| G_sic | 0 | 5.14 | 7.39 | 11.18 | 10.80 |
| G_raw | 0 | 2.41 | 13.20 | 14.79 | 22.97 |

累加**在 N≤8 略微提升** SIC 相对收益（因为 raw 还没有自相消），随后 raw 迅速自相消、
SIC 残余因模型 CFO 误差受限而下降。SIC 的 8 符号 CFO 估计实测精度（`sic_cfo_hz_est`
均值 vs 注入）：

| cell | 估计均值 | 误差 |
|---|---:|---:|
| isr0_cfo∓50k | ∓47350 Hz | 2650 Hz |
| isr10_cfo∓50k | ∓49199 Hz | 801 Hz |
| isr20_cfo∓50k | ∓49751 Hz | 249 Hz |
| isr0_cfo+50k_four | 46908 Hz | 3092 Hz |

即 CFO 精度随 ISR 降低而变差（ISR0 时 2.65 kHz），在剩余 56 个符号上累积成远端残余，
与本报告“远端独立区 C_CIR 只有 9.4 dB 甚至负值”的观测一致，也与 S1 的“8 符号 CFO
精度限制远端深度”一致。

### 5.6 逐窗状态与失败

* 1200/1200 `committed`，`sic_uses_truth=0`，`apply_refit_count=0`（冻结模型不复拟合）。
* 原始 IQ 实测抑制（sensing 前导区间，mean / p05）：
  ISR0 18.3/18.3、ISR10 28.3/28.3、ISR20 38.3/38.3 dB；
  ISR0 CFO±50k 6.0/5.9、ISR10 16.4/16.2、ISR20 26.6/26.4；
  ISR0/four/CFO 4.1/4.0、ISR10/four 25.2/25.1、ISR10/off0.4 23.0/23.0 dB。
* `sic_fit_residual` 均值：ISR0≈0.48–0.56、ISR10≈0.09、ISR20≈0.010（均 < 0.8 门限）。
* `n_paths` 均值：single 3.00、four 4.00、off0.4 1.00；`sub_len=65025`（=64·1016+1）。
* SIC 核心耗时（本机、同载、work 网格 157376 样点，**诊断值**）：
  `core_total_us` 均值 3.9–5.4 ms/窗（ISR0 最慢），与 S1 报告的 core 量级一致；
  含 Python/PMT 包装的整窗 0.185 s/窗（1200 窗 222 s，含 receiver 20 窗/cell）。
  这不是吞吐验收，不宣称实时能力。

## 6. A3 bypass / 目标保护（1000 窗无干扰）

* 1000/1000 **bypassed**（`output_changed=0`，`out == y` 逐字节，verifier 断言）。
* 所有 N/ROI 的 `c_cir_flag` = `na`（1000/1000），**没有任何伪造的 C_CIR**。
* 弱目标：`|amp err| P95 = 0.0 dB`、峰移 0，门通过。
* `iq_suppression = N/A`（无干扰、无重叠）；`uses_truth=0`。

## 7. receiver 层结果（现有 `uwb.radar_cir_estimator`，原样运行）

每 cell 前 20 窗 × 3 路 × 4 个配置（`rx_default [10,64)=54`、`count8/16/32`）。
这是 block 自己的“末尾 N”语义，**与固定算子的 `[0,N)`/`[8,8+N)` 不同**，只作工程对照。

target-to-background proxy（receiver tap 域 ROI [51,60) / 背景 [80,116)），`rx_default` 均值：

| cell | clean | raw | sic | 备注 |
|---|---:|---:|---:|---|
| isr0_cfo0_single | 7.3e5 | 22.6 | 1.5e3 | +18.3 dB |
| isr10_cfo0_single | 7.1e5 | 8.7 | 1.5e3 | +22.4 dB |
| isr20_cfo0_single | 7.1e5 | **0/20 timing_failed** | 1.5e3 | raw 无法锁定时序 |
| isr0_cfo±50k_single | 7.0e5 | 3.3e3 | 7.2e3 | +3.4 dB |
| isr10_cfo±50k_single | 7.3e5 | 344 | 8.2e3 | +13.8 dB |
| isr20_cfo±50k_single | 7.1e5 | **0/20 timing_failed** | 8.9e3 | |
| isr0_cfo+50k_four | 7.1e5 | 4.2e3 | 5.8e3 | |
| isr10_cfo0_four | 7.1e5 | 10.2 | 1.05e3 | +20.1 dB |
| isr10_cfo0_off0.4_single | 6.9e5 | 8.0 | 607 | +18.8 dB |

* 240/2880 个 receiver 结果为 `timing_failed`，全部集中在 **3 个 ISR20 单径 cell 的 raw
  分支**（20 窗 × 4 配置 × 3 cell）；clean 与 sic 分支 20/20 全部 ok。
* SIC 后的 proxy 在 CFO=0 的三个 ISR 档几乎**相同**（1.5e3）→ 残余是与 ISR 无关的
  固定地板，与固定算子层 6.1 的结论一致（但两者度量不同，不能直接比数值）。
* receiver 侧 N 扫描（`count8/16/32/54`）对 CFO=0 几乎无影响（1528–1640）；
  对 CFO≠0 随 count 增大而改善（4213 → 8239），说明 receiver 自身的全局 CFO
  处理在这些场景下更受益于更长平均。

## 8. 独立验证与 QA（全部通过）

| 检查 | 结果 |
|---|---|
| `tools/sic/accum_metrics.py --self-test` | 16/16 PASS（qa1–qa6 六组，含 1/N 统计容差、相干地板、相位斜坡、6.0206 dB、sum/average、ROI 边界） |
| `gr-uwb/apps/uwb_sim_sic_accum.py --self-test` | 0 failures（含 seed 互斥/唯一、A2 网格、ROI 契约、聚合健壮性） |
| `tools/sic/verify_s2a.py --self-test` | 0 failures（稀疏 MF vs 暴力、码布局、eta/lag 参考） |
| `testdata/sic_accum/qa_s2a.py --run .../smoke` | `qa_s2a: 0 failure(s)`：三个 self-test + 合成两径窗（真实 code-9 chips）验证固定算子 peak=2/target=39/幅度比 0.25 + verifier/reject-QA |
| verifier：smoke / calibration / acceptance / bypass | 45 / 45 / 47 / 48 项检查，**failures = 0** |
| 拒绝 QA（对真实 acceptance run 做 14 种篡改） | **14/14 全部被拒** |
| `ctest -R "preamble_sic\|radar_cir"` | 5/5 通过（未改旧门槛） |
| 从 archive 独立复算 | 96 个归档窗的 `H_clean/H_raw/H_sic`、accumulation 表、目标、相干指标全部复现（容差 1e-9/1e-6） |
| 聚合复算 | `summary.json` 与 `cells_summary.csv` 由 `windows.jsonl` 逐项复算一致（N/A/±Inf/失败均入分母） |
| 数据纪律 | 每窗 `uses_truth=False`、`apply_refit_count=0`、seed 公式/集合/唯一性、archive 为“按序号前 8 窗”、库在 worktree |
| 重算 vs 归档 | verifier 从 IQ 重算的 `H` 与归档张量逐元素一致（max abs 差 ≤1e-9 相对） |

QA 日志：`gr-uwb/build-sic-accum/results/s2a/qa/00..07_*.txt`。

## 9. 未达效果、失败与未运行项（如实列出）

**失败/负结果（保留，不掩盖）**

1. CFO≠0 的远端独立区 `C_CIR(56)` 对 ISR0 为 **−1.06 dB**、four-path ISR0 为
   **−2.72 dB**：大 N 下 SIC 输出比 raw 更差。根因是 8 符号 CFO 估计误差
   （ISR0 实测 2.65 kHz）在 56 个符号上累积成远端残余，而 raw 的相位斜坡自相消。
2. `G_sic(64) ≈ 10–11 dB`，明显低于 10log10(64)=18.06 dB；即 **SIC 残余不是非相干噪声**，
   无法通过累加获得额外收益。
3. receiver 层在 ISR20/单径的 raw 分支 **timing_failed 0/20**（240 个非 ok 结果），
   receiver 自身无法在该条件下产出 CIR；本报告不把它算作 SIC 的功劳，只记录事实。
4. 未做：Pd/Pfa 曲线、完整包解调/PER（S2-B）、native/UHD/FPGA、连续 1 GS/s、
   多干扰源；**未跑 MATLAB 对照**（脚本与导出器已提供，本机无 MATLAB/Octave）。
5. 若把 receiver 层结果当“纯干扰残余”是错的：它经过同步/SFD 搜索/L2 归一化，
   与固定算子层不可混用——本报告始终分列。

**未实跑声明**（manifest `not_run`）：`matlab`、`native` 两项。

## 10. 回答任务书 §7 的六个问题

1. **同 N 的 SIC 收益多少？** 固定算子层：CFO=0 时 = 18.4/28.4/38.4 dB（ISR 0/10/20），
   与 N 无关；CFO=±50 kHz 时 N=1 为 16.5/26.8/36.9 dB，N=64 降到 3.6/14.7/25.1 dB；
   receiver 层 proxy 提升 18–22 dB（ISR≥10、CFO0）。
2. **N 增加的累加收益多少？** 不是 10log10(N)。CFO=0 时 ≈0（相干）；CFO≠0 时 raw 得
   ~23 dB、SIC 得 ~10–11 dB。
3. **残余是否相干？** 是。CFO=0 为相干地板；CFO≠0 为确定性相位斜坡
   （centered lag-1≈0.999），非随机。
4. **弱目标是否受损？** SIC 后未受损（|amp err| P95 ≤0.32 dB、峰移 0，168/168 通过）；
   raw 受损明显（ISR20/CFO0 达 5.2 dB，且 ISR20 raw 连时序都锁不住）。
5. **CFO 困难场景累加是否有效？** 在 N≤8 有一点效果（16.5→19.1 dB），更大 N 反而变差；
   ISR0 远端会出现负收益。8 符号 CFO 精度是瓶颈。
6. **哪些结论只属于固定算子诊断，哪些经过接收机验证？**
   * 固定算子诊断（主结论，含所有 C_CIR/NMSE/G/eta/逐符号残余/目标幅相）：第 5–6 节全部。
   * 经过实际接收机验证（工程对照）：SIC 把 raw 的 target proxy 提升 ~18–22 dB、
     ISR20 单径 raw timing_failed 而 SIC/clean 20/20 ok（第 8/7 节）。
     两者不可互相替代。

## 11. 复现与归档

* 结果目录：`gr-uwb/build-sic-accum/results/s2a/{smoke,calibration,acceptance,bypass,qa}`。
* 每 run：`run_manifest.json`（commit/dirty、库与 ldd、9 个输入 hash、几何、ROI、
  种子、归档策略、`not_run`）、`summary.json`（逐 cell 聚合）、`windows.jsonl`（逐窗）、
  `accum_table.csv`（逐窗×N×集合×ROI）、`cells_summary.csv`、`operator.json`/`cells.json`
  副本、`operator/operator_arrays.npz`（算子向量/chips/pulse taps/模板）、
  `archive/<cell>_<wid>.npz`（96 窗）、`templates/`、`figures/`。
* 归档限制（写入 summary 与 README）：只归档每 cell 前 8 窗，其余窗口无法独立复算；
  报告中的聚合统计覆盖全部窗口，但只有归档窗口能被 verifier 从数组重算。
* 跨 N 的同一窗是有意配对，不是独立样本；95% 区间按 window/seed 成组 bootstrap。

## 12. 提交与文件清单

新增/修改（本 worktree，未动父仓库与另一 session）：

```
gr-uwb/apps/uwb_sim_sic_accum.py                新增（评价驱动 CLI）
tools/sic/accum_metrics.py                      新增（固定算子数学 + QA）
tools/sic/verify_s2a.py                         新增（独立 verifier + 拒绝 QA）
testdata/sic_accum/operator.json                新增（冻结算子/ROI/N/种子契约）
testdata/sic_accum/cells.json                   新增（A2 12 cells）
testdata/sic_accum/README.md                    新增
testdata/sic_accum/export_matlab_fixture.py     新增（MATLAB fixture 导出，已跑）
testdata/sic_accum/matlab/verify_s2a_cir.m      新增（未运行）
testdata/sic_accum/matlab/README.md             新增
testdata/sic_accum/qa_s2a.py                    新增（一键 QA）
docs/sic/S2A_累加后CIR评价设计与QA契约.md         新增（A0）
docs/sic/S2A_累加后CIR开发报告.md                新增（本文件）
docs/sic/figures/s2a/*.png                      新增（5 张）
```

共享 SIC 核心/adapter 与旧 QA、`tools/sic/sic_reference.py` **未修改**。
`.gitignore`/`AGENTS.md` 的改动来自本轮任务交接文件，不是本实现引入的算法改动。

## 13. 后续（不在本轮范围，不自动推进）

* 8 符号 CFO 精度提升（更长观测或显式 refine）以改善 CFO 场景的远端累加收益；
  需要动共享核心，本任务不做（共同约定 §2）。
* 加长接收机侧可选 repetition / 逐符号 receiver CIR 输出后的 receiver N 扫描。
* MATLAB 对照的实际执行（另一台有 MATLAB 的机器）。
* Pd/Pfa 与完整包解调（S2-B 方向）。
