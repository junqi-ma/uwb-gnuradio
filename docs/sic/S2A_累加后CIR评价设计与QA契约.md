# S2-A 累加后 CIR 评价：设计与 QA 契约（A0）

日期：2026-10-17。基础提交 `93c80b3`；分支 `feature/sic-sensing-accum`。
本文件在实现与正式矩阵之前写成，冻结坐标、算子、指标与验收口径；
**结果与真实数据一律见 `docs/sic/S2A_累加后CIR开发报告.md`**，本文件不含效果数字（除
QA 自行构造的合成用例）。

---

## 1. 研究问题与范围

原始 IQ 上约 28 dB 的名义抑制（S0/S1 已测）不直接等于累加后的 sensing 收益。
本任务回答：

1. 相同 N 下 SIC 是否降低 CIR 干扰、保护弱目标（相对于 clean 的相对量）。
2. 随 N 增大，残余是**非相干衰减**（近 1/N，约 10log10(N)）还是被**固定模型
   误差/CFO 误差**钉成**相干地板**。
3. 弱目标（delay 37、gain 0.25）在各 N / 候选条件下是否受损。
4. CFO 困难场景（±50 kHz）累加是否有效、SIC 与 raw 的差别如何随 N 变化。
5. 哪些结论只属于**固定算子诊断**，哪些已经过**实际接收机**验证。

不在范围：新检测器、大规模 Pd/Pfa 扫描、核心（SIC 数学）优化、native 491.52/UHD/
FPGA/TWR、连续 1 GS/s、多干扰源、完整包解调 PER（那是 S2-B）。
本任务不改 `uwb_preamble_sic_core.{h,cc}`、不改现有 adapter、不改
`tools/sic/sic_reference.py`、不改其既有 QA 门槛。

## 2. 复用的现有实现（先追踪，再评价）

### 2.1 信号生成（不改算法，只 import）

`gr-uwb/apps/sic_preamble_sim.py::SicSignalGenerator`
（998.4 MS/s work 网格、`SPS=1016`、`sync_reps=64`、sensing code 9、comm code 10、
`4z2`；`pre_guard = llround(2 µs · 998.4 MHz) = 1997`）：

* `build(seed, comm_paths, start_offset_work, cfo_hz, isr_db, snr_db)`
  返回 `s`（clean sensing）、`i`（comm 前导，preamble-only 有限码脉冲串）、
  `w`（AWGN，`default_rng(seed)`）、`y = s+i+w`（CFO=0 时 `y` 与 `s+w` 同表达式）、
  `truth`（只在 sidecar）、`isr_db`（实测 ISR）、`overlap`。
* sensing 路径（**本端公开配置**，非注入真值）：`[[0,[1,0]],[37,[0.25,0]]]`。
* 公开模板：`generator.template = tx[:1016]`（code-9 第一个 SYNC 的成形波形），
  `generator.comm_sync/comm_wave`（code-10 有限 SYNC 码脉冲串）。

### 2.2 SIC（必须走真实 GNU Radio adapter）

`uwb.preamble_sic.make_from_templates(...)`（pybind）+ PDU 端口 `in/out/status/i_hat`，
由新 app 内一个极薄的 `SicRunner`（`gr.top_block` + `ResultSink`）驱动，参数取 S1 默认
冻结路径：`estimate8_frozen` / `support_mode=configured_nominal` / `fit_reps=8` /
`validation_reps=8` / `online_validation_reps=0` / `k_paths=4` / `coarse_stride=14` /
`detect=0.40` / `validate=0.25` / `post_tail=100` / `fit_residual_max=0.8`。
在线路径只拿到混合 IQ、公开 profile、公开粗搜索 ROI 与前缀可见性；**不读**注入起点/
CFO/路径/干扰 IQ/目标位置真值（`sic_uses_truth` 每窗必须为 False，verifier 强制）。
**不使用** `tools/sic/sic_reference.py` 的 NumPy SIC 冒充在线结果。

### 2.3 现有 sensing CIR 算法：实际用到哪些 repetition、相关方向、归一化、相位、CFO

追踪结论（C++ 与 MATLAB 一致）：

| 项 | 现有 receiver（`radar::estimate_radar_cir`，被 `uwb.radar_cir_estimator` 使用） | MATLAB `+uwbdecoder/estimateCir.m` |
|---|---|---|
| repetition 集合 | 由 block 构造参数决定；`uwb_sim_preamble_sic.py` 用 `cir_skip_initial=10, cir_repetitions=0 → 54`，即 `[10,64)` | `cir_repetitions` 默认 64，默认取末尾 N 个；caller 覆盖为 `preamble_repetitions-10`（QM35 64→54，DW1000 128→118） |
| 累加方式 | 先对 repetition **相干平均**（复数均值，1/N average），只做**一次**相关 | 完全相同：`averageSegment = mean(alignedSegments,2)`，再 `conv(averageSegment, codeMf, 'valid')` |
| 相关方向 | 与**共轭**裸码匹配滤波：`acc += a[nn+m]·conj(sampled_code[m])` | `codeMf = flipud(conj(sampled_code))`，`conv(·,codeMf,'valid')` |
| 归一化 | 先固定除 `code_energy`，再对整条 tap 向量做 L2 归一化（`norm_taps`），payload 仍给 `raw_taps` | 先除 `code_energy`，再 `values/(norm(values)+eps)`（只影响幅度标度） |
| 相位参考 | CIR 相位以**预测/精化 SYNC 原点**为参考；块内**不做任何 CFO 估计/补偿/refit** | CFO 在 CIR 之前**全局一次**去除（`compensateCarrierOffset`：相位斜率拟合 + 单常数相位），CIR 内**不再逐符号去相位** |
| 检测累加 | — | `detectRepeatedPreamble.m`：非相干 `|·|` 后按 16 个周期相加（与 CIR 相反） |
| 采样支撑 | `cir_pre=16, cir_post=100` → 116 taps；`zero_delay_tap = cir_pre` | `cir_pre_samples=8, cir_post_samples=30` 默认 |

实际阅读并引用的文件（**未运行 MATLAB**）：
`+uwbdecoder/buildUwbReference.m`（`sampled_code`/`code_energy`/`preamble_waveform` 三套
参考）、`+uwbdecoder/estimateCir.m`、`+uwbdecoder/estimateCirAndSoftChips.m`、
`+uwbdecoder/estimateCirAndPostDespreadPaths.m`、`+uwbdecoder/detectRepeatedPreamble.m`、
`+uwbdecoder/compensateCarrierOffset.m`、`+uwbdecoder/defaultOptions.m`、
`+uwbdecoder/packageResult.m`、`+uwbdecoder/plotDespreadCir.m`、`+uwbdecoder/constants.m`、
`decode_uwb.m`（阶段顺序：CFO 在 CIR 之前）、`estimate_uwb_cir_slow_phase.m`、
`analyze_qm35_cir_slow_phase.m`、`apply_estimated_cir_to_uwb.m`、caller 配置
`run_decode_uwb.m` / `run_decode_uwb_all.m` / `decode_scheduled_sc16_dump.m` /
`decode_uwb_all.m` / `run_decode_uwb_radar.m` / `run_decode_and_regenerate_uwb.m` /
`+uwbdecoder/mergeOptions.m`。

**由此得到的关键设计事实**：

* 现有 receiver 的累加集合是 `[10,64)`，**不是**本任务的 `[0,N)` 或 `[8,8+N)`，并且
  逐符号 CIR 只有在 `emit_individual_repetitions=True` 时才发布。
* 因此**固定算子层**必须自己构造（用公开 code-9 裸码模板），不能把 receiver 的 54 符号
  平均当作 N 扫描；**receiver 层的 N 扫描**只能用 block 自己的 `skip/count`（“末尾 N”
  语义），并单独标注。
* 现有 receiver 的 sensing CIR **不做 CFO refit**，所以注入 CFO 会以逐符号相位斜坡
  保留在 `H[r]` 中——这正是“相干地板 vs 1/N”的问题所在。

### 2.4 冻结的 SIC 核心与库

`uwb_preamble_sic_core.h` / `uwb_preamble_sic.cc` / `uwb_radar_cir_*` 本任务**不修改**；
构建产物绑定本 worktree `gr-uwb/build-sic-accum`（verifier 校验 ldd/模块路径与 hash）。

## 3. 冻结坐标与固定算子（`testdata/sic_accum/operator.json`）

* 时间原点 `origin = 1997`（公开 `pre_guard`，绝不用注入起点）。
* 模板：code-9 **裸采样码** `code[m]`（127 三值码 → ×4 扩频 → 每脉冲 2 样点，chip j 落在
  样点 `8j`），`code_energy = Σ|code|² = 64`。从公开调制器输出恢复：对一个成形 SYNC
  符号 + 公开脉冲 shaping taps 做最小二乘，要求 `rank=127, resid_rel≤1e-6,
  max_int_dev≤1e-3, 非零 chip=64, chip_energy=64`，否则 fail-closed。
* 相关：`H_x[r,t] = Σ_m y_x[origin + r·sps + t + m]·conj(code[m]) / code_energy`
  （与 MATLAB `estimateCir` / C++ `estimate_radar_cir` 同向、同归一化）。
* tap 网格：`t ∈ [-32, 128)`（160 taps）；三路**完全相同**。
* 相位旋转：**identity**（无逐符号、无逐路去相位）。诊断列另用**在线 SIC 估计的
  CFO**对三路做**同一**标量旋转，单独命名，不作为默认指标。
* 归一化：固定 `1/code_energy`；不做逐窗 L2/峰值/逐路缩放。
* 约定：`average`，`H_N = (1/N)Σ_{r∈R_N}H[r]`；同时记录 `sum` 换算
  `H_sum = N·H_avg`，`N=1..64` 幅度参考一致（同一算子）。
* repetition 集合：全前导 `R_N=[0,N)`，`N∈{1,2,4,8,16,32,64}`；
  独立区 `R_N=[8,8+N)`，`N∈{1,2,4,8,16,32,56}`；**禁止**把 `[8,72)` 当 64 个独立符号。
* ROI（冻结，校准集验证，不在 SIC 结果上重搜）：
  * target offsets `[35,44)`，`target_tap=39`（物理 delay 37 + 公开因果脉冲中心 tap +2）；
  * background offsets `[64,100)`；direct offsets `[-8,8)`（峰值应在 +2）。
  * receiver tap 域：R04 `target ROI [51,60)`、tap 55，两者关系
    `receiver_tap = fixed_offset + cir_pre(16)`。
* 冻结验证：用**校准专用** seed `11020000..11020004`（在每 cell 网格之外）跑
  clean CIR，要求 ROI 内 argmax 恒为 39、direct argmax 恒为 2、
  `|H[39]|/|H[2]| = 0.25±0.05`；不满足即 fail-closed（实际运行输出见报告）。

## 4. 指标定义

在固定 tap ROI（`full` = 全部 160 taps、`target`、`background`、`direct`）上：

* 残余：`E_raw,N = H_raw,N − H_clean,N`，`E_sic,N = H_sic,N − H_clean,N`。
* `C_CIR(N) = 10log10(‖E_raw,N‖² / ‖E_sic,N‖²)`（同 N 的 SIC 收益）。
* `NMSE_x = ‖H_x,N − H_clean,N‖² / ‖H_clean,N‖²`（目标/背景分开）。
* 累加收益 `G(N) = 10log10(P(1)/P(N))`，**同 start 嵌套窗**；raw 同样画；显式记录
  `g_is_sic_gain=false`（**这不是 SIC 增益**）。
* 相干性（逐窗、逐符号残余 `E_sic[r,tap]`）：`eta = ‖Σ_r E_r‖²/(N Σ_r‖E_r‖²)`；
  uncentered 与 centered lag 相关（lag 1..8）；均值/随机能量分解；
  主 tap 相位轨迹与线性斜率（Hz）。单窗统计与多窗 ensemble 分列。
* 弱目标：`|amp err|`（dB）、相位误差（deg）、ROI 内 argmax 峰移（work sample）、
  背景假峰比、`target-to-background proxy = |H[39]|²/mean_bg|H[tap]|²`
  （明确写入分子/分母，**不是**检测 SINR）。
* IQ 实测：`10log10(‖raw−clean‖²/‖sic−clean‖²)`，区间分别为
  `[origin, origin+64·sps)` 与整窗；原始 IQ 上的实际抑制必须保留。
* 零处理：精确零分子/分母记为 `+Inf/-Inf/N/A`（`c_cir_flag`），**不用 eps 兜底、
  不随意截顶**；`*_clipped` 仅用于画图，界限 `[-80,140] dB` 记录在契约里。

## 5. 数据 / 真值 / 重复性合同

* 三路配对：`clean=s+w`、`raw=s+i+w`、`sic=raw−i_hat`，**同一 payload、信道、
  采样坐标、噪声 realization、幅度尺度**；不逐路归一化。
* 种子：calibration base `11000000`，acceptance base `12000000`，
  `stride_per_cell=1000`，`seed = base + cell_index·1000 + window_index`；
  子流：noise=`default_rng(seed)`（`build` 中唯一 RNG）、payload=固定 PSDU（无 RNG）、
  path=确定性 profile（无 RNG）。calibration/acceptance 集合互斥且不重复（self-test
  与 verifier 双重检查）。
* 真值：注入起点/CFO/路径/干扰 IQ/目标位置只在 `fixtures`/archive sidecar 与离线
  verifier；在线路径与 SIC 均不可见。
* 归档：每 cell **按窗口序号固定取前 8 窗**（无任何指标驱动筛选），存
  `clean/raw/sic/ihat/i_truth/H_clean/H_raw/H_sic/truth_json/meta_json`；
  未归档窗口**不能**独立复算——`archive_policy` 明确写入 summary 与 README。
* 计划与真实结果分开：本文档只写计划/契约；命令标注“已跑/未跑”。

## 6. QA 契约

### 6.1 固定算子数学 QA（`tools/sic/accum_metrics.py --self-test`，16 项）

按指示 §5 的 1–7 项：完美相减（零残余、0/0 → N/A、无除零伪值）；无 SIC
（raw==sic → C_CIR=0 dB，非零能量）；人工逐符号独立零均值残余（多 seed ensemble 的
平均功率随 1/N 变化，**统计容差**非精确 3 dB）；每符号相同残余（average 后不衰减）；
固定相位斜坡（非单调积累、eta 低但 centered lag≈1，**不冒称随机**）；配对同噪声/
固定线性算子可加性、sum↔average 换算、全区/独立区边界；已知缩放比 → 6.0206 dB；
NMSE 手算一致。

### 6.2 app 自检（`gr-uwb/apps/uwb_sim_sic_accum.py --self-test`）

调用 6.1，并检查：seed 集合互斥/唯一、64 SYNC 支撑=65024、12 cells 唯一、
A2 网格 = 9+3、SNR 冻结 30 dB、ROI/receiver 映射与契约一致、聚合函数对
`inf/None` 健壮、verifier 存在。

### 6.3 独立 verifier（`tools/sic/verify_s2a.py`）

**不 import** app 或 `accum_metrics`，自行实现稀疏匹配滤波/累加/相干/目标指标：

1. 从 archive 的成形 SYNC + pulse taps **独立最小二乘**恢复 chips，重建裸码并与
   archive 的算子向量逐位比较。
2. 每个归档窗口：从 IQ 重算 `H_clean/H_raw/H_sic`，与归档张量比较。
3. 由归档张量重算 accumulation/目标/相干指标，与 `windows.jsonl` 比较。
4. 由 `windows.jsonl` 重算 `summary.json` 与 `cells_summary.csv` 的聚合
   （**失败/N/A/±Inf 全部入分母**）。
5. provenance/纪律：库在本 worktree、ldd 命中 build 库、每窗
   `uses_truth=False`、`apply_refit_count=0`、seed 公式与集合、archive 策略、
   `not_run` 声明、receiver 标注、bypass 无伪造 C_CIR。

### 6.4 拒绝 QA（`--reject-qa`，对真实 run 做 14 种篡改，每种必须被拒）

漏窗（错分母）、改 C_CIR 值、改归档 H 阵列、缩放归档 sic IQ、改算子 ROI、改 N 集合、
改 summary 均值、把库指到 worktree 外、删除 MATLAB 未跑声明、把在线窗标成用真值、
把 seed 改到集合外、删归档文件、改 `cells_summary.csv`、改 summary 的 seed_set。

### 6.5 回归

保持 `uwb_qa_uwb_preamble_sic_core.cc`、`uwb_qa_uwb_preamble_sic.cc`、
`uwb_qa_uwb_radar_cir_{core,estimator,estimator_block}.cc` 全绿（不改其门槛）。

## 7. 实验矩阵与交付物

* **A0**：本文件 + 6.1/6.2 QA。
* **A1**：nominal（ISR10/SNR30/offset0/CFO0/single）8 窗 smoke，完整数组归档。
* **A2**：acceptance 12 cells × 100 窗（ISR{0,10,20}×CFO{0,−50k,+50k} 共 9 + 另 3：
  ISR0/CFO+50k/four、ISR10/CFO0/four、ISR10/CFO0/offset0.4/single），SNR=30，
  所有 N 由同一窗派生；先 calibration（20 窗/cell）再 acceptance。
  receiver 层：`uwb.radar_cir_estimator` 原样运行，每 cell 前 20 窗 × 3 路 ×
  4 个 `(skip,count)` 配置（`[10,64)`=54、8、16、32），明确标注其“末尾 N”语义与
  固定算子 N 集合**不同**。
* **A3**：1000 窗无干扰 bypass / 目标保护（out==y 逐字节、无伪造 C_CIR）。
* **A4**：独立复算、verifier、图、报告、可复制命令、未跑清单。

交付物：`gr-uwb/apps/uwb_sim_sic_accum.py`、`tools/sic/accum_metrics.py`、
`tools/sic/verify_s2a.py`、`testdata/sic_accum/`（契约、README、MATLAB 对照脚本与
导出器）、`docs/sic/S2A_累加后CIR开发报告.md`、结果目录
`gr-uwb/build-sic-accum/results/s2a/{smoke,calibration,acceptance,bypass,qa}`。

## 8. 已知限制与诚实声明（实现前冻结）

* 固定算子层是**因果主评价**；receiver 层带自身同步/搜索/L2 归一化，**不是**纯干扰
  残余，两者分列。
* receiver 层无法给出与固定算子相同的 `[0,N)`/`[8,8+N)` 集合；其 N 扫描只覆盖
  block 自己的 `skip/count`，且 receiver 不做 CFO 补偿。
* `eta` 单靠值不能判定随机（相位斜坡会抵消）；必须结合 centered lag 与相位斜率。
* 弱目标 `target-to-background proxy` 不是检测 SINR；无 Pd/Pfa 曲线。
* MATLAB 对照脚本已提供但**未运行**（未实跑），不得据此声称 MATLAB 验证通过。
* 效果不达标（收益为 0、变差、或远低于 10log10(N)）也照实报告，不调门槛、不调 ROI、
  不剔除窗口。
