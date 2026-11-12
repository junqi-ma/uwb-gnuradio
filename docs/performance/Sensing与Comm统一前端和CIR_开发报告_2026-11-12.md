# Sensing 与 Comm 统一接收前端和 CIR：开发报告

对应任务书 `docs/performance/OpenCode_Sensing与Comm统一前端和CIR_开发方案_2026-11-12.md`。
接口契约 `docs/performance/接口契约_统一SYNC_CIR核与公共前导前端_2026-11-12.md`。
性能报告 `docs/performance/Sensing与Comm统一前端和CIR_性能报告_2026-11-12.md`，
原始数据 `docs/performance/data_sensing_shared_cir_20261112/`。

- 分支 `feature/sensing-shared-cir`，worktree `.worktrees/sensing-shared-cir`
  （从 `068d21e` 创建；`068d21e` 只比基线 `bd311cd` 多一份任务书文档）。
- 新构建目录 `gr-uwb/build-sensing-shared-cir`；基线 build 与历史结果未覆盖。
- 未新增 GNU Radio block，未安装，未上板，未移动 tag，未推送。

## 1. 交付内容

### 1.1 共享 SYNC-CIR 内核（新文件）

`gr-uwb/include/gnuradio/uwb/uwb_sync_cir_core.h`，命名空间
`gr::uwb::demod::sync_cir`：

```cpp
bool prepare_sync_code(const int8_t* code, size_t code_len, size_t code_index,
                       size_t max_pre, size_t max_post,
                       PreparedSyncCode& prepared, SyncCirScratch& scratch);
bool required_input_range(const SyncCirRequest& req, size_t code_len, size_t n,
                          int64_t& lo, int64_t& hi);
bool estimate_sync_cir(const std::complex<float>* rx, size_t n,
                       const SyncCirRequest& req,
                       const PreparedSyncCode& prepared,
                       SyncCirScratch& scratch, SyncCirResult& out);
```

- 只依赖 `uwb_phy_profile.h` / `uwb_rx_sync_opt.h` / `uwb_rx_cfo_opt.h`，不反向
  依赖 demod/HRP 调制器/SIC，无 include 环。
- 复用 `rxopt::vector_accumulate_complex` / `vector_scale_complex` /
  `correlate_cir_sparse` / `build_sparse_cir_template`。
- 数学顺序与 comm 旧 `stage_cir_softchips` 完全一致（float 累加 → `1/valid`
  缩放 → 稀疏相关并 `/code_energy` → float `nrm` → `/(nrm+1e-12f)`），因此
  **comm 逐位不变**。`raw` 与 `norm` 分开保存，`raw` 保留真实幅度与相位。
- 热路径零分配；`avg` 按 `code_len + max_pre + max_post - 1` 预留。
- 失败关闭：`InvalidInput` / `CapacityInsufficient` / `NoValidRepetitions` /
  `NonFinite`，一律把 `tap_count` 清零，不返回残留 tap。

### 1.2 公共前导接收前端（`uwb_demod_core.h` 内）

`demodulate_one()` 的前导段（timing → CFO 初估 → bootstrap SFD → tail 重跟踪
+ CFO 复估 → final SFD）被整体抽成 `demod::acquire_preamble()`，`demodulate_one()`
改为调用它，**没有保留第二份等价实现**。

```cpp
enum class PreamblePurpose : uint8_t { SyncSfdOnly, CirOnly, FullPacket };
struct PreambleRxContext { /* timing/cfo/sfd、final_cfo_hz、三个原点、
                              covered_lo/hi、cfo_epoch、子计时 */ };
bool acquire_preamble(const std::complex<float>* rx, size_t n,
                      const Qm35825Profile& profile,
                      const std::vector<std::complex<float>>& template_wf,
                      int64_t rel_seed, PreamblePurpose purpose,
                      const sync_cir::SyncCirRequest* cir_req,
                      core::DemodScratch& scratch, PreambleRxContext& out,
                      double period_hint = 0.0);
```

**关闭的覆盖缺口（方案 §4 的关键缺口）**：此前 `rx_stop_after_sfd=true` 时最终
CFO 的第二次旋转只覆盖 final SFD 范围，较早的 SYNC 缓存仍是 bootstrap CFO 结果。
现在：

- `CirOnly` 由 `required_input_range()` 推导全部合法 CIR 输入窗口的并集，与 final
  SFD 区间取并集，再加 128 样点 halo，一次性按 **final CFO** 旋转；
- 覆盖不足（并集越界、无有效窗口、超过 4 Mi 样点上限）→ `CirFailed` 且
  `derotated_valid=false`，**不发布任何 stale-epoch 区间**；
- 旋转**始终从 raw `rx` 重新计算**，不在旧旋转结果上二次相乘；
- `cfo_epoch`：bootstrap=1、final=2，调用方必须校验 epoch 与
  `[covered_lo, covered_hi)` 覆盖所需区间；
- `phase_reference_sample` 固定为窗口内绝对样点 0（`dst[i]=src[i]·e^{-jwi}`），
  sensing 与 comm 共用同一约定。

**新增 `period_hint`**：方案 §2 要求「重复周期使用统一的显式配置/估计值 P」。
当 SYNC 跟踪无法测得周期时，调用方给出的显式 P 生效并置
`period_from_hint=true`；comm 传 0，因此 comm 的严格行为完全不变。

### 1.3 Sensing 接入（`uwb_radar_cir_core.h` / estimator / pipeline）

- `RadarCirConfig` 新增 `compensate_target_cfo`（默认 true）、`target_cfo_known`、
  `target_cfo_hz`。`Qm35825Profile` 在 `prepare_radar_cir_core()` 内由既有配置字段
  构建并冻结。
- `radar_cir_one()`：
  - `use_predicted_timing=false` → `acquire_preamble(CirOnly)` + 共享 CIR 内核；
  - `use_predicted_timing=true` 且 `target_cfo_known` → 直接按已知 CFO 补偿；
  - `use_predicted_timing=true` 且 CFO 未知 → 保留原无补偿行为，但显式记录
    `final_cfo_hz=0`、`cfo_epoch=0`，**不把「跳过 SFD」当成「CFO 为零」而不留痕**。
- 三个原点分别命名、分别保存：
  - `detected_sync_origin`（==`preamble_start_sample`）：前端检测到的 SYNC 原点；
  - `cir_axis_origin`（==`cir_origin_sample`）：`nominal_preamble_start(...)` 的
    发送时序/公开预测原点，**从不按检测峰重新居中**；
  - `phase_reference_sample`：去 CFO 的绝对相位锚点。
- raw/norm taps 从共享内核拷入既有 `scratch.cir.raw_taps/norm_taps`（预分配，
  热路径零分配），所有既有消费者不变。**raw 保留真实幅度**，不复用只有实部的
  `cir_values`，也不复用已归一化的 `cir_filter_taps`。
- `RadarCirResult` 新增 `final_cfo_hz`、`cfo_epoch`、`cir_samples_per_symbol`
  与 `frontend_timing_us/frontend_cfo_us/frontend_sfd_us/coverage_us/cir_us`。
- pipeline 把 provenance 写入 `WindowResult`（`radar_cfo_hz`、
  `radar_cir_origin`、`radar_detected_origin`、`radar_valid_reps`、
  `radar_frontend_us`、`radar_cir_us`），并 `reserve_radar_cir_core()` 预留前端
  scratch；burst 级多 worker、每 worker 私有状态与调度行为未改。
- `tools/sic/benchmark_chain_workers.cc` 删除了手工重实现 radar 三段的
  `detailed_radar()`，改为调用生产 `radar_cir_one()` 并读取其子计时，旧手工诊断
  路径不再绕开生产实现。

### 1.4 新增 QA

| 文件 | 用例 | 覆盖 |
|---|---|---|
| `qa_uwb_sync_cir_core.cc` | 13 | codes 9–12 × 38/116 tap × 64/256 SYNC × skip10 × 整数/分数 P、clipped 边界、0 有效重复、容量不足、非有限输入、独立 float64 参考、comm 逐位一致、幅度/相位、延迟轴、`required_input_range` |
| `qa_uwb_preamble_frontend.cc` | 6 | CFO 覆盖用 final CFO（0/±50 kHz）、无 double rotation、bootstrap→final epoch 失效、覆盖不足 fail-closed、`SyncSfdOnly` 早停、`FullPacket` 字段与重构前基线一致 |
| `qa_uwb_radar_shared_cir.cc` | 5 | 目标 CFO 补偿（0/±50 kHz）、距离轴保留、幅度/相位、失败路径、与旧标量核 float64 对照 |

`qa_uwb_pdu_roi_cir_65_32.cc` 的 `require_sfd` fixture 由「LCG 噪声 + 三个孤立
脉冲」改为**真实调制帧**（hrp_mod 998.4M → 32/65 → 491.52M native）。原 fixture
依赖 `sfd_threshold=1e-6` 让噪声通过检测；共享前端要求相干 SYNC 训练，噪声不再
可能通过。改后 ROI 与 FULL 的 raw/norm taps **逐位相同**（`max|diff|=0`），并新增
`|sfd-predicted|≤2`、`|origin-cir_origin|≤2` 两条断言，防止「两条链在垃圾上一致」
再次满足测试。

## 2. 验证结果

### 2.1 全量串行 CTest

```
83 tests, 79 passed, 3 failed, 1 skipped (uwb_qa_install_consumer 按设计跳过)
Total Test time ≈ 121 s
```

3 项失败为**既有失败**，已在未修改的基线上复现同一输出：

| 测试 | 失败点 | 判定 |
|---|---|---|
| `uwb_qa_uwb_pdu_rational_resampler_65_32_workers.cc` | `avx2_fma_macroblock != volk_macroblock`（`-march=native` 下默认内核选择） | 既有，与本次改动无关 |
| `uwb_qa_uwb_fullpacket_cfo_diag.cc` | 既有 | 既有 |
| `uwb_qa_uwb_preamble_sic_core.cc` | `geom_miss == 0`（SIC self-energy 几何） | 既有 |

复现方式：`git stash` 掉 `uwb_demod_core.h` 后重建这三个目标并运行，得到逐字节
相同的失败输出。

### 2.2 comm 逐位不变

10 个配置（默认 64-SYNC code-10、256-SYNC code-9、38/116 tap、整数/分数周期、
on-demand 前缀、fused payload+PHR、`skip_initial=0`、code-11、125 B PSDU）逐字段
对照重构前后：`cir_values`、`cir_filter_taps`、`first_path_sample`、
`cir_peak_metric`、`soft_chip_count`、`samples_per_chip`、soft-chip FNV、payload
FNV/PSDU/FCS **全部逐字节相同**，diff 为空。

### 2.3 数学与估计误差分离

- 公共核 vs 独立 float64 参考：96 格，相对 L2 ≤ 2e-5，峰 tap 一致。
- 公共核 vs 旧标量 `estimate_radar_cir`（同一零 CFO 输入）：raw 相对 L2
  **9.8e-8**、norm **5.8e-8**，峰 tap 一致。
- 接收端 CFO 估计误差单独列出，不并入 2e-5 数值门槛：±50 kHz 时共享路径峰 tap
  不变、幅度差 **0.0002 dB**；不补偿的对照臂在 ±50 kHz 下损失 **−21.66 dB**。

### 2.4 性能（摘要，详见性能报告）

| 门槛 | 配置 | 实测 | 判定 |
|---|---|---|---|
| 116-tap 共享 CIR 比旧标量核快 ≥20% | 64/32 | 174.89 → 13.03 µs（13.4×） | 通过 |
| 同上 | 256/127 | 217.82 → 46.67 µs（4.7×） | 通过 |
| sensing 接收阶段整体快 ≥10% | 64/32 | 855.77 → 648.07 µs（−24.3%） | 通过 |
| 同上 | 256/127 | 899.25 → 1176.61 µs（**+30.8%**） | **未通过** |
| comm 不得稳定退化 >5% | — | 本 build 内无 control 可对照 | 未验证 |
| SIC-only 不得稳定退化 >5% | — | 本基准不覆盖 | 未验证 |

256/127 的退化已拆解到 µs：共享内核 −166.6、窄窗 SFD −179.8，但公共前端必须
跟踪整条 256-SYNC 训练序列 **+375.5**、新增最终 CFO 旋转 **+240.3**。跟踪项
单独就是净增量的 135%。**这不是内核缺陷，而是采用 comm 检测前端换取非零 CFO
能力的正确性成本**。已实施的低风险缓解（`timing_seed_refine_radius=8`、
`timing_max_backtrack_symbols=0`，与旧 radar 的局部细化语义对齐）把
`frontend_timing` 从 414.43 降到 387.93 µs，但**没有**关闭该缺口。

## 3. 未验证 / 未完成

- **方案 §7.7 端到端功能矩阵未跑**：64/32 与 256/127 × work/native 两网格 ×
  单径/分数多径 × 两个目标码方向（9 受 10 干扰、10 受 9 干扰）× 目标与干扰 CFO
  0/±50k × ISR 0/10/20。现有证据是 QA 级的 CFO/延迟轴/幅度相位检查 + 零 CFO 的
  性能格，**不是**该矩阵。
- **非零 CFO 的感知质量未测**：raw CIR 峰位/幅度、PSLR/ISLR、主瓣宽度、
  强弱双目标保护均未在非零 CFO 下测量。
- **comm 的 control/candidate 对照未做**：`demodulate_one()` 已被就地替换，同一
  build 内不存在第二份旧实现；跨 build 对照需要另建目录与 fixture，本轮未做。
  逐位一致性（§2.2）是替代证据，但不等价于跨 build 计时对照。
- **SIC-only 成本未测**；SIC 训练仍限前 8 符号、末 4 SYNC 实减由既有
  `uwb_qa_uwb_preamble_sic_core.cc` / `uwb_qa_uwb_sic_unified.cc` 覆盖（通过）。
- **256/127 sensing 退化未修复**。下一步是给 `CirOnly` 增加「种子短跟踪」模式：
  只跟踪尾部若干 SYNC 并从 SFD 重建网格，从而省掉整条训练序列的跟踪。未实现、
  未验证。
- **逐重复发射未做 CFO 补偿**：`emit_individual_repetitions` 仍走旧标量估计器
  读 raw 窗口；只有 `cir_avg` 走共享内核补偿后的缓冲。
- **`WindowResult` 新字段未导出到 JSON**：`apps/pipeline_result_json.h` 不在本轮
  文件范围，字段已填充但未打印。
- **`apps/benchmark_radar_pipeline.cc` 仍手工拆分** SFD/refine/CIR，与
  `benchmark_chain_workers.cc` 已修掉的是同一类陈旧诊断路径。
- 未上板、未做 paced 到达、未做 8 worker、未做 MATLAB 交叉验证、未做 65/48 与
  65/32 native 网格。
- 绝对耗时 648 µs（64/32）与 1177 µs（256/127）仍是纯 DSP，**未达** mean≤0.5 ms /
  P99≤1 ms / 200 窗/s 目标，本轮不宣称实时达标。

## 4. 评审者需要知道的默认行为

1. **sensing 现在估计并补偿目标 CFO**（`compensate_target_cfo` 默认 true）。
   这是行为变更：此前 sensing 直接平均 SIC 输出，不补偿目标 CFO，因此目标
   CFO=0 的历史矩阵不能用来验证非零 CFO 感知。
2. **raw CIR 的标度**：`raw = Σ avg[n+m]·conj(code[m]) / code_energy`，保留幅度
   与相位；`norm` 是单位 L2，不覆盖 `raw`。
3. **距离原点**：`cir_axis_origin` 是发送时序/公开预测原点（含传播时延），
   检测峰**不会**把它重新居中；改变检测起点不会把物理峰归零。
4. **相位原点**：`phase_reference_sample` 是窗口内绝对样点 0；跨窗复数相位按
   `dst[i]=src[i]·e^{-jwi}` 约定可比。
5. **失败语义**：无有效重复、非有限输入、CFO 失败、覆盖不足一律失败关闭，
   `tap_count=0`，不返回 seed-only 或上次窗口残留。
6. **旧标量数学**只保留给逐重复发射与测试参考，不再是第二套在线算法。
