# Timing / SFD / CIR 与匹配滤波优化实施报告

> **日期**：2026-11-19  
> **分支 / 基线**：`feature/sic-research`，基于 `b654733`  
> **依据**：[多 Agent 并行开发方案](多Agent并行开发方案_Timing_SFD_CIR匹配滤波优化_2026-11-19.md)、
> [Timing/SFD/CIR 与匹配滤波优化分析](Timing_SFD_CIR与匹配滤波优化分析_2026-11-19.md)  
> **性能数据与复现包**：[Timing_SFD_CIR 优化：性能结果与复现](Timing_SFD_CIR优化_性能结果与复现_2026-11-19.md)  
> **范围**：除 `CIR-4`（Radar 专项）外的优化项；**未修改任何 `uwb_radar_*`
> 头文件、Radar 测试或 Radar 逻辑**。

## 1. 执行方式与边界

原方案按 3 个 Wave、6 个 worktree 并行下发。本次由单个 OpenCode 会话在同一
工作树内**顺序**完成 Wave 1 与 Wave 2 的代码改造：同一时刻只有一个写者，
因此不存在文件级冲突，无需合并分支。Wave 3 的 `MF-4` 已按 agy 复核补做
（opt-in，见 §2.6）；完整 ABBA / 双 ISA 全量验收未执行，只做了本机可复现的
构建、CTest 与 6×40 固定窗回归（见 §5）。

硬件环境：Intel i7-12700（**仅 AVX2/FMA，无 AVX-512**），20 核，
`ENABLE_UHD_BACKEND=OFF`，Release。AVX-512 只做了编译期验证。

## 2. 已实施条目

### 2.1 P0 / 匹配滤波内核

| 项 | 实施内容 |
|---|---|
| **P0-1** | `sparse_ternary_dot_16lag` / **新增 32-lag**、`pack_even_complex` 的宽指令实现移入 `uwb_demod_sic_avx512.cc`（`UWB_DS_AVX512_TARGET`），头文件只保留纯 POD 接口；`uwb_rx_matched_filter.h` 通过 `use_avx512(KernelGroup::Sync)` 运行时分发。`SyncMatchedFilter::for_each_lag` 改为 **32 → 16 → 8 → 4 → 1** 级联，`lag_count = 8 / 20 / 38` 不再掉进标量尾。 |
| **MF-1** | `SparseCirTemplate` 增加 `alignas(64) uint16_t pos_off[128] / neg_off[128]` 与计数，热路径不再走 `std::vector` 指针间接；AVX-512 32-lag（4 个独立累加器对）与 AVX2 16-lag（4×YMM）。每个 lag 的累加顺序与 8-lag 参考逐位一致。 |
| **MF-2** | 初版实现为无分支 `acc += sign * x`。隔离微基准 26.4 µs vs 26.8 µs（−1.6%，在噪声内）；整包 `st_timing` 的 +40% 回归经隔离确认来自 MF-3，而不是 MF-2。仍回退为原 add/sub，以保留原实现并减少改动面。 |
| **MF-3** | 初版实现了交叉相乘免除法。实测 `scan_normalized_sync` 的浮点除法吞吐约 4 周期，交叉相乘 + 近平分点分支反而**慢约 18%**（1016 lag：4.8→5.7 µs），因此**回退**为原逐 lag 除法；同时恢复 `timing_metric` 的逐位语义。 |
| **SFD-1** | `fold_sfd_window` 改为单遍 8-way：每个输出元素按符号顺序在寄存器内累加，只写一次 `out`（写流量约降为 1/8）；AVX-512 / AVX2 / 标量三条路径都保持与旧“逐符号 read-modify-write”逐位一致。 |

### 2.2 Scratch / 软芯片域

| 项 | 实施内容 |
|---|---|
| **P0-3（Scratch）** | `DemodScratch` 新增 `half_cir_template`、`fold_buf`、`fold_sup_buf`、`cir_avg_tail`、`cir_phase_rot`，并在 `reserve()` 中预留容量。 |
| **SFD-4** | 评估了 `stage_ns_sfd` 的无分支符号乘。与 MF-2 同理，符号模式在 17 个 lag 间固定、分支可预测，`sign * v` 每个 entry 多一次乘法；本轮保留原 add/sub（NS-SFD 仍为 ~7 µs 量级，改动无正收益）。未引入 pos/neg 分离累加（会改变结合顺序、影响既有 NS-SFD golden 判决）。 |

### 2.3 Timing

| 项 | 实施内容 |
|---|---|
| **P0-3（Timing）** | `stage_timing` 在 `sparse_tmpl != nullptr` 时**跳过** `scratch.work.assign`、`uwb_l2_normalize` 与 `build_real_sync_template`；稠密路径新增 `RealSyncTemplate::key` 内容哈希缓存（`ensure_real_sync_template`）。`fold_acquire` / `fold_anchor_fit` 改用 `scratch.fold_buf` / `fold_sup_buf`，能量改用 `window_power_8acc`。`stride == 2` 改用按 key 缓存的 `scratch.half_cir_template`。`window_power_8acc` 上移到 `detail` 之前以便 fold 复用。 |
| **TM-1** | `AcquireLegacy` 在 `!seed_acquired` 且跨度 ≥ `2P` 时，先在 seed 与 ±32P 处取长度 `P` 的探针窗做全速 `scan_normalized_sync`；**仅当探针 metric ≥ 跟踪阈值**才用 `phi + mP` 网格 + `local_best(..., radius=2)` 定位起点。探针不达标或网格全不达标时，**原粗扫路径原样保留**，因此低 SNR 行为不变。 |
| **TM-2** | `LockedBounded` 的 `m = 0` 在 `roi > 32` 时先扫 ±32；未命中再扫完整 ±roi。`m0_raw_max` 的语义与后续 early-stop 保持一致。 |
| **TM-4** | `try_reuse_initial_tail_timing` 改为以 SFD 锚点 `cfg.tail_start + period` 预测尾部网格，在 Stage 1 的 `peak_samples` 中按 **±2 样点**就近匹配；匹配数 ≥ `tail_repetitions - 4` 且匹配峰 metric 均达阈值即复用，避免再次跑 40-SYNC `stage_timing`。 |
| **TM-5** | `probe_fold_phase` 的 `eb` 改为 `ea - head + tail`，只额外读 2×1016 样点，省掉第二次 13 万点全窗扫描；探测上限保留 128 符号（实测降到 64 会使雷达时序 QA 的 `-14 dB` 折叠捕获从 50/50 降到 39/50）。`fold_accumulate` 的非零相位路径改为 AVX2 复乘累加（`addsub` 复数乘法，逐点结果与标量一致）。 |

### 2.4 SFD

| 项 | 实施内容 |
|---|---|
| **P0-3（SFD 初始功率）** | `stage_sfd::scan_window` 的初始窗能量改用 `window_power_8acc`。 |
| **SFD-2** | `anchor_and_verify_final_sfd`：当 `cfo_refit_action == 1/2`（复用 Stage-2 旋转），或 refit CFO 在 8 个符号内累积相位 < 0.05 rad 时，**跳过** `run_final_sfd()`，直接沿用 bootstrap SFD 结果。 |
| **SFD-3** | (1) `polarity_ok` 统一跳过 `sfd_sequence[si] == 0`（去掉 `par_only &&`，修复 ieee/decawave 含 0 序列恒被拒）；(2) 离散网格精修跳过已算过的 `d == 0`；(3) 符号相关缓存改为**按实际 `delta_k = round(k*period) - k*sym` 分桶**，`+27 ppm` 等任意 SFO 下仍复用，且数值与逐点直接计算逐位一致；(4) `best_pos == gpos` 时直接复用缓存的 `sym_corr` 做极性校验，省 8–16 次 `sync_filter.dot`。 |

由于 (1) 是真实缺陷修复，两个既有 QA
`test_demod_core_r1_sfd_bounded_five_symbol_{forward,backward}` 的旧断言
（“±5 符号必须失败”）与 ±36 符号网格救援的既定契约矛盾。测试已改为断言
网格救援能找到真实 SFD（`75008`、metric ≥ 0.95），不是删除断言或放宽容差。

### 2.5 流水线与 CIR / SIC

| 项 | 实施内容 |
|---|---|
| **P0-2** | `LockedBounded` 重试前恢复 `locked_roi_samples`；若重试参数与原参数等价（`locked_roi_samples >= profile.locked_roi_samples`）则**不再做必败的第二次 `stage_timing`**。当 Stage 1 为 `sync_peaks_folded` 且 `stage2_cfo.peaks_used >= 16` 时直接跳过 `retrack_tail_timing`，沿用 Stage 2 CFO。 |
| **CIR-1** | `estimate_cir` 新增可选 `cfo_hz`：传原始 `rx` 时，每个 SYNC 只乘一个复标量 `exp(-j w lo_k)`，循环外对 ~1053 点乘一次 `exp(-j w m)`。同时生成末尾 32 个 SYNC 的 `cir_avg_tail`；`stage_softchips_from_cir` 的 `skip_sync_fir` 相位改用 `sync_phase_gain_from_tail`，不再读 32×1052 点。`compute_post_sfd_prefix_bounds`（`--fused on`）与 `compute_cir_only_rotation_range`（`--stop-after-cir on`）不再把前导纳入去旋转范围。`uwb_discrete_cir_estimator.cc` 的 Ipatov 训练同样改为“每 repetition 一个标量 + 结束乘一次 `width` 长度旋转表”。 |
| **CIR-2** | `estimate_cir` 的 `values` 改 `std::array<complex<float>,64>`（>64 taps 才回落堆分配）；`scratch.corr` 尾部补 8 个零；归一化路径使用 `correlate_cir_sparse_raw`，省去可被 L2 归一化约掉的 `/code_energy` 与 `1/valid`。 |
| **CIR-1 数值验证** | 新增 QA：`estimate_cir(raw, +25 kHz)` 与 `estimate_cir(derotated, 0)` 的 CIR L2 差 < 1e-4；`sync_phase_gain_from_tail` 与 `sync_phase_gain` 的相对差 < 1e-3。 |

### 2.6 MF-4：`(r ⋆ code) ⋆ p` 因子分解

| 项 | 实施内容 |
|---|---|
| **数学前提（已验证）** | 工程里的稠密前导模板就是 `code ⊛ p`：`testdata/reference_preamble_code10_998p4.cf32` 与 `BuildSampledCode(code10) ⊛ kPulse48` 的相对 L2 误差 **4.2e-6**。因此 `r ⋆ (code*p) = (r ⋆ code) ⋆ p` 对实际模板成立，不是近似模型。 |
| **核提取** | `build_shaped_sparse_template()` 从 `template_wf` 的第一个非零码片位置取 `nt` 个抽头作为 `p`。码片间距 8 样点，故 `nt ≤ 8`（第 9 抽头已被下一个脉冲污染）。`nt` 同时被 `L - last_code_pos` 夹紧，保证 `c[m]` 的所有读取仍在调用方的 L 样点契约内。 |
| **重建门控** | 模板不是 `code*p`（自定义波形、复数波形、能量异常）时直接 `valid=false`，调用方保持原路径；不会静默换核。 |
| **块内共享** | `for_each_lag`：每 32 个输出 lag 用一次 32-lag 稀疏批-次 + `nt-1` 个标量尾 lag，再对共享的 `c[]` 做短实 FIR；`dot()`：一次 8-lag 批 + FIR。`scan_normalized_sync`、`fold_acquire`、`fold_anchor_fit` 全部走同一 `SyncMatchedFilter` 分发。 |
| **精度** | 省略的脉冲尾部只占模板能量 ~0.3%。高 SNR 合成峰（真实模板）实测 metric 相对稠密 **−0.03 dB**，峰位置一致。 |
| **开关** | `Qm35825Profile::rx_timing_shaped_sparse` + `uwb_dw3000_live --timing-shaped on`（要求 `--timing-sparse off`）。它替换**稠密**模板（同一 metric 尺度、同一 lag 约定），不是稀疏路径的叠加。 |
| **为什么不叠加在稀疏上** | 稀疏模板按“脉冲峰值样本”对齐，峰位比稠密/因子分解晚约 2 样点；两者混用会使低 SNR 下的 SFD 锚点改变，`-12 dB` 实测少 1–2 包（tmetric 已与稠密一致，不是灵敏度问题）。因此把 shaped 定位为稠密路径的替代。 |
| **验证** | 新增 QA：合成 `code*p` 模板上 32-lag 块 + FIR 与 1024-tap 稠密相关逐点相对误差 < 1e-5、峰位相同、`dot` 一致；不可分解模板被拒绝。live 文件回放 3 次：`control 40/40`、`seed1729 38/38`、`cfo40k 37/37`、`sfo27ppm 38–39`，FCS 数不低于稠密路径；`st_timing` 比稠密快 **3.7–4.5×**，`st_total` 快 **19–61%**（SFO 场景因既有的 ~8.9 ms 全流 soft-chip，仅 −11%），比纯稀疏在 seed1729/cfo40k 多 1–2 包，SFO 持平。 |

## 3. 明确未实施或部分实施的条目

| 项 | 状态 / 原因 |
|---|---|
| **MF-2 / SFD-4 无分支符号运算** | MF-2 隔离微基准在噪声内（26.4 vs 26.8 µs），无正收益；SFD-4 未单独测量且量级很小。两者均保留原 add/sub；`st_timing` 的 +40% 回归已确认来自 MF-3。 |
| **MF-3 交叉相乘免除法** | 实现后实测更慢（`divss` 吞吐优于“双乘 + 近平分点分支”），已回退为原逐 lag 除法；见 §2.1。 |
| **MF-3 的 PAR 融合** | `raw_lag_par_passes` 仍做一次 `powers[]` 背景遍历。PAR 路径的 `powers` 长度受 `local_best` 限制（≤1024）且已有峰值 guard，二次遍历占比很小；为保持背景样本计数与三样点聚类的既有语义，未改。 |
| **TM-3** | **分析前提不成立**：现实现每个 8-lag 点只调用一次 `window_power_8acc(rx+base, L)`，其余 7 个 lag 用 `norm` 增量更新，并不存在“每个 lag 重读 1016 点、合计 2 MB”的行为。因此未改，避免改变 argmax 语义。 |
| **TM-5 的 fold 组复用** | 计划中的“先折 4×16 组给 `fold_acquire` 复用”需要重排 `fold_anchor_fit` 的窗口与相位处理；在 `-14 dB` 雷达 QA 已经对探测长度敏感的情况下，本轮未做。 |
| **MF-4 的 MATLAB 交叉验证** | 因子分解已实现（§2.6）、C++ 逐点等价与 live FCS 对齐均已验证；但 `AGENTS.md` 要求的 MATLAB `detectRepeatedPreamble.m` 交叉验证本机无法执行（无 MATLAB/Octave）。因此默认 profile 仍保留稠密路径，`--timing-shaped` 仅为 opt-in。 |
| **CIR-3** | 大 SFO 的 4 桶分数相位相干平均会改变 CIR 数值，需要 MATLAB `estimateCir` 的逐样点对比；本轮只验证了现有 `llround` 路径与 CIR-1 的等价性，未引入桶插值。 |
| **CIR-4** | 任务书明确排除 Radar 专项，未触碰任何 `uwb_radar_*` 文件。 |

## 4. 构建与代码验证

| 构建 | 结果 |
|---|---|
| `UWB_SIMD=AUTO`（本机选 AVX2/FMA），`ENABLE_UHD_BACKEND=OFF` | 编译通过；全量 CTest **54 项 53 通过 + 1 跳过**（安装消费者无安装前缀，按既有约定 SKIP），0 失败。 |
| `UWB_SIMD=SCALAR`，`ENABLE_UHD_BACKEND=OFF` | 编译通过；全量 CTest **57 项 56 通过 + 1 跳过**，0 失败。 |
| `UWB_SIMD=AVX512`，`ENABLE_UHD_BACKEND=OFF` | **仅编译验证通过**。本机 i7-12700 无 AVX-512，无法运行 AVX-512 分支的运行时测试；不得写成 AVX-512 已运行验收。 |

测试命令统一使用 `env -u LD_LIBRARY_PATH`；未使用 `cmake --install` 作为前置。
AVX-512 内核逐位契约由
`qa_uwb_demod_sic_avx512.cc` 新增用例覆盖：16/32-lag 稀疏相关对 8-lag 参考、
4-lag 对 8-lag、`pack_even_complex` 对标量、单遍 `fold_sfd_window` 对标量。
`qa_uwb_demod_core.cc` 新增：SFD 在 +27 ppm 下的 per-delta 缓存与直接计算等价、
CIR 融合旋转对全帧去旋转等价、`avg_tail` 对 `sync_phase_gain` 等价。AVX-512 主机上
应能在同一二进制内强制 `Sync=avx512` 后再跑一次。

## 5. 6×40 固定窗回归（与 `b654733` 对照）

夹具由 `tools/rx/low_snr_eval/fixed_scenarios.py --generate` 现场生成
（code10 / 256 SYNC / 4z2 / 127 B，`pre_guard=36864`、`window=450000`、
`eq16`），基线解码器为改动前 `build-native` 的静态二进制副本。

| 场景 | 基线 exact | 优化 exact | wrong FCS | 逐字段差异 |
|---|---:|---:|---:|---|
| control (+4 dB) | 40/40 | 40/40 | 0 | **0 帧** |
| seed1729 (−12 dB) | 38/40 | 38/40 | 0 | 36 帧，仅 `timing_metric` |
| seed2718 (−12 dB) | 40/40 | 40/40 | 0 | 37 帧，仅 `timing_metric` |
| cfo10k (−12 dB) | 37/40 | 37/40 | 0 | 33 帧，仅 `timing_metric` |
| cfo40k (−12 dB) | 38/40 | 38/40 | 0 | 31 帧，仅 `timing_metric` |
| sfo27ppm (−12 dB) | 37/40 | 37/40 | 0 | 33 帧，仅 `timing_metric` |

- FCS / PSDU / status / SFD / CFO / PHR 全部逐包一致，**无错误 FCS**。
- 唯一差异是 `timing_metric`（折叠路径的归一化分母改用 `window_power_8acc`
  后，最后约 1e-6 的浮点结合顺序差），不是判决量，也不改变任何解码结果。
  因此本轮**没有做到“逐字节完全一致”**；若后续要求字节级固定窗门禁，需要
  把折叠路径的分母改回 `sample_power_sum`，或把 `timing_metric` 从门禁字段
  中显式排除（二选一需评审决定）。

### 5.1 同机串行 live replay A/B（非验收）

基线为 `b654733` 在 `/tmp` 独立 worktree 构建的 `uwb_dw3000_live`，优化版为
本工作树 `build-native`。命令：

```
--replay <6x40 .sc16> --seconds 0 --period-ms 1.0 --mode continuous
--workers 1 --queue 256 --cir full --fused on --taps eq16 --tone off
```

A/B/B/A ×2（共 8 进程），表内为 `SUMMARY` 均值中位数与 `PACKET` 阶段中位数
（i7-12700、AVX2/FMA、UHD OFF、单 worker、文件回放）：

| 场景 | demod_mean | st_total | st_timing | st_sfd | st_cir | st_payload |
|---|---:|---:|---:|---:|---:|---:|
| control (+4 dB) | 885.5→724.7 (−18.2%) | 935.6→677.8 (**−27.6%**) | 219.8→158.6 (−27.8%) | 141.8→28.2 (−80.1%) | 171.2→160.5 (−6.3%) | 322.3→261.7 (−18.8%) |
| seed1729 (−12 dB) | 1766.2→1574.2 (−10.9%) | 1687.0→1538.1 (**−8.8%**) | 550.2→573.5 (+4.2%) | 231.8→48.4 (−79.1%) | 147.7→173.1 (+17.2%) | 409.8→404.4 (−1.3%) |
| cfo40k (−12 dB) | 1609.4→1311.8 (−18.5%) | 1878.6→1349.7 (**−28.2%**) | 708.3→628.9 (−11.2%) | 298.4→49.0 (−83.6%) | 163.3→165.3 (+1.3%) | 385.7→336.0 (−12.9%) |
| sfo27ppm (−12 dB) | 10441.8→10455.1 (+0.1%) | 10507.2→10347.8 (−1.5%) | 690.5→718.8 (+4.1%) | 518.4→406.6 (−21.6%) | 8979.4→8904.1 (−0.8%) | 242.6→235.7 (−2.9%) |

- 高 SNR 与 `+40 kHz` CFO 的端到端阶段总耗时下降 **~28%**；`-12 dB` 无 CFO 约
  **−9%**。主要贡献是 SFD（SFD-2 跳过 final SFD）与 payload（CIR-1 缩小去旋转
  范围），高 SNR 的 timing 也有 −28%（P0-3 + 稀疏 32-lag 级联 + TM-2）。
- `sfo27ppm` 的 `st_cir` 在两侧都约 8.9 ms（分数网格强制全流 soft-chip，属于
  既有路径），因此端到端被它掩盖，阶段总耗时仅 −1.5%。
- 低 SNR 的 `st_cir` +17%（+25 µs）来自 CIR-1 的 `avg_tail` 额外乘加；它与
  SFD/去旋转的收益不在同一阶段，但整包仍是净收益。
- `st_nssfd` / `st_phr` 的 ±1.5 µs 波动在噪声范围内。
- 同机固定窗 `uwb_dw3000_decode`（无 `--timing-sparse`、无 live 调度）ABBA
  变化为 control −2%、seed1729 −1%、sfo27ppm ±1%，说明收益主要来自 live 默认
  的稀疏 timing 与 fused 路径，而不是固定窗解码器本身。

以上均为 i7-12700 / AVX2 文件回放，**不是** 9700X / AVX-512 的验收数据。

## 6. 验收边界

- 本轮全部为**软件 / 文件回放**：无 UHD 上板、无 paced 流、无 8 worker、
  无负载 P99、无多径、无 MATLAB 交叉验证。
- 未能运行 AVX-512 硬件测试；AVX-512 只保证编译与代码路径正确性，性能收益
  未在本机测量。
- `--timing-sparse on` 的 `+27 ppm` SFO 稀疏路径没有独立夹具；SFD 的 per-delta
  缓存与直接计算在 `+27 ppm` 稠密 real 模板下做了等价性 QA，稀疏模板在
  `0 ppm` 的既有低 SNR QA 全通过。
- `MF-4` 的 MATLAB 交叉验证、`CIR-3` 的灵敏度收益、`TM-1` 在真实 `AcquireLegacy` 低 SNR 宽窗上的
  召回率，均未做 MATLAB 或 6×40 之外的大样本验证，**不得写成已验收**。

## 7. 复现

```bash
# AVX2（本机 AUTO 选 AVX2）
cmake -S gr-uwb -B gr-uwb/build-native -DCMAKE_BUILD_TYPE=Release \
  -DUWB_SIMD=AUTO -DENABLE_UHD_BACKEND=OFF
cmake --build gr-uwb/build-native -j
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-native --output-on-failure

# SCALAR
cmake -S gr-uwb -B gr-uwb/build-opt-scalar -DCMAKE_BUILD_TYPE=Release \
  -DUWB_SIMD=SCALAR -DENABLE_UHD_BACKEND=OFF
cmake --build gr-uwb/build-opt-scalar -j
env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-opt-scalar --output-on-failure

# AVX-512：仅编译（本机无 AVX-512 CPU）
cmake -S gr-uwb -B gr-uwb/build-opt-avx512 -DCMAKE_BUILD_TYPE=Release \
  -DUWB_SIMD=AVX512 -DENABLE_UHD_BACKEND=OFF
cmake --build gr-uwb/build-opt-avx512 -j

# 6x40
python3 tools/rx/low_snr_eval/fixed_scenarios.py --generate \
  --fixtures /tmp/uwb-low-snr-6x40 --build-dir gr-uwb/build-native
env -u LD_LIBRARY_PATH python3 tools/rx/low_snr_eval/fixed_scenarios.py \
  BASE_DECODER gr-uwb/build-native/apps/uwb_dw3000_decode \
  --fixtures /tmp/uwb-low-snr-6x40 --out /tmp/fixed_6x40.json
```
