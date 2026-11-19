# Timing、SFD、CIR 与匹配滤波（互相关）优化分析与决策清单

> 日期：2026-11-19  
> 分支：`feature/sic-research`（基于 `b654733` / `08052b3` 现状梳理）  
> 用途：梳理当前 C++ 原生解调/感知/雷达链路在 Timing、SFD、CIR 估计以及底层匹配滤波上的剩余优化空间，解释背后的 **C++ 工程技巧、CPU 微架构原理与 DSP 数学原理**，并给出性能收益预估与勾选决策表。

---

## 使用说明与总览勾选表

请在您希望实施的条目方框中勾选（改为 `- [x]`），不修改的保留 `- [ ]`。  
表中的耗时基线均来自仓库已有实测记录（未标“实测”的预期收益为基于算力/访存比例的工程估算，实际结果以 ABBA 配对实测为准）：
- **9700X 高 SNR 实采基线**（[开发状态.md](../../开发状态.md)）：`AVX512`、`--sync locked --tone off --taps eq16`
  - Comm 稳态（`path=target`）：`demod_us = 288.2 µs`，总计算 `894.4 µs`
  - Sensing（`--stop-after-cir on --lock-fcs off`）：`demod_us = 194.1 µs`，总计算 `812.8 µs`（其中 `resample_us ≈ 606–619 µs`）
  - Acquire 宽窗（`path=acquire`）：`demod_us = 845.3 µs`
- **i7-12700 低 SNR `-12 dB` 基线**（[低SNR解调计算耗时优化开发报告_2026-11-17.md](低SNR解调计算耗时优化开发报告_2026-11-17.md)）：`AVX2`、`locked w1`
  - `demod = 1383 µs`（`st_timing = 482 µs`，`st_sfd = 317 µs`，其中 `sfd_tail = 103–262 µs`、`sfd_cforef = 100–132 µs`）

### 待决策优化清单（勾选框）

#### 第一类：隐性 Bug 与“免费”性能修复（P0，几乎无算法风险）
- [ ] **P0-1**：修复 `AVX-512` 16-lag 稀疏相关与 `pack_even_complex` 的死代码问题，并补齐 `16→8→4→1` 尾项级联
- [ ] **P0-2**：修复 `retrack_tail_timing` 在 `LockedBounded` 低 SNR 下的必败双倍重跑（及折叠路径下跳过必败重跟踪）
- [ ] **P0-3**：消除 `stage_timing` 每包无条件重建稠密模板、标量 `sample_power_sum` 与热路径 `std::vector` 堆分配

#### 第二类：底层匹配滤波（互相关）内核深度优化（P0/P1）
- [ ] **MF-1**：稀疏三元码核微架构提速（32-lag AVX-512 / 16-lag AVX2 打破 FADD 延迟墙 + 寄存器数据复用 + `uint16_t` 偏移表）
- [ ] **MF-2**：修复 `sparse_ternary_dot_single` 分支与串行依赖链
- [ ] **MF-3**：滑窗能量归一化（分母）瘦身（argmax 交叉相乘免除法、PAR 融合累计、去重）
- [ ] **MF-4（数学级）**：用卷积因子分解 $(r \star \text{code}) \star p$ 统一“稀疏速度”与“稠密低 SNR 灵敏度”

#### 第三类：Timing 阶段优化（P1）
- [ ] **TM-1**：`AcquireLegacy` 宽窗盲搜从 $O(2 \cdot \text{margin})$ 降维到 $O(P) + O(\text{margin}/P)$（单/双符号相位探针 + 网格沿搜索）
- [ ] **TM-2**：`LockedBounded` 首探 `m=0` 先窄（`±32`）后外环，减少 87% 稳态首探 lag
- [ ] **TM-3**：`LockedBounded` `grid_track` 先对 8-lag 选纯相关峰值再单次归一化，大幅削减 2 MB 内存分母读取
- [ ] **TM-4**：放宽 `try_reuse_initial_tail_timing` 的复用条件（高/中 SNR 免重跑 40-SYNC 尾部）
- [ ] **TM-5**：Fold 折叠回退优化（`probe_fold_phase` 滑窗差分、`fold_accumulate` 向量化、与 `fold_anchor_fit` 共享前 64 符号）

#### 第四类：SFD 阶段优化（P1）
- [ ] **SFD-1**：`fold_sfd_window` 单遍 8-way 合并（将写内存流量降为 1/8）
- [ ] **SFD-2**：`cfo_refit` 未重旋转或极小频偏时跳过/极窄窗化 `run_final_sfd`
- [ ] **SFD-3**：离散网格清理（去掉 `d=0` 重复算、复用 `sym_corr`、支持任意 SFO 去重缓存、修 `sfd_seq==0` 逻辑）
- [ ] **SFD-4**：软芯片域 `stage_ns_sfd` 稀疏向量化

#### 第五类：CIR 估计与跨阶段架构融合（P1/P2，最大单点收益）
- [ ] **CIR-1（高收益）**：融合 CFO 去旋转与 SYNC 相干平均，并与 `sync_phase_gain` 共享末尾 32 SYNC（免去 ~250k 样点前导去旋转）
- [ ] **CIR-2**：`estimate_cir` 微调（尾部补零对齐 SIMD、栈数组去分配、省去可抵消的标量缩放）
- [ ] **CIR-3**：大 SFO（如 `+27 ppm`）下的 4 桶分数相位相干平均（零代价对齐 MATLAB `interp1` 精度）
- [ ] **CIR-4（Radar 专项）**：Radar `estimate_radar_cir` / `estimate_radar_cir_repetition` / `search_sfd` / `refine_sync_origin` 迁移稀疏 SIMD

---

## 一、隐性 Bug 与“免费”性能修复（P0）

这一类改动不涉及任何新算法，纯粹是修正现有代码里“写了但没生效”或“白算了两遍”的地方。

### P0-1：修复 `AVX-512` 16-lag 稀疏相关死代码与尾项断崖

- [ ] **勾选修改 P0-1**

#### 问题原理（工程技巧：编译单元隔离与 `__AVX512F__` 陷阱）
为了防止在不支持 AVX-512 的 CPU 上执行程序时因为编译器自动向量化而直接触发 `SIGILL`（非法指令崩溃），项目在 [UwbSimd.cmake](../../gr-uwb/cmake/Modules/UwbSimd.cmake#L165-L179) 中采取了一种很标准的**安全架构**：
- 全局所有文件都只用 `-O3 -mavx2 -mfma` 编译；
- 只有 [uwb_demod_sic_avx512.cc](../../gr-uwb/lib/uwb_demod_sic_avx512.cc) 这一个 `.cc` 文件里的函数通过 `__attribute__((target("avx512f,avx512bw,avx512dq,avx512vl")))` 局部开启 AVX-512，并在运行时检查 CPU 支持后通过 `use_avx512(KernelGroup::Sync)` 调用。

**问题出在这里**：[uwb_rx_matched_filter.h](../../gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h#L118-L230) 是一个**头文件**，它在被 `uwb_demod_pipeline.cc` 包含编译时，编译器处于 AVX2 基线模式，因此宏 `defined(__AVX512F__)` **恒为假**！
结果就是：即使您用 `-DUWB_SIMD=AVX512 -DUWB_SPARSE_MF_AVX512_16LAG=ON` 编译：
1. `sparse_ternary_dot_16lag` 编译进去的其实是“两次 8-lag AVX2”；
2. `pack_even_complex`（用于 `stride=2` 宽窗降采样打包）走的是标量 `for` 循环；
3. `SyncMatchedFilter::for_each_lag` 里的 `visit_batched<16>` 也退回到了 `visit_batched<8>`。

**连带隐患（尾项断崖）**：如果直接把 16-lag 打开，[visit_batched](../../gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h#L245-L258) 目前只做了“16 个一组，剩下全走标量 `single`”。这会导致：
- `grid_track` 每次只算 **8 个 lag**（每包调 255 次）——会全部掉进 **8 次标量循环**，反而不如现在的 AVX2 8-lag 快！
- `estimate_cir` 算 **38 个 tap**——前 32 个走 2 批，最后 6 个走 6 次标量；SIC 算 **20 个 tap**——最后 4 个走标量。

#### 修改方案 & 收益评估
- 把 `sparse_ternary_dot_16lag` 和 `pack_even_complex` 的 AVX-512 实现挪到 `uwb_demod_sic_avx512.cc` 并加 target 属性，通过 `use_avx512(KernelGroup::Sync)` 调度；
- 在 `visit_batched` 中补齐 **`16 → 8 → 4 → 1` 级联**（或者对 `<16` 的尾项直接用 AVX-512 掩码 `_mm512_maskz_loadu_ps` 一次性收尾）。
- **预期提升**：在 9700X 等 AVX-512 机器上，真正激活 16-lag 后，稀疏相关内核吞吐再提升 **~1.5×–1.8×**（尤其对 `AcquireLegacy` 宽窗与 `fold_acquire`）。

---

### P0-2：修复 `retrack_tail_timing` 在 `LockedBounded` 低 SNR 下的“必败双倍重跑”

- [ ] **勾选修改 P0-2**

#### 问题原理（控制流冗余）
在 [uwb_demod_pipeline.cc](../../gr-uwb/lib/uwb_demod_pipeline.cc#L281-L320) 的 `retrack_tail_timing` 中，SFD 锚定后会对最后 40 个 SYNC 重跑一次 `stage_timing` 以重新拟合更准的 CFO。当 `sync_strategy == LockedBounded` 且低 SNR 时：
1. 函数开头第 284 行把 `anchor.locked_roi_samples` 强行夹紧到 `min(roi, 64)`，同时设 `anchor.timing_seed_refine_radius = 8`；
2. 但在 `stage_timing` 的 `LockedBounded` 分支中，**根本不读取 `timing_seed_refine_radius`**，只读取 `locked_roi_samples`；
3. 当低 SNR 导致首次 40-SYNC 尾部跟踪失败时，第 298 行把 `anchor.timing_seed_refine_radius = 0` 清零后重跑第二次 `stage_timing`，**却忘了恢复 `locked_roi_samples`**！
   这相当于用一模一样的参数，把“内侧探针 → 窄尾扫 → 40 符号折叠（因为小于 16 组或不够阈值而失败）→ 宽尾扫”**原封不动跑了两遍**！
4. **更进一步**：即使不看清零 Bug，当 Stage 1 已经通过 `fold_acquire` 才进锁（说明单符号 SNR 极低，单符号相关必败），且 `stage2_cfo.peaks_used >= 16` 时，对仅有 `40` 个符号的尾部重跟踪最多只能分出 $\lfloor 40/16 \rfloor = 2$ 组 16-符号超锚点（必然不满足 `< 16` 条件，后续第 449 行 `refit_well_conditioned` 也会丢弃它）。

#### 修改方案 & 收益评估
- 恢复重试时的 `locked_roi_samples`（如果还需要重试）；
- 当 Stage 1 是 `fold_acquire` 救回且 `stage2_cfo.peaks_used >= 16` 时，直接跳过 `retrack_tail_timing`（复用 Stage 2 CFO）。
- **预期提升**：在 `-12 dB` / `-14 dB` 低 SNR `LockedBounded` 下，单包 `sfd_tail` 耗时（当前实测 **103–262 µs**，占总 `st_sfd` 的约 60%–80%）可直接省去大半，**预计整包 `demod_us` 降低 100–200 µs（约 10%–15%）**，且不改变任何高/中 SNR 行为。

---

### P0-3：清理重复模板构建、标量初始功率与热路径堆分配

- [ ] **勾选修改 P0-3**

#### 问题原理（工程技巧：热路径零拷贝与向量化归约）
1. **无条件构建稠密模板**：[uwb_demod_timing.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h#L38-L41) 每次调用 `stage_timing`（每包至少调 1 次，加上 `retrack_tail` 调 2 次）都会执行 `scratch.work.assign(...)`、`uwb_l2_normalize` 和 `build_real_sync_template`（遍历 1016 个复数、复制成 2032 个 float）。但在 `--timing-sparse on` 时，**这三个产物完全不会被用到**！即使在 `--timing-sparse off` 时，`RealSyncTemplate` 里其实已经预留了 `key` 字段，却从未做哈希命中复用。
2. **单累加器浮点归约陷阱**：在 GCC 启用 `-ffp-contract=off`（或无 `-ffast-math`）时，像 `for (k=0..L-1) pwr += norm(x[k]);` 这样的**单变量累加循环，编译器绝对不敢自动 SIMD 向量化**（因为浮点加法不满足结合律，改变累加顺序会改变结果）。这就是为什么 `window_power_8acc` 用 8 个独立累加器 `a0..a7` 就能让编译器直接生成 AVX2/FMA 指令、速度翻倍。但目前 [uwb_demod_sfd.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sfd.h#L163)（计算 8128 样点能量）和 [uwb_demod_sync.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h#L268) 仍在用单累加器标量 `sample_power_sum`。
3. **热路径堆分配**：[estimate_cir](../../gr-uwb/include/gnuradio/uwb/uwb_demod_softchips.h#L99) 里的 `std::vector<std::complex<float>> values`、[fold_acquire](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h#L255) 里的 `folded`、[fold_anchor_fit](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h#L319) 里的 `sup` 和 [stage_timing](../../gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h#L588-L589) 里的 `half_tmpl` 都是在热路径现 `new/delete` 内存。

#### 修改方案 & 收益评估
- 仅在 `sparse_tmpl == nullptr` 时才归一化和构建 `real_sync_template`，并用 `key` 缓存；
- 所有 `sample_power_sum` 替换为 `window_power_8acc`；
- 临时 `vector` 全部移入 `DemodScratch` 或改固定长度栈数组。
- **预期提升**：稳态每包稳定节省 **15–30 µs**，多 worker 并发时消除内存分配器锁争用与缓存抖动。

---

## 二、底层匹配滤波（互相关）内核深度优化

目前无论 Timing、SFD 还是 CIR，底层都走 [uwb_rx_matched_filter.h](../../gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h) 的稀疏三元码匹配滤波。开启稀疏后，这里有巨大的微架构和数学优化空间。

### MF-1：稀疏三元码核微架构提速（破除延迟墙 + 寄存器复用）

- [ ] **勾选修改 MF-1**

#### 原理详解（CPU 微架构技巧：延迟墙 vs 吞吐）
看当前的 `sparse_ternary_dot_16lag`（8-lag 同理）：
```cpp
for (const size_t off : tmpl.pos_offsets) {
    p0 = _mm512_add_ps(p0, _mm512_loadu_ps(rx_f + off));
    p1 = _mm512_add_ps(p1, _mm512_loadu_ps(rx_f + off + 16));
}
```
在现代 CPU（如 Ryzen 7 9700X Zen 5 或 Intel 12 代以上）上：
- 一条向量浮点加法 `vaddps` 的**延迟（Latency）是 3–4 个时钟周期**，而执行端口的**吞吐（Throughput）是每周期 2 条**。
- 这意味着 CPU 同一瞬间其实可以同时推进 `3 × 2 = 6` 到 `8` 条互不依赖的加法链！但上面的循环里只有 `p0` 和 `p1` **2 条累加链**，下一轮循环的 `p0` 必须等上一轮 `p0` 算完才能算，导致 **2/3 的 CPU 算力都在空等**（这就是“FADD 延迟墙”）。

此外还有两个硬件友好的优化点：
1. **8-sample 对齐带来的寄存器级数据复用**：Ipatov 码每 4 个码片（即 `8` 个工作样点）才有一个非零值。8 个复数刚好是 **64 字节 = 1 个 ZMM 寄存器 = 1 条 L1 Cache Line**！假设某一段码片连续非零（例如 Code 10 中有连续的 `1, 1, 1` 和 `-1, -1, -1, -1`），在 32-lag 或 16-lag 扫描时，`c` 位置加载的 `rx[pos_c + 8 .. pos_c + 23]` 和 `c+1` 位置加载的 `rx[pos_{c+1} + 0 .. pos_{c+1} + 15]` **其实是同一块内存**！在按滞后（lag）分块的循环中展开 2–4 个码片，可以直接在寄存器里复用，减少 30%–50% 的 L1 Load 端口压力。
2. **偏移表瘦身**：`pos_offsets` / `neg_offsets` 目前存的是 `std::vector<size_t>`（每个元素 8 字节，且多一次堆指针间接访问）。实际上最大偏移 `< 2032`，完全可以用结构体自带的定长数组 `alignas(64) uint16_t pos_offsets[64]`，体积缩小 4 倍且紧贴 L1 Cache。

#### 修改方案 & 收益评估
- AVX-512 扩为 **32-lag（`p0..p3` + `n0..n3` 或单循环 4 路交错）**，AVX2 扩为 **16-lag（4 路 YMM）**；偏移表改 `uint16_t` 定长数组。
- **预期提升**：纯稀疏匹配滤波内核吞吐提升 **1.8×–2.5×**（对宽窗搜索、`fold_acquire`、`correlate_cir_sparse` 均直接生效）。

---

### MF-2：修复 `sparse_ternary_dot_single` 的分支与串行链

- [ ] **勾选修改 MF-2**

#### 问题原理
[sparse_ternary_dot_single](../../gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h#L61-L74) 在 SFD 离散网格（80 次）、`polarity_ok`（16 次）、前导沿消歧（20+ 次）等单点场景被频繁调用。目前代码是：
```cpp
for (const auto& e : tmpl.entries) {
    if (e.sign > 0) acc += rx[e.pos];
    else            acc -= rx[e.pos];
}
```
这里有两个大忌：**循环内 `if/else` 分支**（取决于伪随机码符号，CPU 分支预测器几乎必错，每次错判罚 15–20 周期）和 **单复数串行累加链**。

#### 修改方案 & 收益评估
- 直接复用已分好组的 `pos_offsets` 和 `neg_offsets`（彻底消灭 `if` 分支），用 4 个独立复数累加器（或 SSE/AVX2 把 4 个 `complex<float>` 打包进一个 YMM 做水平归约）。
- **预期提升**：单点稀疏相关速度提升 **3×–5×**。

---

### MF-3：滑窗能量归一化（分母）的彻底瘦身

- [ ] **勾选修改 MF-3**

#### 问题原理
把匹配滤波从 1016 次乘加优化到 64 次加减后，**分母（滑窗信号能量 $E_w = \sum |rx|^2$ 与浮点除法）往往反客为主，变成比分子还慢的瓶颈**：
1. [scan_normalized_sync](../../gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h#L275-L294) 在不需要保存 `metrics` 数组（仅找最大峰）时，**每个 lag 都在做标量浮点除法 `std::norm(corr) / (power * Et + eps)`**。除法指令延迟高达 11–14 周期且不可流水化。
   - **代码技巧（交叉相乘去除法）**：比较 $\frac{N_1}{D_1} > \frac{N_2}{D_2}$（且 $D_1, D_2 > 0$）完全等价于 $N_1 \cdot D_2 > N_2 \cdot D_1$！只用乘法跟踪最大值，整段窗口**只在最后返回赢家时做 1 次除法**。
2. [raw_lag_par_passes](../../gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h#L321-L350) 在 PAR 模式下先写入 `powers[]` 数组，然后再从头到尾遍历一遍 `powers[]` 算背景均值。其实在 16/32-lag 刚出寄存器时直接累加整窗总功率 `total_sum`，最后减去峰周围 `±12` 个 guard 点功率就得到 `background_sum`（注意 guard 仅 25 个点，几乎零成本）。
3. 衔接今天的 [原始相关功率PAR判决实验_2026-11-19.md](原始相关功率PAR判决实验_2026-11-19.md#L127-L156)：文中提到 `window3`（3 点功率和）在 SFD 上零误报但速度与单点持平，而在 SYNC 上变慢（`-12 dB` 从 `626 µs` 升到 `848 µs`），原因正是三点 PAR 天然只有单点的 `57%`（两径 `65%`），却沿用了单点的数值门限 `20` 导致更常回退折叠。若针对 3 点分布重标定 SYNC 门限（如降到 `11–13`），或结合下节 MF-4 的相干滤波，即可兼顾零误报与高速度。

#### 预期提升
- 消除逐 lag 除法与二次扫描后，窄窗与 `grid_track` 相关扫描预计再提速 **15%–30%**。

---

### MF-4（数学级突破）：用因子分解 $(r \star \text{code}) \star p$ 统一“稀疏速度”与“稠密低 SNR 灵敏度”

- [ ] **勾选修改 MF-4**

#### 原理解析（强烈建议关注的信号处理技巧）
[开发状态.md](../../开发状态.md#L108-L110) 留了一个关键的 TODO：
> `--timing-sparse on` 虽然快 ~7×，但在 `-12 dB / -14 dB` 低 SNR 下的检测损失和与脉冲成形模板的 2 样点群延迟偏移尚未验证，因此 `Qm35825Profile` 默认仍保留稠密路径。

为什么会存在这种“速度 vs 灵敏度”的矛盾？因为：
- **纯稀疏模板**（64 个间隔 8 样点的冲激）只采样了每个脉冲的 1 个点，丢掉了脉冲其余部分的能量，也没做脉冲成形匹配滤波（无法抑制脉冲滚降带外的噪声），所以低 SNR 下损失约 **1.5–3 dB**，且峰形变窄、受分数样点偏置更敏感。
- **稠密模板**（1016 个非零复数）其实就是**稀疏码 $\text{code}$ 与单个发送脉冲 $p$ 的卷积**：$t = \text{code} * p$。脉冲 $p$ 的主瓣能量集中在短短 **5–9 个样点**内！

根据卷积/互相关的结合律：
$$r \star (\text{code} * p) = (r \star \text{code}) \star p$$
这意味着我们**完全不需要做 1016 次乘法**，只需要：
1. 先用极快的 64 点稀疏加减核算出中间结果 $c[\text{lag}] = (r \star \text{code})[\text{lag}]$（无任何乘法，16/32-lag 并行）；
2. 再对 $c[\text{lag}]$ 做一次极短的 **5–9 tap 实数对称 FIR**（即 $\star p$）。

#### 为什么不用 FFT 做互相关？
这是一个常见疑问。简单算一笔帐就清楚了：
- 2048 点基-2 FFT + IFFT + 复乘，即使全用 SIMD，折算到每个输出 lag 上也需要 **100+ 次浮点乘加**，且有蝶形跨步访存和线程安全/临时缓冲区开销；
- 64 点稀疏三元码时域 SIMD 每个 lag 只需要 **64 次加减**，连续顺序访存、无乘法、零 FFT 准备开销，**时域全场景比 FFT 更快**。
- 加上 5–9 tap 短 FIR 后，也仅增加 ~10% 算力，**依然比 1016 点稠密快 6–10×**，却在数学上**相干恢复了完整脉冲匹配滤波增益与群延迟对齐**！而且它比“3 点功率非相干求和（`window3`）”更优——相干合并能压低噪声方差并直接输出单点尖峰，完美兼容现有单点 PAR 判决。

#### 预期提升
- 若验证通过，可**彻底淘汰 1016 点稠密模板**，在 `-12/-14 dB` 低 SNR 下实现与稠密路径同等的灵敏度，同时获得接近纯稀疏路径的 **~5×–7× 加速**。

---

## 三、Timing 阶段优化

### TM-1：`AcquireLegacy` 宽窗盲搜的算法级降维：$O(2 \cdot \text{margin}) \to O(P) + O(\text{margin}/P)$

- [ ] **勾选修改 TM-1**

#### 问题原理
在盲采模式（`AcquireLegacy`）下，[uwb_dw3000_live.cc](../../gr-uwb/apps/uwb_dw3000_live.cc#L591) 将搜索半径设为 `timing_search_margin = 131072`（全宽 **262,144 个样点**）。
查看 [uwb_demod_timing.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h#L568-L643)：
- 默认 `timing_coarse_stride = 14`，代码就会落入 `else` 分支：跑 **18,725 次标量单点相关**，加上 262k 次滑窗功率推进；
- 即使改用 `stride = 2`，依然要扫 **131,072 个半速 lag**，在 9700X 上占了 Acquire 模式 `demod_us = 845.3 µs` 的大头。

#### 数学优化技巧（利用周期性解耦“相位”与“起点”）
UWB 前导是 **64 个或 256 个完全一模一样的 SYNC 符号**，周期严格为 $P = 1016$。如果包在窗内，那么**在包前导内部的任意一段连续 $P = 1016$ 样点窗口里，必然恰好出现 1 次完整的码相位对齐**！
因此完全不需要在 262,144 个位置上逐一搜索：
1. **第一步（找精确码相位 $\phi \in [0, 1015]$）**：对于 `256` SYNC（长度 ~260k > 2×131k），只需在 `seed` 附近取 **1 个长度为 $P=1016$ 的探针窗**，做全速（`stride=1`）批量稀疏相关，直接锁定精确样本相位 $\phi$（对于 `64` SYNC，前导长 ~65k，取间距 `32*P` 的 2 个 $P$ 长度探针即可 100% 覆盖 `±131k` 窗）。若低 SNR 单符号不够高，可将探针做 4–8 符号的相干/非相干折叠，依然只有 1016 个 lag！
2. **第二步（沿网格找上升沿）**：一旦知道 $\phi$，真正的包起点只可能落在 $\phi + m \cdot 1016$ 上。全窗最多只有 ~256 个网格点（用单点或二分搜索只需 **8–20 次**单点相关）即可精确找出第 0 个 SYNC。

#### 预期提升
- 需要计算的相关点数从 **131,072 ~ 262,144 个 lag 暴降到 1,016 ~ 2,032 个 lag**（计算量降 **>95%**）；
- 同时**彻底消除了 `stride=14/16` 造成的相位混叠漏检风险**。预计 Acquire 宽窗 `demod_us`（`845 µs` → **~300–400 µs**）可再提速 **2× 以上**，大幅减少队列丢包。

---

### TM-2：`LockedBounded` 首探 `m=0` 先窄后宽

- [ ] **勾选修改 TM-2**

#### 问题原理
[uwb_demod_timing.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h#L324-L345) 中，锁定态每次都要对 `m = 0`（预测的第一个 SYNC）扫描完整的 `±locked_roi_samples`（`±256` = **513 个 lag**）。但在稳态锁定下，时钟跟踪残差通常只有 **±2 到 ±10 个样点**。

#### 修改方案 & 收益评估
- 先在 `±32`（**65 个 lag**，恰好满足 `count >= 64` 的 PAR 背景门槛）内扫描；只有 `±32` 未命中时才补扫外环 `[-256, -33]` 与 `[33, 256]`。
- **预期提升**：稳态包首探计算量直接降为 **1/8**（513 → 65 lag）。

---

### TM-3：`LockedBounded` `grid_track` 去掉 2 MB 内存分母重算

- [ ] **勾选修改 TM-3**

#### 问题原理
开启 `--timing-sparse on` 后，[grid_track](../../gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h#L863-L922) 里每包对 255 个 SYNC 各做一次 8-lag 相关。注意到一个惊人的事实：
- 8 个连续 lag 的**稀疏相关**只读 64 个非零点；
- 但为了算归一化分母 `pw = window_power_8acc(rx + base, 1016)`，**每个 SYNC 都要把连续 1016 个复数（8 KiB）全部读一遍**，255 个 SYNC 合计读取 **~2 MB 内存**！分母耗时反而是稀疏相关分子的 **2–4 倍**！

#### 修改方案 & 收益评估
1. 8 个连续 lag 窗口重叠了 `1009 / 1016 = 99.3%`，窗口总能量几乎不变，**完全可以先用纯相关幅度 $|corr|^2$ 选出 8 个 lag 中的最大值**，再仅对这 1 个胜出位置算归一化度量（甚至对低 SNR/稳态可用间隔估计或稀疏采样能量近似，仅在接近门限 `gthr` 时才算精确 1016 点能量）。
- **预期提升**：`grid_track` 阶段读内存带宽与耗时预计减少 **50%–70%**（9700X 稳态 `demod_us` 可再降 **~30–60 µs**）。

---

### TM-4：放宽 `try_reuse_initial_tail_timing` 复用条件

- [ ] **勾选修改 TM-4**

#### 问题原理
[uwb_demod_pipeline.cc](../../gr-uwb/lib/uwb_demod_pipeline.cc#L199-L221) 原本打算在 SFD 锚定后直接复用 Stage 1 已经跟踪好的末尾 40 个 SYNC 峰值来算 CFO，省去重跑 `stage_timing`。但它的准入条件极其严苛：要求 `detected_peaks == 256` **一个都不能少**，且 `cfg.tail_start == expected_tail_start` **0 样点偏差**。实际中只要前导开头受轻微干扰少检 1 个 SYNC，或周期拟合差半个样点，就会整段作废，重跑 40 次局部搜索（约 713 lag + 55 次 1016 点能量求和）。

#### 修改方案 & 收益评估
- 改为在 Stage 1 已有的 `peak_samples` / `peak_corr` 中按 `[tail_first .. reps-1]` 坐标就近匹配（容差 `±2` 样点），只对缺失的极少数符号补算单次 8-lag。
- **预期提升**：在中高 SNR 及轻微前导受损包上，彻底免去二次 `stage_timing`（节省 **~30–80 µs/包**）。

---

### TM-5：Fold 折叠回退路径优化（低 SNR `-12 / -14 dB`）

- [ ] **勾选修改 TM-5**

#### 问题原理与优化点
在 [uwb_demod_sync.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h) 中：
1. `probe_fold_phase`（[L150-L168](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h#L150-L168)）：计算 `ea`（样本 `[0..count)` 能量）和 `eb`（样本 `[1016..1016+count)` 能量）时，中间 **129,032 个样点完全重叠**，却独立扫了两遍内存。算完 `ea` 后只需减去前 1016 点、加上后 1016 点即可得 `eb`；且粗测 CFO 相位只需 32–64 符号，不必扫满 128 符号。
2. `fold_accumulate`（[L170-L186](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h#L170-L186)）：当存在大 CFO（如 `+40 kHz` 导致 `phase != 0.0`）时，走的是**纯标量**循环。将其改为类似 `derotate_block_anchored` 的 AVX2/AVX-512 复标量 FMA，可提速 4–8×。
3. **复用折叠结果**：`fold_acquire` 折了前 64 符号，紧接着 `fold_anchor_fit` 又把前 64 符号分成 4 组每组 16 符号重折了一遍——其实可以先折 4 组 16 符号，再把 4 组加起来给 `fold_acquire` 用。

#### 预期提升
- 低 SNR 进入折叠兜底的包（`-12 dB` 部分包 / `-14 dB` 全部包），`fold` 总耗时预计降低 **30%–50%**。

---

## 四、SFD 阶段优化

### SFD-1：`fold_sfd_window` 单遍 8-way 合并

- [ ] **勾选修改 SFD-1**

#### 原理与收益
在 [uwb_demod_sic_avx512.cc](../../gr-uwb/lib/uwb_demod_sic_avx512.cc#L180-L213)（以及 AVX2 版 [uwb_rx_sync_opt.h](../../gr-uwb/include/gnuradio/uwb/uwb_rx_sync_opt.h#L520-L561)）中，`fold_sfd_window` 对输出数组 `out[0..fold_len)` 做了 **8 遍**独立循环（第 1 遍 `memcpy`/`sub`，后面每遍都把 `out` 从 L1 读出来、加上一个符号、再写回 `out`）。
- **代码技巧（多流合并写入）**：对 `4z2` 等已知非零符号（共 8 个符号，对应 8 个指针 `rx_base + i * 1016`），在一个循环里按 16 字节/32 字节/64 字节步进，**一次性把 8 个指针的数据加载到寄存器里加减好，最后只写 1 次 `out`**！写内存流量直接降为 **1/8**。
- **预期提升**：`fold_sfd_window` 耗时降低 **~50%–70%**（单次窄窗 `sfd_boot` / `sfd_final` 可压至几微秒）。

---

### SFD-2：免去无意义的 `run_final_sfd` 重跑

- [ ] **勾选修改 SFD-2**

#### 原理与收益
在 [uwb_demod_pipeline.cc](../../gr-uwb/lib/uwb_demod_pipeline.cc#L505-L643) 中：
- 如果 `cfo_refit_mode` 选择跳过第二次去旋转（`cfo_refit_action == 1` 或 `2`），`scratch.derotated` 在 SFD 区间**连一个 bit 都没有变过**，但第 643 行依然会把 `run_final_sfd()` 完整重跑一遍！
- 即使做了第二次去旋转，只要 `|refit_delta_hz|` 在 8 个符号（`8.1 µs`）内累积的相位差很小（例如 `< 0.05 rad`），SFD 的峰值位置根本不可能移动超过 1–2 个样点，完全不必重跑全套 `±64` + `±1..±4` 符号窗口 + 离散网格。
- **预期提升**：跳过或极窄化（`±4` 样点）`Final SFD` 后，在低 SNR（若原本进入过 `±1..±4` 偏移或网格）可直接省去 **15–200 µs**。

---

### SFD-3：离散网格细节清理与任意 SFO 支持

- [ ] **勾选修改 SFD-3**

#### 问题与优化点
在 [uwb_demod_sfd.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sfd.h) 中发现 4 处细节问题：
1. **重复算 `d = 0`**（L402-L411）：找到最佳网格点 `gpos` 后，`d ∈ {-1, 0, +1}` 精修把 `d = 0` 又算了一遍；
2. **重复算 `polarity_ok`**（L435）：当 `best_pos == gpos` 且走了 `sym_decompose` 路径时，8 个符号的相关值早已保存在 `sym_corr` 里，`polarity_ok` 却又用 `sync_filter.dot` 重新算了一遍（多算 8–16 次完整点积）；
3. **`+27 ppm` SFO 退化陷阱**（L289）：`std::llround(kGridHalf * (period - sym)) == 0` 在 `+27 ppm` 时，由于 `36 * 0.027 ≈ 0.97` 四舍五入为 `1`，**会导致 `sym_decompose` 优化被关闭**！离散网格从 `80` 次点积瞬间退化为 `73 × 8 = 584` 次完整点积（暴慢 7 倍）。只需按实际样本坐标做去重缓存即可兼容任意 SFO；
4. **含 `0` 序列的逻辑隐患**（L138）：`if (par_only && sfd_sequence[si] == 0) continue;` —— 若未来在 `par_only == false` 下使用含 `0` 的 SFD（如 `ieee`/`decawave`），`proj * 0 > 0.0` 会恒为 `false` 导致误拒，应直接写 `if (sfd_sequence[si] == 0) continue;`。

---

### SFD-4：软芯片域 `stage_ns_sfd` 稀疏向量化

- [ ] **勾选修改 SFD-4**

- [uwb_demod_phr.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_phr.h#L94-L97) 中对 `soft_chips` 搜索 `±8` 芯片（17 个 lag）时，遍历 512 个稀疏条目用了带 `entry.sign > 0` 分支的标量循环。拆成正负偏移表并用 AVX2/AVX-512 向量化（类似 `MF-2`），可把该阶段压到 ~1–2 µs。

---

## 五、CIR 估计与跨阶段架构融合（最大单点收益）

### CIR-1（高收益）：融合 CFO 去旋转与 SYNC 相干平均 + 共享末尾 32 SYNC

- [ ] **勾选修改 CIR-1**

#### 原理详解（最值得学习的数学结合律技巧！）
当前架构中，为了算 `estimate_cir`，流水线在 `stage_sfd` 之后必须先执行一次**昂贵的 CFO 去旋转**：把整个前导 246 个 SYNC（约 **250,000 个复数 = 2 MB 内存**）从原始 `rx` 读出、乘以 $e^{-j\omega n}$、写进 `scratch.derotated`；紧接着 `estimate_cir` 再把这 2 MB 从 `scratch.derotated` 读出来做相干平均！

但在 **Sensing 模式（`--stop-after-cir on`）** 下，前导后面的负载根本不解调；甚至在 **Comm 默认模式（`--fused on`，即 `skip_sync_fir == true`）** 下，查阅 [uwb_demod_execution.h](../../gr-uwb/include/gnuradio/uwb/uwb_demod_execution.h#L94-L107) 和 [uwb_rx_softchip_opt.h](../../gr-uwb/include/gnuradio/uwb/uwb_rx_softchip_opt.h#L62-L101) 可知：**前导区域的 `scratch.derotated` 仅仅被 `estimate_cir` 和 `sync_phase_gain`（末尾 32 SYNC）读取，后续 PHR/Payload 根本不读前导！**

现在来看一个简单却威力巨大的数学恒等式。第 $k$ 个 SYNC 的局部窗起始位置是 $\text{lo}_k$（长度 $W = 1053$），把旋转因子拆开：
$$\sum_{k} r[\text{lo}_k + m]\,e^{-j\omega(\text{lo}_k + m)} = \underbrace{e^{-j\omega m}}_{\text{循环外乘 1 次 (}1053\text{ 点)}} \cdot \sum_{k} \underbrace{e^{-j\omega\,\text{lo}_k}}_{\text{每符号 1 个标量}} \cdot\, r[\text{lo}_k + m]$$

**这意味着什么？**
1. `estimate_cir` **直接读原始未旋转的 `rx`**，完全不需要对 250k 个前导样点做去旋转！
2. 对每个 SYNC $k$，只需算一个复标量 $\alpha_k = e^{-j\omega\,\text{lo}_k}$，做一次向量化乘加 `avg[m] += alpha_k * rx[lo_k + m]`（`avg` 只有 `1053 × 8B ≈ 8.4 KB`，全程死死锁在 CPU **L1 Cache** 里）；
3. 顺便把 `avg` 分成前段和末尾 32 个 SYNC（`avg_tail`）两段累加。循环结束后只对这 `1053` 个样点乘一次 $e^{-j\omega m}$；
4. Comm 模式下的 `sync_phase_gain` 根本不需要再去读 32×1052 样点算 `sum`，**直接复用 `avg_tail`**（其本质就是用 CIR 抽头对 `avg_tail` 做一次稀疏码加权点积，计算量几乎为零）！
5. **SIC 路径同理**：[uwb_discrete_cir_estimator.cc](../../gr-uwb/lib/uwb_discrete_cir_estimator.cc#L1132-L1168) 目前为 `64 × 1016 = 65,024` 样点构建 1 MB 双精度 `cfo_rot` 表；用同样因式分解只需 `1035` 项旋转表！

#### 预期提升
- **Sensing 模式（`--stop-after-cir on`）**：CFO 去旋转范围从 ~250k 样点缩减到仅 SFD 窗（~9k 样点），**去旋转计算量暴降 ~95%**，预计 Sensing `demod_us`（9700X 当前 `194.1 µs`）可再降 **30%–45%**（接近 ~110–130 µs）！
- **Comm 模式（`--fused on`）**：二次去旋转范围缩减 **~85%**，且彻底免去 `sync_phase_gain` 约 32k 样点重复读取，预计全包 `demod_us` 再降 **40–80 µs**。
- **数值等价性**：无论是整数还是分数 `measured_period`，该恒等式在数学上**完全等价**（仅存在浮点结合律极微差异）。

---

### CIR-2：`estimate_cir` 自身微调

- [ ] **勾选修改 CIR-2**

1. **补零对齐满宽 SIMD**：`tap_count = 38` 时，16-lag/32-lag 会留下尾项。在 `scratch.corr` 尾部预留 10 个零，将 `tap_count` 向上补齐到 48（或 40），直接走满宽无分支循环；
2. **去除可约掉的标量缩放**：如果上游只需归一化 CIR（即 `values` 后面立刻要除以自身 L2 范数 `nrm`），那么前面乘以 `1/valid`（1053 次浮点乘法）和 `correlate_cir_sparse` 里的 `/ code_energy`（38 次浮点除法）在最终归一化时**会作为正标量被完全约掉**，可以直接省去（注意：若要保存物理幅度 `raw_taps` 如 Radar/SIC 路径，则保留标量缩放）。

---

### CIR-3：大 SFO（如 `+27 ppm`）下的 4 桶分数相位相干平均

- [ ] **勾选修改 CIR-3**

#### 问题原理
当存在 `+27 ppm` 时钟偏差时，246 个 SYNC 之间累积漂移达 **~6.75 个样点**。MATLAB 的 `estimateCir.m` 是用 `interp1` 对每个 SYNC 做线性插值再平均；而 C++ 版 `estimate_cir` 用 `std::llround(k * period)` 四舍五入取整，引入了 ±0.5 样点（~0.5 ns）的量化展宽，在高速率/强多径下会轻微模糊信道主瓣。

#### 修改方案（4 桶分相法）
不需要对 246 个 SYNC 每个都做昂贵的线性插值！只需要在 `integer_grid_ok == false` 时：
- 计算每个 SYNC 的小数偏移 $f_k = k \cdot \text{period} - \text{llround}(k \cdot \text{period}) \in [-0.5, 0.5]$，按四舍五入分到 **4 个子相位桶**（`-0.25, 0, +0.25, +0.5` 或 `[0, 0.25, 0.5, 0.75)`）；
- 246 个 SYNC 直接在各自所属的 4 个桶内做**纯整数对齐向量累加**；
- 循环结束后，**只对这 4 个长度为 1053 的桶做一次线性插值合并**，总插值成本降为 **4 / 246 ≈ 1.6%**！

---

### CIR-4（Radar 专项）：将 Radar 路径迁移到稀疏 SIMD 核

- [ ] **勾选修改 CIR-4**

#### 问题原理
相比 Comm/Sensing 链路已经被高度向量化，Radar 路径的几个核心头文件目前还停留在“纯标量时代”：
1. [uwb_radar_cir_estimator.h](../../gr-uwb/include/gnuradio/uwb/uwb_radar_cir_estimator.h#L258-L267) 的 `estimate_radar_cir` **仍在对 1016 个样点（其中 952 个是 0）跑双精度 `complex<double>` 标量稠密循环**；
2. 同文件的 `estimate_radar_cir_repetition`（[L349-L376](../../gr-uwb/include/gnuradio/uwb/uwb_radar_cir_estimator.h#L349-L376)）虽然使用了 `active_code_indices` 跳过零点，但仍是标量 `double` 复乘循环（且没利用码片 $\in \{-1, +1\}$ 纯实数免乘法特性）。在逐 repetition 实时落盘模式下（128 rep × 116 tap，200 Hz 实测单包 **3.45 ms**），这里几乎全是算力浪费；
3. [uwb_radar_sfd_core.h](../../gr-uwb/include/gnuradio/uwb/uwb_radar_sfd_core.h#L164) 与 [uwb_radar_timing_core.h](../../gr-uwb/include/gnuradio/uwb/uwb_radar_timing_core.h#L151) 仍在调用标量 `conjugate_dot_scalar`。

#### 修改方案 & 收益评估
- 若 Radar QA 允许 float 精度（或按 4/8-lag `double` 稀疏加减写一个无乘法的 AVX2/AVX-512 核），接入稀疏/SIMD 核后：
  - `estimate_radar_cir_repetition`（128 rep × 116 tap）预计提速 **8×–12×**（从 ~3.45 ms 压至 **~0.3–0.5 ms** 级别，为 300–500 Hz 实时 Radar 解锁充裕 CPU 余量）；
  - Radar SFD/Timing 复用 `fold_sfd_window` + `SyncMatchedFilter` 预计提速 **5×–10×**。

---

## 六、综合性能评估汇总

如果按上述清单分步落地，预期整机性能变化如下（注：以下估算针对 `demod_us`；由于实采端到端 `compute_us` 中 `resample_us` 固定占约 606–619 µs，端到端总计算时间降幅会按比例折算）：

| 场景 | 当前实测基线 (`demod_us`) | 预期优化范围（实施 P0 + P1 主要项后） | 核心贡献项 |
|---|---:|---:|---|
| **9700X Comm Locked 稳态**（`--timing-sparse on`） | `288.2 µs` | **~160–200 µs**（-30%~45%） | P0-1(AVX-512 16/32-lag)、TM-2(首探先窄后宽)、TM-3(`grid_track`去分母)、CIR-1(去前导去旋转+复用`avg_tail`) |
| **9700X Sensing CIR-Only**（`--stop-after-cir on`） | `194.1 µs` | **~100–130 µs**（-33%~48%） | P0-1、TM-2/TM-3、**CIR-1**（Sensing 几乎完全消除前导 CFO 去旋转） |
| **9700X Acquire 宽窗盲搜**（`path=acquire`） | `845.3 µs` | **~250–380 µs**（2.2×–3.4×） | **TM-1**（$O(2\cdot\text{margin}) \to O(P)$）、P0-1、MF-1、CIR-1 |
| **i7-12700 低 SNR `-12 dB` Locked** | `1383 µs` | **~900–1100 µs**（-20%~35%） | **P0-2**（修 `retrack_tail` 双倍必败）、TM-5（Fold 优化）、MF-1/3 |
| **低 SNR `-12/-14 dB` 稠密灵敏度对齐** | 稀疏约损失 1.5–3 dB | 恢复稠密级灵敏度，耗时仅比纯稀疏多 ~10% | **MF-4** $(r \star \text{code}) \star p$ 因子分解（待 MATLAB 交叉验证） |
| **Radar 逐 repetition CIR**（128 rep × 116 tap） | `~3452 µs` | **~300–500 µs**（~7×–10×） | **CIR-4**（稀疏 + AVX2/AVX-512） |
