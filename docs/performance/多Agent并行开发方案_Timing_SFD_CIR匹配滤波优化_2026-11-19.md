# 多 Agent 并行开发与执行方案：Timing / SFD / CIR 与匹配滤波优化（供 OpenCode 执行）

> **日期**：2026-11-19  
> **基线分支**：`feature/sic-research`（基于 `b654733`）  
> **技术依据**：[Timing_SFD_CIR与匹配滤波优化分析_2026-11-19.md](Timing_SFD_CIR与匹配滤波优化分析_2026-11-19.md)  
> **范围边界**：覆盖上述分析文档中除 **CIR-4（Radar 专项）** 外的全部 17 项优化（`P0-1~3`、`MF-1~4`、`TM-1~5`、`SFD-1~4`、`CIR-1~3`）。**禁止修改** `uwb_radar_*` 头文件及 Radar 相关测试/逻辑。

---

## 1. 全局调度拓扑与防冲突边界

### 1.1 波次划分（3 个 Wave，同波次内零文件重叠）

```text
Base (feature/sic-research @ b654733)
  │
  ├── Wave 1（底层内核与 Scratch 基座，2 路并行，零文件冲突）
  │     ├── Agent-MF-Kernel     ── 独占 uwb_demod_sic_avx512.{h,cc}, uwb_rx_matched_filter.h,
  │     │                            uwb_rx_sync_opt.h, uwb_matched_filter.h, qa_uwb_demod_sic_avx512.cc
  │     └── Agent-Scratch-NsSfd ── 独占 uwb_demod_scratch.h, uwb_demod_phr.h
  │
  ├── [Integrator 合并 Wave 1 + 全量 CTest (AVX512/AVX2/SCALAR)] ──> tag: opt-w1
  │
  ├── Wave 2（上层 Timing / SFD / CIR-Pipeline，3 路并行，零文件冲突）
  │     ├── Agent-Timing        ── 独占 uwb_demod_timing.h, uwb_demod_sync.h
  │     ├── Agent-SFD           ── 独占 uwb_demod_sfd.h
  │     └── Agent-CIR-Pipeline  ── 独占 uwb_demod_softchips.h, uwb_demod_execution.h,
  │                                  uwb_rx_softchip_opt.h, uwb_demod_pipeline.cc,
  │                                  uwb_discrete_cir_estimator.cc, qa_uwb_demod_core.cc
  │
  ├── [Integrator 合并 Wave 2 + 全量 CTest + 6x40 固定窗回归] ──> tag: opt-w2
  │
  └── Wave 3（因子分解 MF-4 + 完整 ABBA 性能验收）
        └── Agent-MF4-Eval      ── 实现 (r ⋆ code) ⋆ p 因子分解开关 + 运行 ABBA 与回归矩阵
```

### 1.2 所有 Agent 必须遵守的硬约束（`AGENTS.md`）

1. **Git Worktree 隔离**：每个 Agent 在独立 worktree 工作（`.worktrees/<agent-name>`，分支 `exp/<agent-name>`），严禁跨区修改非本 Agent 独占文件。
2. **构建与测试环境**：
   - 使用 `env -u LD_LIBRARY_PATH` 运行所有测试和基准，防止命中系统旧 `.so`；禁止把 `cmake --install` 当作测试前置。
   - **禁止添加 `-march=native`**，必须严格遵守 `UwbSimd.cmake` 的编译期隔离契约（AVX-512 指令只允许出现在 `uwb_demod_sic_avx512.cc` 的 `__attribute__((target("avx512f,avx512bw,avx512dq,avx512vl")))` 函数中）。
   - 统一构建验证命令（按当前主机 `AVX512` 与 `AVX2` 两档至少跑通一档，Integrator 须跑通 `AVX512` + `AVX2`）：
     ```bash
     cmake -S gr-uwb -B gr-uwb/build-opt -DCMAKE_BUILD_TYPE=Release \
       -DUWB_SIMD=AVX512 -DENABLE_UHD_BACKEND=OFF
     cmake --build gr-uwb/build-opt -j
     env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-opt --output-on-failure
     ```
3. **契约保留**：
   - 保留分数网格回退、`Full -> Bypass` 降级、`timed` 无 replay fail-closed；
   - 不得触碰 `uwb_radar_cir_estimator.h`、`uwb_radar_cir_core.h`、`uwb_radar_sfd_core.h`、`uwb_radar_timing_core.h`；
   - 不放宽既有 QA 容差；若新增更宽 SIMD 核（如 32-lag）改变了累加树而无法与标量逐位相等，须保留原测试并用独立数值参考（如双精度参考）验证容差，不得静默删除旧断言。

---

## 2. Wave 1 任务书与下发 Prompt

### Subagent 1-A：`Agent-MF-Kernel`

- **Worktree / 分支**：`.worktrees/opt-mf-kernel` / `exp/opt-mf-kernel`（基于 `b654733`）
- **独占文件**：
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_sic_avx512.h`
  - `gr-uwb/lib/uwb_demod_sic_avx512.cc`
  - `gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_rx_sync_opt.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_matched_filter.h`
  - `gr-uwb/lib/qa_uwb_demod_sic_avx512.cc`
- **涵盖优化项**：`P0-1`、`MF-1`、`MF-2`、`MF-3`、`SFD-1`

#### 详细技术要求
1. **`P0-1` 修复 AVX-512 死代码与尾项级联**：
   - 将 `sparse_ternary_dot_16lag`（并扩展为 32-lag）和 `pack_even_complex` 的 AVX-512 实现移入 `uwb_demod_sic_avx512.cc`（带 `UWB_DS_AVX512_TARGET` 属性），在 `uwb_demod_sic_avx512.h` 声明纯 POD 参数接口（如 `const uint16_t* pos_offsets, size_t pos_count, const uint16_t* neg_offsets, size_t neg_count`）。
   - 在 `uwb_rx_matched_filter.h` 通过 `use_avx512(KernelGroup::Sync)` 运行时分发（与 `real_template_dot_8lag` 完全一致），彻底移除依赖 `#if defined(__AVX512F__)` 的死代码。
   - 在 `SyncMatchedFilter::for_each_lag` 中实现 **`32/16 → 8 → 4 → 1` 级联**（保证 `lag_count == 8`、`38`、`20` 绝不掉进 8 次或 6 次标量 tail）。
2. **`MF-1` 微架构提速**：
   - `SparseCirTemplate` 中增加紧致定长偏移数组 `alignas(64) uint16_t pos_off[64], neg_off[64]` 及计数（保留原 `entries`/`pos_offsets`/`neg_offsets` 兼容性或同步填充），消除热路径 `std::vector` 指针间接寻址。
   - AVX-512 实现 **32-lag** 核（4 条独立 `p0..p3` + `n0..n3` 或单循环 4 路交错累加打破 FADD 延迟墙，支持在 `c` 与 `c+1` 偏移相差 8 复数时做寄存器级复用）；AVX2 实现 **16-lag** 核（4×YMM）。
3. **`MF-2` 修复 `sparse_ternary_dot_single`**：
   - 移除 `if (e.sign > 0)` 分支，改为先无分支 4 路累加 `pos_off`、再 4 路累加 `neg_off`，支持 AVX2 水平归约最后合并减法。
4. **`MF-3` 滑窗能量分母瘦身**：
   - `scan_normalized_sync` 增加快路径：当 `metrics == nullptr` 时，比较 `std::norm(corr) * best_den > best_num * cur_den`（`cur_den = power * template_energy + kUwbEpsilon`），整窗只在最后算 1 次除法（注意先在 `power > 0` 守卫下避免溢出，可用 `double` 交叉相乘确保精度与等价性）。
   - `scan_raw_sync` / `raw_lag_par_passes`：支持在 `scan_raw_sync` 累加总功率 `total_sum`（或仅在 `raw_lag_par_passes` 改用 `total_sum - guard_sum` 避免二次全窗扫描；注意与 `three_samples=true` 边界契合）。
5. **`SFD-1` `fold_sfd_window` 单遍 8-way 合并**：
   - 当 `sfd_len == 8` 且全为 `±1`（如 `4z2`）时，在 `uwb_demod_sic_avx512.cc` 与 `uwb_rx_sync_opt.h` 提供特化单循环：按 16/8 复数步进一次加载 8 路源指针累加后只写 1 次 `out`。
6. **单测**：在 `qa_uwb_demod_sic_avx512.cc` 新增 16/32-lag 稀疏相关、`pack_even_complex`、8-way `fold_sfd_window`、`scan_normalized_sync` 交叉相乘与标量参考的逐样点数值一致性测试（容差 `1e-5`）。

#### 下发给 OpenCode 的 Prompt
```text
你是 Wave 1 的 Agent-MF-Kernel，请在独立 worktree `.worktrees/opt-mf-kernel`（分支 `exp/opt-mf-kernel`，基于 `b654733`）工作。
只允许修改以下文件：
- gr-uwb/include/gnuradio/uwb/uwb_demod_sic_avx512.h
- gr-uwb/lib/uwb_demod_sic_avx512.cc
- gr-uwb/include/gnuradio/uwb/uwb_rx_matched_filter.h
- gr-uwb/include/gnuradio/uwb/uwb_rx_sync_opt.h
- gr-uwb/include/gnuradio/uwb/uwb_matched_filter.h
- gr-uwb/lib/qa_uwb_demod_sic_avx512.cc

请阅读 docs/performance/Timing_SFD_CIR与匹配滤波优化分析_2026-11-19.md 中的 P0-1, MF-1, MF-2, MF-3, SFD-1，并完成以下改造：
1. 修复 UwbSimd.cmake 下 `__AVX512F__` 未定义导致的 16-lag 稀疏相关与 `pack_even_complex` 死代码：把宽指令内核迁入 `uwb_demod_sic_avx512.cc` 并通过 `use_avx512(KernelGroup::Sync)` 分发。
2. `SparseCirTemplate` 增加 `alignas(64) uint16_t pos_off[64]/neg_off[64]`；在 AVX-512 中实现 32-lag（4 路累加破 FADD 延迟墙）和 16-lag，在 AVX2 中实现 16-lag；在 `SyncMatchedFilter::for_each_lag` 实现 32→16→8→4→1 级联（确保 lag_count=8/20/38 不掉标量）。
3. 重写 `sparse_ternary_dot_single` 为无分支 pos/neg 4 路累加。
4. 优化 `scan_normalized_sync`（`metrics == nullptr` 时用 double 交叉相乘免逐 lag 除法）和 `raw_lag_par_passes`。
5. 优化 `fold_sfd_window`：对 8 符号全 ±1 SFD（如 4z2）实现单遍 8-way 合并累加，写内存降为 1/8。
6. 在 `qa_uwb_demod_sic_avx512.cc` 补齐新内核的精度与边界 QA（涵盖 lag=1..40）。

完成后构建 Release（UWB_SIMD=AVX512）并用 `env -u LD_LIBRARY_PATH ctest --output-on-failure` 全量通过。禁止修改 Radar 文件和 `-march=native`。
```

---

### Subagent 1-B：`Agent-Scratch-NsSfd`

- **Worktree / 分支**：`.worktrees/opt-scratch-nssfd` / `exp/opt-scratch-nssfd`（基于 `b654733`）
- **独占文件**：
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_scratch.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_phr.h`
- **涵盖优化项**：`P0-3`（Scratch 结构部分）、`SFD-4`、`CIR-1` 预留缓冲

#### 详细技术要求
1. **`DemodScratch` 扩充（为 Wave 2 消除热路径堆分配准备）**：
   - 为 `RealSyncTemplate` 增加 `uint64_t key = 0`（注：`RealSyncTemplate` 已定义 `key` 字段）及缓存支持；
   - 在 `DemodScratch` 新增预留缓冲区：`rxopt::SparseCirTemplate half_cir_template;`、`std::vector<std::complex<float>> fold_buf;`、`std::vector<std::complex<float>> fold_sup_buf;`、`std::vector<std::complex<float>> cir_avg_tail;`（长度 ≥ `1056`）、`std::vector<std::complex<float>> cir_phase_rot;`（长度 ≥ `1056`），并在 `reserve()` 中提前 `reserve`。
2. **`SFD-4` 软芯片域 `stage_ns_sfd` 稀疏向量化**：
   - `CachedSequenceState` 中把 `ns_sfd_sparse` 同步拆分为 `ns_sfd_pos_offsets` 和 `ns_sfd_neg_offsets`（`std::vector<uint32_t>`）。
   - `stage_ns_sfd` 中（[uwb_demod_phr.h#L93-L97](../../gr-uwb/include/gnuradio/uwb/uwb_demod_phr.h#L93-L97)）消除 `entry.sign > 0` 分支，改为无分支 4 路累加 `pos - neg`（支持 AVX2/标量）。
   - **注意**：`stage_ns_sfd` 要求结果与原标量版本在 `qa_uwb_demod_core.cc` 容差内完全一致。

#### 下发给 OpenCode 的 Prompt
```text
你是 Wave 1 的 Agent-Scratch-NsSfd，请在独立 worktree `.worktrees/opt-scratch-nssfd`（分支 `exp/opt-scratch-nssfd`，基于 `b654733`）工作。
只允许修改以下文件：
- gr-uwb/include/gnuradio/uwb/uwb_demod_scratch.h
- gr-uwb/include/gnuradio/uwb/uwb_demod_phr.h

请完成以下改造：
1. 在 `DemodScratch` 中为 Wave 2 预留复用缓冲：
   - `rxopt::SparseCirTemplate half_cir_template;`
   - `std::vector<std::complex<float>> fold_buf;`
   - `std::vector<std::complex<float>> fold_sup_buf;`
   - `std::vector<std::complex<float>> cir_avg_tail;`
   - `std::vector<std::complex<float>> cir_phase_rot;`
   并在 `DemodScratch::reserve()` 中合理预留容量（fold_buf/cir_avg_tail 等 >= 2048）。
2. 在 `CachedSequenceState` 增加 `ns_sfd_pos_offsets` / `ns_sfd_neg_offsets`（`std::vector<uint32_t>`），在 `update()` 及 `stage_ns_sfd` 缓存刷新处同步维护。
3. 优化 `stage_ns_sfd`（uwb_demod_phr.h）：消除内循环 `entry.sign > 0` 分支，改为无分支 4 路累加正负偏移（并在支持 `__AVX2__` 时用 4 路/8 路或保持浮点精度一致）。

完成后构建 Release（UWB_SIMD=AVX512）并运行 `env -u LD_LIBRARY_PATH ctest --output-on-failure` 全量通过。
```

---

## 3. Wave 2 任务书与下发 Prompt（基于 `opt-w1`）

> Wave 1 合并为 `opt-w1` 并通过全量 CTest 后，同时启动以下 3 个 Agent。

### Subagent 2-A：`Agent-Timing`

- **Worktree / 分支**：`.worktrees/opt-timing` / `exp/opt-timing`（基于 `opt-w1`）
- **独占文件**：
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h`
- **涵盖优化项**：`P0-3`（Timing/Fold 部分）、`TM-1`、`TM-2`、`TM-3`、`TM-5`

#### 详细技术要求
1. **`P0-3` 模板跳过与零分配**：
   - 在 `stage_timing` 开头：若 `sparse_tmpl != nullptr`（即 `--timing-sparse on`），**跳过** `scratch.work.assign`、`uwb_l2_normalize` 和 `build_real_sync_template`！仅在 `sparse_tmpl == nullptr` 时才构建，并利用 `RealSyncTemplate::key`（基于 `template_wf` 内容哈希）缓存命中跳过重建。
   - `fold_acquire` 和 `fold_anchor_fit` 改用 `scratch.fold_buf`、`scratch.fold_sup_buf`（不再局部构造 `std::vector`）；其 `sample_power_sum` 全部替换为 `window_power_8acc`。
   - `stride == 2` 分支改用 `scratch.half_cir_template`（按 `key` 缓存，不再每包新建 `SparseCirTemplate`）。
2. **`TM-1` `AcquireLegacy` 宽窗盲搜降维**：
   - 当 `!seed_acquired && j_hi > j_lo` 且窗口跨度 `j_hi - j_lo >= 2 * kQm35SamplesPerSymbol` 时，先在 `[j_lo, j_hi]` 内选取 `1` 个（`preamble_repetitions >= 128` 且 `margin <= 131072`）或 `2` 个（间距 `32 * 1016`，覆盖 `preamble_repetitions == 64`）长度为 `P = 1016` 的相位探针窗跑全速 `scan_normalized_sync`（若单符号不够强，可保留原低 SNR `try_fold_acquire` 兜底），锁定最优余数相位 $\phi = \text{peak} \bmod 1016$，然后在全局网格 $\phi + m \cdot 1016 \in [j\_lo, j\_hi]$ 上仅测网格点（使用 `local_best(..., radius=2)` 容忍轻微 SFO 漂移）定位最佳起点；仅当网格点均未达标时才回退至原 `stride` 粗扫或 `try_fold_acquire`。
   - **安全门控**：确保 `64` SYNC 短前导和低 SNR `-12 dB` 不会因为探针窗刚好落在前导外而漏检（探针坐标应以 `seed` 为中心对称覆盖 `±32*P`，并在未命中时无缝衔接 `try_fold_acquire`）。
3. **`TM-2` `LockedBounded` 首探先窄后宽**：
   - `m = 0` 时，若 `roi > 32`，先调用 `local_best(want, 32, thr, ..., allow_par_fallback=true, &raw_max)`；只有未命中时才补扫外环（或直接扫 `roi`），稳态将 `513 lag` 降为 `65 lag`。
4. **`TM-3` `grid_track` 去 2 MB 内存分母重算**：
   - 在 `grid_track` 的单点阶段（[uwb_demod_timing.h#L884-L907](../../gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h#L884-L907)）：对 `nl == 8` 的窗口，先用 `filter.for_each_lag` 仅计算 8 个复相关 `corr` 并以 `std::norm(corr)` 选出唯一胜出 `best_lag`（8 个连续 lag 共享 99.3% 样点），**只对 `base + best_lag` 调用一次 `window_power_8acc`** 做归一化！
   - 进一步优化：保留 `nl == 1` 边界处理；确保 `m` 和 `peak.corr` 与原算法仅在极少数平局边缘有 `<1e-4` 差异。
5. **`TM-5` Fold 折叠回退优化**：
   - `probe_fold_phase`：计算 `ea` 后通过首尾 `sym` 样点差分算 `eb`（省 1 次 130k 点扫描）；并将默认探测上限从 `128 * sym` 降为 `64 * sym`。
   - `fold_accumulate`（`phase != 0.0`）：用 AVX2/FMA 向量化（4 复数步进，类似 `derotate_block_anchored` 的 `permute + addsub` 复乘累加）。
   - `fold_acquire` 与 `fold_anchor_fit`：保留结果等价性。

#### 下发给 OpenCode 的 Prompt
```text
你是 Wave 2 的 Agent-Timing，请在独立 worktree `.worktrees/opt-timing`（分支 `exp/opt-timing`，基于 `opt-w1`）工作。
只允许修改以下文件：
- gr-uwb/include/gnuradio/uwb/uwb_demod_timing.h
- gr-uwb/include/gnuradio/uwb/uwb_demod_sync.h

请阅读 docs/performance/Timing_SFD_CIR与匹配滤波优化分析_2026-11-19.md 中的 P0-3, TM-1, TM-2, TM-3, TM-5，并完成以下改造：
1. P0-3：
   - `stage_timing` 当 `sparse_tmpl != nullptr` 时跳过 `scratch.work` 归一化与 `build_real_sync_template`；为 `build_real_sync_template` 加 `key` 缓存命中跳过。
   - `stride==2` 改用 `scratch.half_cir_template`（按 key 缓存）；`fold_acquire`/`fold_anchor_fit` 改传 `scratch.fold_buf`/`scratch.fold_sup_buf`（需要时加默认参数或重载），并将 `sample_power_sum` 改为 `window_power_8acc`。
2. TM-1：`AcquireLegacy` 宽窗盲搜在 `!seed_acquired` 且跨度 >= 2*P 时，先在 seed 附近取 1~2 个 P=1016 探针窗全速定码相位 phi，再沿 phi+m*P 网格找最高峰（未命中再走原 fallback / fold_acquire）。
3. TM-2：`LockedBounded` 首探 `m=0` 若 `roi > 32` 先扫 `±32`，未命中再扫 `±roi`。
4. TM-3：`grid_track` 的 8-lag 单点阶段先按 `std::norm(corr)` 选最大 lag，再只对胜出位置算一次 `window_power_8acc` 归一化。
5. TM-5：`probe_fold_phase` 用滑窗差分算 `eb`（上限 64*sym）；`fold_accumulate` 非零相位路径改用 AVX2 FMA 复数乘加。

完成后构建 Release（UWB_SIMD=AVX512）并运行 `env -u LD_LIBRARY_PATH ctest --output-on-failure` 全量通过。
```

---

### Subagent 2-B：`Agent-SFD`

- **Worktree / 分支**：`.worktrees/opt-sfd` / `exp/opt-sfd`（基于 `opt-w1`）
- **独占文件**：
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_sfd.h`
- **涵盖优化项**：`P0-3`（SFD 初始功率）、`SFD-3`

#### 详细技术要求
1. **`P0-3`**：将 [uwb_demod_sfd.h#L163](../../gr-uwb/include/gnuradio/uwb/uwb_demod_sfd.h#L163) 中的 `detail::sample_power_sum(rx + j_lo, sfd_len)` 替换为 `window_power_8acc(rx + j_lo, sfd_len)`。
2. **`SFD-3` 离散网格清理与任意 SFO 兼容**：
   - 修复第 138 行：`if (sfd_sequence[si] == 0) continue;`（去掉 `par_only &&`）。
   - 修复第 402-411 行：`d ∈ {-1, 0, +1}` 跳过 `d == 0`（已算过）。
   - 修复第 289 行：不再用 `llround(kGridHalf * (period - sym)) == 0` 门控 `sym_decompose`！对于任意 `period`（如 `+27 ppm` SFO），每个网格点 $k$ 的各符号起点为 `pos_k + s * sym`（或将同一 `(pos_k + s * sym)` 缓存，或者直接仅当任意两相邻网格点的符号坐标偏移差为 `sym` 时用缓存，若仅因 `llround(k * period)` 与 `k * sym` 差了 1 样点，只需对每个实际用到的符号起点 `spos` 在一个小哈希/排序表或按 `[expected - kGridHalf*period - 4 .. expected + (kGridHalf+len)*period + 4]` 仅当偏移刚好落在 `sym` 网格时复用，否则按需计算并缓存）。最简且覆盖 `+27 ppm`（73 符号总共只差 ~2 样点）的做法：保留 `sym_decompose` 结构，对 `pos = expected + llround(k * period)`，计算符号索引偏移，若 `llabs(pos - (expected + k * sym)) <= 2` 仍可按需懒计算并缓存到小数组，或直接对 `spos` 做局部小范围缓存命中复用！
   - 修复第 435 行：当 `best_pos == gpos` 且已通过 `sym_decompose` 算过各符号 `sym_corr` 时，直接用这 `len` 个 `sym_corr` 执行极性投影校验，省掉 `polarity_ok(best_pos)` 中的 8–16 次重复 `sync_filter.dot`。

#### 下发给 OpenCode 的 Prompt
```text
你是 Wave 2 的 Agent-SFD，请在独立 worktree `.worktrees/opt-sfd`（分支 `exp/opt-sfd`，基于 `opt-w1`）工作。
只允许修改以下文件：
- gr-uwb/include/gnuradio/uwb/uwb_demod_sfd.h

请阅读 docs/performance/Timing_SFD_CIR与匹配滤波优化分析_2026-11-19.md 中的 P0-3 和 SFD-3，并完成以下改造：
1. 将 `scan_window` 初始能量计算的 `sample_power_sum` 改为 `window_power_8acc`。
2. `polarity_ok` 内移除 `par_only &&` 前缀，统一跳过 `sfd_sequence[si] == 0`。
3. 离散网格 `d ∈ {-1, 0, +1}` 精修跳过 `d == 0`。
4. 改造离散网格的符号相关缓存，使其在存在 SFO（如 +27 ppm，llround(kGridHalf*(period-sym)) != 0）时依然能复用重复的符号坐标 dot 与能量（按实际符号起点坐标缓存/去重，最多 ~80-100 个唯一位置，不再被 `== 0` 门控关闭）。
5. 当 `best_pos == gpos` 且已缓存了该位置各符号 `sym_corr` 时，直接复用它们完成极性校验，避免重复调用 `sync_filter.dot`。

完成后构建 Release（UWB_SIMD=AVX512）并运行 `env -u LD_LIBRARY_PATH ctest --output-on-failure` 全量通过。
```

---

### Subagent 2-C：`Agent-CIR-Pipeline`

- **Worktree / 分支**：`.worktrees/opt-cir-pipeline` / `exp/opt-cir-pipeline`（基于 `opt-w1`）
- **独占文件**：
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_softchips.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_execution.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_rx_softchip_opt.h`
  - `gr-uwb/lib/uwb_demod_pipeline.cc`
  - `gr-uwb/lib/uwb_discrete_cir_estimator.cc`
  - `gr-uwb/lib/qa_uwb_demod_core.cc`
- **涵盖优化项**：`P0-2`、`TM-4`、`SFD-2`、`CIR-1`、`CIR-2`、`CIR-3`

#### 详细技术要求
1. **`P0-2` 修复 `retrack_tail_timing` 双倍重跑**：
   - 在 `retrack_tail_timing` 第 298 行重试时，恢复 `anchor.locked_roi_samples = profile.locked_roi_samples;`（若 `LockedBounded` 下 `seed_radius` 根本不生效，则当 `profile.sync_strategy == SyncStrategy::LockedBounded` 且 `anchor.locked_roi_samples >= profile.locked_roi_samples` 时不做相同参数重试）。
   - 在 `run_tail_retrack_and_cfo_refit` / `anchor_and_verify_final_sfd`：若 `scratch.initial_timing.sync_peaks_folded && sync_state.stage2_cfo.peaks_used >= 16`，直接跳过 `retrack_tail_timing`（置 `have_tail_peaks = false` 走重建网格并沿用 Stage 2 CFO）。
2. **`TM-4` 放宽 `try_reuse_initial_tail_timing`**：
   - 在 `try_reuse_initial_tail_timing` 中，不再硬要求 `initial_timing.detected_peaks == cfg.full_repetitions` 与 0 样点偏差。遍历 `k ∈ [cfg.tail_first, cfg.full_repetitions)`，在 `initial_timing.peak_samples` 中查找与 `expected_k` 距离 `<= 2` 样点的已有峰值直接拷贝，仅要求匹配到 `>= cfg.tail_repetitions - 4` 个且 metric 均 `>= anchor.radar_verification_threshold` 即可复用，免去重跑 40-SYNC `stage_timing`。
3. **`SFD-2` 免除无变化 `run_final_sfd`**：
   - 在 `anchor_and_verify_final_sfd` 中：若 `cfo_refit_action == 1 || cfo_refit_action == 2`（即未重新旋转 `scratch.derotated`），**直接跳过** `run_final_sfd()`（复用 `bootstrap` SFD 结果）；若重新旋转了，但 `std::abs(res.cfo.cfo_hz - sync_state.stage2_cfo.cfo_hz)` 在 8 个符号（`8 * 1016 / sample_rate`）累积相位差 `< 0.05 rad`，同样可跳过或仅在 `bootstrap_sfd ± 4` 极窄复核。
4. **`CIR-1`（核心架构收益）融合 CFO 去旋转与 SYNC 相干平均 + 共享末尾 32 SYNC**：
   - **Range 缩小**（`uwb_demod_execution.h`）：
     - `compute_cir_only_rotation_range`：当 `estimate_cir` 直读原始 `rx` 时，Sensing 模式只需旋转 `final_sfd_range`（~9k 样点，不再旋转 250k 前导！）；
     - `compute_post_sfd_prefix_bounds`：当 `skip_sync_fir == true`（`--fused on`）时，`req_lo` 不再下探到 `cir_lo`（前导第 10 个 SYNC），只需从 `sfd_range.lo`（或 `sfd_start - 64`）开始旋转！
   - **`estimate_cir` 直读未旋转 `rx` 并输出 `avg_tail`**：
     - 给 `estimate_cir` 增加可选参数（或重载）接收原始 `rx` 与 `cfo_hz`（若 `cfo_hz == 0` 保持原逻辑；若传入原始 `rx` 和 `cfo_hz`）：对每个 SYNC $k$ 计算复标量 $\alpha_k = e^{-j 2\pi f_{\text{cfo}} \cdot \text{lo}_k / f_s}$，用 AVX2/AVX-512 复标量乘加累加到 `scratch.corr`（以及末尾 32 个 SYNC 累加到 `scratch.cir_avg_tail`），循环结束后只对这 `1053` 个样点乘一次 $e^{-j 2\pi f_{\text{cfo}} m / f_s}$。
     - `stage_softchips_from_cir` 中 `skip_sync_fir` 路径的 `sync_phase_gain`：直接消费 `scratch.cir_avg_tail`（不再读 32×1052 样点重算一遍）。
   - **SIC 同理**（`uwb_discrete_cir_estimator.cc` L1132-L1168）：`build_cfo_rotation` 只需生成 `width`（~1035）长度的旋转表，循环内用每 repetition 一个标量 $\alpha_k$ 累加 `avg`，结束后乘一次旋转表。
   - **向后兼容**：保持原 `estimate_cir(scratch.derotated.data(), ...)` 签名不变（传 `cfo_hz = 0.0`），新增带 `cfo_hz` 的接口供 `uwb_demod_pipeline.cc` 调用，确保所有既有单阶段 QA 零破坏。
5. **`CIR-2` & `CIR-3`**：
   - `CIR-2`：`estimate_cir` 中 `values` 改栈数组 `std::array<std::complex<float>, 64>`；`scratch.corr` 尾部补 10 个零使 `correlate_cir_sparse` 无需走标量尾项；仅需 L2 归一化时可省去 `1/valid` 整段缩放与 `/ code_energy`。
   - `CIR-3`：当 `std::abs(period - round(period)) * available > 0.5`（大 SFO 如 `+27 ppm`）且直读 `rx` 时，按小数偏移 $f_k \in [-0.5, 0.5)$ 分 4 桶相干平均，最后对 4 桶做线性插值合并。
6. **单测**：在 `qa_uwb_demod_core.cc` 新增 `CIR-1`（直读 `rx` 融合去旋转 vs 全帧去旋转后 `estimate_cir`）与 `sync_phase_gain` 复用 `avg_tail` 的数值等价性测试（含非零 CFO 与 `+27 ppm` SFO）。

#### 下发给 OpenCode 的 Prompt
```text
你是 Wave 2 的 Agent-CIR-Pipeline，请在独立 worktree `.worktrees/opt-cir-pipeline`（分支 `exp/opt-cir-pipeline`，基于 `opt-w1`）工作。
只允许修改以下文件：
- gr-uwb/include/gnuradio/uwb/uwb_demod_softchips.h
- gr-uwb/include/gnuradio/uwb/uwb_demod_execution.h
- gr-uwb/include/gnuradio/uwb/uwb_rx_softchip_opt.h
- gr-uwb/lib/uwb_demod_pipeline.cc
- gr-uwb/lib/uwb_discrete_cir_estimator.cc
- gr-uwb/lib/qa_uwb_demod_core.cc

请阅读 docs/performance/Timing_SFD_CIR与匹配滤波优化分析_2026-11-19.md 中的 P0-2, TM-4, SFD-2, CIR-1, CIR-2, CIR-3，并完成以下改造：
1. P0-2：修复 `retrack_tail_timing` `LockedBounded` 下未恢复 `locked_roi_samples` 导致的双倍重跑；若 Stage 1 为 `sync_peaks_folded` 且 `stage2_cfo.peaks_used >= 16`，直接跳过 `retrack_tail_timing`。
2. TM-4：放宽 `try_reuse_initial_tail_timing`，允许在 Stage 1 `peak_samples` 中按 `±2` 样点容差匹配尾部符号并复用（减少不必要的二次 40-SYNC `stage_timing`）。
3. SFD-2：`cfo_refit_action == 1 || 2`（未重旋转）时直接跳过 `run_final_sfd()`。
4. CIR-1（核心）：
   - 保持原 `estimate_cir` 签名兼容（默认 `cfo_hz = 0`），新增接收未旋转 `rx` 与 `cfo_hz` 的融合路径：每 SYNC 乘标量 `exp(-j*w*lo_k)` 向量累加到 `scratch.corr` 和末尾 32 SYNC 的 `scratch.cir_avg_tail`，循环外对 1053 点乘一次 `exp(-j*w*m)`。
   - 更新 `uwb_demod_execution.h` 的 `compute_cir_only_rotation_range` 与 `compute_post_sfd_prefix_bounds`（`--stop-after-cir on` 和 `--fused on` 不再把前导 250k 样点纳入 CFO 去旋转范围）。
   - 更新 `sync_phase_gain` / `stage_softchips_from_cir` 在 `skip_sync_fir` 时直接复用 `scratch.cir_avg_tail`。
   - 同步用同样因式分解优化 `uwb_discrete_cir_estimator.cc`（L1132-L1168）的 Ipatov CFO 旋转。
5. CIR-2 & CIR-3：`values` 改栈数组、去可约掉的 `1/valid`；大 SFO（如 +27 ppm）下支持 4 桶分数相位相干平均。
6. 在 `qa_uwb_demod_core.cc` 补齐 `CIR-1` 与 `CIR-3` 的数值等价 QA。

完成后构建 Release（UWB_SIMD=AVX512）并运行 `env -u LD_LIBRARY_PATH ctest --output-on-failure` 全量通过。
```

---

## 4. Wave 3：`MF-4` 脉冲因子分解与 ABBA 全量验收（基于 `opt-w2`）

Wave 2 合并为 `opt-w2` 并通过全量 CTest 及既有 6×40 固定窗基线核对后，由 `Agent-MF4-Eval` 执行收尾：

### Subagent 3：`Agent-MF4-Eval`
1. **实现 `MF-4`（`rx_timing_sparse_shaped` 实验开关）**：
   - 从 `template_wf`（或脉形成形参数）中提取主瓣短实脉冲核 $p$（5–9 taps），在 `uwb_rx_matched_filter.h` 提供先稀疏 `64` 点加减（$c = r \star \text{code}$）、再短实 FIR（$c \star p$）的组合通道。
   - 在 `-12 dB` / `-14 dB` / `-16 dB` 完整帧夹具及多径夹具上对比：`稠密 1016-tap` vs `纯稀疏` vs `因子分解 (r ⋆ code) ⋆ p` 的检测率、2 样点群延迟对齐情况与 `demod_us` 耗时。
   - **边界**：按 `AGENTS.md` 约定，若要将全局默认 Profile 切离稠密模板，需标注“待 MATLAB `detectRepeatedPreamble.m` 交叉验证”；本轮可通过 `uwb_dw3000_live` 参数开关提供验证证据。
2. **完整 ABBA 性能基准与回归门禁**：
   - 串行、绑核、同环境 ABBA 对比（`Base b654733` vs `opt-w2` / `opt-w3`），覆盖：
     - 高 SNR 实采 1 s 回放（`Comm Locked`、`Sensing --stop-after-cir on`、`Acquire`）；
     - `-12 dB` locked & acquire、`-14 dB` locked（`--stop-after-sfd on --lock-fcs off`）、纯噪声 `0` 虚警；
     - 6×40 固定窗（含 `+40 kHz` CFO、`+27 ppm` SFO）逐帧逐字节核对。
   - 分别在 `UWB_SIMD=AVX512` 和 `UWB_SIMD=AVX2` 两套独立构建下运行全量 CTest。
   - 整理最终实验报告写入 `docs/performance/`，并更新 `开发状态.md`。

---

## 5. Integrator 合并与验收检查表

| 检查阶段 | 命令 / 检查项 | 门禁要求 |
|---|---|---|
| **Wave 1 合并后** | `cmake -DUWB_SIMD=AVX512` & `AVX2` + `env -u LD_LIBRARY_PATH ctest` | 100% 通过（0 失败），`qa_uwb_demod_sic_avx512` 新增用例全过 |
| **Wave 2 合并后** | 全量 CTest + `tools/rx/low_snr_eval/par_only.py` 或 `fixed_scenarios.py` 6×40 | 6×40 逐帧 FCS 与 `b654733` 无退化，纯噪声 0 虚警 |
| **Wave 3 收尾后** | 串行 ABBA 基准 + `git diff` 核查 `uwb_radar_*` 未被修改 | Radar 文件 0 变更；所有数据/CPU/ISA/哈希/统计口径随报告记录 |
