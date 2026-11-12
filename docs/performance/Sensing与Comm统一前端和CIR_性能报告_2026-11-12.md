# Sensing 与 Comm 统一前端和 CIR：性能报告

对应任务书 `docs/performance/OpenCode_Sensing与Comm统一前端和CIR_开发方案_2026-11-12.md`
§8「性能验收」，接口契约
`docs/performance/接口契约_统一SYNC_CIR核与公共前导前端_2026-11-12.md`。
原始数据：`docs/performance/data_sensing_shared_cir_20261112/`（纯文本，84 KB，
2026-11-12 23:28 重新生成）。
基准源码：`tools/sic/benchmark_shared_cir.cc`、`tools/sic/run_shared_cir_bench.py`。

> **数据版本说明**：本报告的所有数字来自**重新生成**的
> `data_sensing_shared_cir_20261112/`（生产侧新增
> `timing_seed_refine_radius=8` / `timing_max_backtrack_symbols=0` 之后的一次完整测量）。
> 上一版报告（candidate total 688.01 / 1207.55 µs）已作废，请勿引用。

**一句话结论**：116-tap 公共 CIR 内核比旧标量核快 **13.4×（64 SYNC）/ 4.7×（256 SYNC）**，
远超 ≥20% 门槛；sensing 接收阶段在 **64/32 快 24.3%**（通过 ≥10% 门槛），
但在 **256/127 慢 30.8%（未通过）**。256 SYNC 的失败原因不是内核，而是公共前端
必须跟踪整条 256-SYNC 训练序列（387.93 µs，旧路径只做 12.45 µs 的局部 refine），
这一项就比内核节省的 167 µs 多出 2.2 倍。**本轮不宣称 256 SYNC 场景达标。**

---

## 1. 判定表（先给结论）

| 门槛（方案 §8） | 配置 | 实测 | 判定 |
|---|---|---|---|
| 116-tap 公共 CIR 比旧标量核快 ≥20% | 64 SYNC / 32 B | 174.89 → 13.03 µs，ratio **0.0745**（−92.6%，13.4×） | **通过** |
| 116-tap 公共 CIR 比旧标量核快 ≥20% | 256 SYNC / 127 B | 217.82 → 46.67 µs，ratio **0.2142**（−78.6%，4.7×） | **通过** |
| sensing 接收阶段整体快 ≥10% | 64 SYNC / 32 B | 855.77 → 648.07 µs，ratio **0.7573**（−24.3%） | **通过** |
| sensing 接收阶段整体快 ≥10% | 256 SYNC / 127 B | 899.25 → 1176.61 µs，ratio **1.3084**（**+30.8%**） | **未通过** |
| comm 不得稳定退化 >5% | 64 SYNC / 32 B | 本 build 内无 control 可对照（见 §6） | **未验证** |
| SIC-only 不得稳定退化 >5% | — | 本基准不覆盖 SIC 路径 | **未验证** |

三个 session 独立复现，方向一致（§4.1 给出逐 session 的 ratio 区间）。

---

## 2. 测量合同

### 2.1 源码与二进制

| 项 | 值 |
|---|---|
| git SHA | `068d21e03b55f8c4f2e665728f31e5ab3df8daef`（分支 `feature/sensing-shared-cir`） |
| 工作树状态 | dirty：三个 session 均为 21 项未提交改动（生产实现 + 本基准新增文件）；逐 session 记录在 `session*.csv` 的 `dirty_entries` |
| 二进制 | `gr-uwb/build-sensing-shared-cir/bench/benchmark_shared_cir` |
| 依赖 | 仅 libvolk + pthread；不链接 `gnuradio-uwb`，不需要 GNU Radio runtime |
| 编译 | 见 §7 |

### 2.2 CPU 与亲和性

| 项 | 值 |
|---|---|
| `nproc` | 20 online / 20 configured |
| 实际亲和性 | `taskset -c 2`，`sched_getcpu()` 8 次采样全部为 `2` |
| 负载 | 运行前 `loadavg` 0.52 / 0.61 / 1.10 |
| 并发 | 全程独占 `/tmp/uwb-sic-benchmark.lock`（`flock LOCK_EX`），三 session 串行 |

### 2.3 输入 fixture（真实调制波形，非合成相关峰）

用 `uwb_hrp_mod_core.h` 在 998.4 MS/s 上真实调制：code 9、SFD `4z2`、
Legacy 48-tap 脉冲、`peak_amplitude=0.8`、PSDU = 随机数据 + IEEE FCS。
窗口 = `pre_guard` 前导零 + 整包 + 4096 尾零，几何与
`gr-uwb/lib/qa_uwb_radar_e2e.cc` 一致：

```
predicted_sfd = pre_guard + sync_reps * 1016
cir_axis      = predicted_sfd - sync_reps * 1016 = pre_guard = 1997
```

| 配置 | sync_reps | PSDU B | 包长 | 窗口长 | predicted_sfd | cir_axis | window FNV-1a | PSDU FNV-1a |
|---|---|---|---|---|---|---|---|---|
| `sync64_psdu32` | 64 | 32 | 133,568 | 139,661 | 67,021 | 1,997 | `cd056a2e6f7b546d` | `35cae5dc461aa14e` |
| `sync256_psdu127` | 256 | 127 | 444,352 | 450,445 | 262,093 | 1,997 | `9b86d67bae9afecb` | `3858a5d239cdba55` |

fixture 与上一版逐字节相同（同一 FNV-1a），因此两版之间的差异只来自生产代码改动
与运行间漂移。

`testdata/uwb_radar/sync_template_998p4.cf32` 与调制器输出的前 1016 样点逐点对照，
相对 L2 = **0.000**（<1e-3 显示精度），说明 fixture 的成形脉冲没有漂移。

公共参数：`sample_rate=998.4e6`、`sps=1016`、`cir_pre=16`、`cir_post=100`、
`taps=116`、`cir_skip_initial=10`、`sfd_mode=4z2`、`code_index=9`。
阈值：`sfd_threshold=0.3`、`sync_refine_threshold=0.3`、`sfd_search_margin=64`、
`sync_refine_margin=8`、前端 `timing_search_margin=64`、`timing_coarse_stride=1`、
`timing_track_radius=8`。**零目标 CFO、单径、无噪声**（方案 §8 要求用两实现都正确的
零 CFO 格衡量提速）。

`prepare_radar_cir_core()` 另外从 `sync_refine_margin` 派生
`timing_seed_refine_radius = 8` 与 `timing_max_backtrack_symbols = 0`（见 §4.5）。
这两个字段不是 `RadarCirConfig` 字段，因此基准的 CSV 头不打印它们；本报告按源码记录。

### 2.4 两臂定义（同一进程、同一二进制、ABBA 交错）

- **control（旧算法，仍在树内）**：`radar::search_sfd` →
  `radar::refine_sync_origin` → `radar::estimate_radar_cir`
  （116 tap，`pre=16/post=100`，double 累加，稠密 1016 项码扫描，**不做任何 CFO 补偿**），
  CIR 轴取公开发送时序原点 `nominal_preamble_start(predicted_sfd, reps, sps)`。
- **candidate（新生产路径）**：`radar::radar_cir_one` 且
  `compensate_target_cfo = true`：公共前导前端（检测 → CFO 估计 → bootstrap SFD →
  tail 重跟踪 + CFO 复估 → final SFD → 覆盖 CIR 并集的最终 CFO 旋转）+ 公共
  `sync_cir::estimate_sync_cir` 内核。

两臂读**同一个 raw 窗口**，唯一差异是算法。因此不存在跨 build 噪声。

### 2.5 协议

- 独占 `/tmp/uwb-sic-benchmark.lock`（`flock LOCK_EX`）覆盖整个进程生命周期；
- 每臂预热 **40** 次（≥32）；
- 每 session 每臂 **520** 个有效观测（≥512）：5 组 ABBA × 2 槽 × 52 次；
- **3 个独立 session**（3 次独立进程调用），逐 session 汇总 + 合并汇总；
- 每个观测都校验 status / tap_count / peak_tap，任何一次失败立即 abort 且**不输出计时**。

### 2.6 正确性守卫（计时之前执行）

| 配置 | 两臂均成功 | peak_tap | valid_repetitions | raw CIR 相对 L2（门槛 2e-5） | final CFO | cfo_epoch |
|---|---|---|---|---|---|---|
| `sync64_psdu32` | 是 | 18 == 18 | 54 == 54 | **8.827e-08** | 0.000 Hz | 2 |
| `sync256_psdu127` | 是 | 18 == 18 | 246 == 246 | **7.342e-08** | 0.000 Hz | 2 |

`cfo_epoch == 2` 证明 candidate 消费的确实是 final-CFO 补偿后的缓冲；
`cir_samples_per_symbol = 1016.000000` 证明前端测得的周期与整数网格一致，
没有引入分数重采样。三者任一不满足则进程 abort，不产生任何计时行。
守卫数值与上一版**完全相同**，说明 §4.5 的窄种子搜索没有改变这两个 fixture 上的 CIR。

---

## 3. 完整矩阵

三 session 合并。单位 µs。`mean/P50/P95` 为逐 session 值的平均，方括号为逐 session
值的 min..max；`min/max` 为任一 session 中出现的单次观测极值。

### 3.1 `sync64_psdu32`（64 SYNC / 32 B PSDU）

| arm | metric | n | mean | P50 | P95 | min | max | ratio |
|---|---|---|---|---|---|---|---|---|
| control | total | 520 | 855.77 [854.18..857.53] | 831.04 | 985.74 | 776.97 | 3015.35 | |
| control | sfd | 520 | 667.85 [663.87..670.47] | 648.69 | 768.64 | 605.83 | 2834.03 | |
| control | refine | 520 | 13.04 [12.18..14.49] | 11.68 | 15.35 | 10.98 | 1137.44 | |
| control | cir_scalar_kernel | 520 | 174.89 [173.96..175.83] | 168.24 | 211.24 | 157.32 | 634.46 | |
| candidate | total | 520 | 648.07 [641.18..652.31] | 628.12 | 761.35 | 590.90 | 2656.86 | **0.7573** |
| candidate | frontend_timing | 520 | 95.82 [93.85..97.50] | 91.67 | 112.00 | 85.00 | 767.00 | |
| candidate | frontend_cfo | 520 | 15.31 [15.14..15.45] | 14.67 | 19.00 | 14.00 | 82.00 | |
| candidate | frontend_sfd | 520 | 521.61 [517.06..524.09] | 505.67 | 622.33 | 477.00 | 2346.00 | |
| candidate | coverage | 520 | 0.00 [0.00..0.01] | 0.00 | 0.00 | 0.00 | 6.00 | |
| candidate | cir_us_reported | 520 | 12.45 [12.28..12.57] | 12.00 | 17.67 | 11.00 | 72.00 | |
| candidate | cir_shared_kernel | 520 | 13.03 [12.80..13.24] | 12.32 | 15.78 | 11.74 | 177.73 | **0.0745** |

`cir_scalar_kernel` = `estimate_radar_cir` 单独计时；`cir_shared_kernel` =
`estimate_sync_cir` 单独计时（生产消费的同一 derotated 缓冲，高分辨率时钟）。
`cir_us_reported` 是 `RadarCirResult::cir_us`（1 µs 量化），仅用于与生产字段对齐。

### 3.2 `sync256_psdu127`（256 SYNC / 127 B PSDU）

| arm | metric | n | mean | P50 | P95 | min | max | ratio |
|---|---|---|---|---|---|---|---|---|
| control | total | 520 | 899.25 [891.77..905.72] | 873.24 | 1068.83 | 810.48 | 4318.85 | |
| control | sfd | 520 | 668.97 [664.77..675.25] | 647.97 | 789.60 | 605.57 | 4042.41 | |
| control | refine | 520 | 12.45 [12.28..12.66] | 11.78 | 15.67 | 11.00 | 147.12 | |
| control | cir_scalar_kernel | 520 | 217.82 [214.72..220.94] | 208.99 | 265.69 | 192.46 | 696.93 | |
| candidate | total | 520 | 1176.61 [1168.43..1187.97] | 1134.37 | 1393.66 | 1044.77 | 5408.27 | **1.3084** |
| candidate | frontend_timing | 520 | 387.93 [380.84..396.30] | 370.67 | 466.33 | 336.00 | 4525.00 | |
| candidate | frontend_cfo | 520 | 17.03 [16.73..17.29] | 16.00 | 21.67 | 15.00 | 114.00 | |
| candidate | frontend_sfd | 520 | 716.50 [710.95..720.01] | 689.00 | 860.33 | 647.00 | 4151.00 | |
| candidate | coverage | 520 | 0.17 [0.11..0.22] | 0.00 | 1.00 | 0.00 | 45.00 | |
| candidate | cir_us_reported | 520 | 51.24 [50.34..52.13] | 45.67 | 78.00 | 41.00 | 507.00 | |
| candidate | cir_shared_kernel | 520 | 46.67 [45.97..47.67] | 42.56 | 65.77 | 38.37 | 268.93 | **0.2142** |

### 3.3 `comm64_psdu32`（`demodulate_one`，64 SYNC / 32 B）

| arm | metric | n | mean | P50 | P95 | min | max | ratio |
|---|---|---|---|---|---|---|---|---|
| comm_legacy_profile | total | 520 | 1311.03 [1292.09..1342.62] | 1272.78 | 1476.25 | 1204.79 | 5853.50 | |
| comm_legacy_profile | stage_cir | 520 | 551.16 [543.17..562.23] | 535.00 | 640.33 | 505.00 | 3676.00 | |
| comm_fast_profile | total | 520 | 344.24 [342.12..348.07] | 327.99 | 429.05 | 309.93 | 1828.74 | 0.2626 |
| comm_fast_profile | stage_cir | 520 | 51.64 [50.89..52.47] | 48.67 | 65.00 | 47.00 | 350.00 | 0.0937 |

两臂都通过 FCS 且 PSDU 逐字节相等（守卫阶段断言，否则 abort）。
**这不是 refactor 的 control/candidate 对照**，见 §6。

### 3.4 关键比值（candidate / control）

| config | metric | control mean | candidate mean | ratio | speedup |
|---|---|---|---|---|---|
| sync64_psdu32 | total | 855.77 | 648.07 | 0.7573 | 1.320x |
| sync64_psdu32 | cir_scalar_kernel → cir_shared_kernel | 174.89 | 13.03 | 0.0745 | 13.425x |
| sync256_psdu127 | total | 899.25 | 1176.61 | 1.3084 | 0.764x |
| sync256_psdu127 | cir_scalar_kernel → cir_shared_kernel | 217.82 | 46.67 | 0.2142 | 4.667x |
| comm64_psdu32 | total（legacy profile → fast profile） | 1311.03 | 344.24 | 0.2626 | 3.808x |
| comm64_psdu32 | stage_cir（legacy profile → fast profile） | 551.16 | 51.64 | 0.0937 | 10.672x |

---

## 4. 时间去向分解

### 4.1 逐 session 复现性

| session | 64 total ratio | 64 kernel ratio | 256 total ratio | 256 kernel ratio | 64 fe_timing | 256 fe_timing |
|---|---|---|---|---|---|---|
| 1 | 0.7637 | 0.0728 | 1.3102 | 0.2141 | 97.50 | 386.64 |
| 2 | 0.7588 | 0.0746 | 1.3196 | 0.2157 | 96.11 | 396.30 |
| 3 | 0.7494 | 0.0761 | 1.2955 | 0.2129 | 93.85 | 380.84 |

方向与量级完全一致；没有挑选单 session 最快值。

### 4.2 逐项增量（3-session 均值，µs）

**64 SYNC / 32 B：总 −207.70 µs**

| 项 | control | candidate | 增量 |
|---|---|---|---|
| CIR 内核 | 174.89（标量） | 12.45（公共，生产字段） | **−162.44** |
| 前导 timing（SYNC 跟踪 + 周期拟合） | 13.04（只 refine 1 个 SYNC） | 95.82（跟踪全部 64 个） | **+82.78** |
| CFO 估计 + bootstrap 旋转 | 0（旧路径完全没有） | 15.31 | **+15.31** |
| SFD 锚定（含 tail 重跟踪 + CFO 复估 + 最终旋转） | 667.85（1 次全区间搜索） | 521.61（2 次窄窗搜索 + tail + 最终旋转） | **−146.24** |
| 覆盖规划 | — | 0.00 | 0.00 |
| `radar_cir_one` 外壳（校验 + 116 tap ×2 拷贝） | — | 2.88 | +2.88 |
| **合计** | 855.77 | 648.07 | **−207.71** |

**256 SYNC / 127 B：总 +277.36 µs**

| 项 | control | candidate | 增量 |
|---|---|---|---|
| CIR 内核 | 217.82（标量） | 51.24（公共，生产字段） | **−166.58** |
| 前导 timing | 12.45 | 387.93 | **+375.48** |
| CFO 估计 + bootstrap 旋转 | 0 | 17.03 | **+17.03** |
| SFD 锚定（含 tail + 最终旋转） | 668.97 | 716.50 | **+47.53** |
| 覆盖规划 | — | 0.17 | +0.17 |
| 外壳 | — | 3.74 | +3.74 |
| **合计** | 899.25 | 1176.61 | **+277.37** |

分解与总量逐 µs 闭合（差值 <0.02 µs，来自子计时的 1 µs 量化）。

### 4.3 新增的 CFO 补偿到底花了多少

`RadarCirResult::frontend_cfo_us` 只包含 CFO **拟合** + bootstrap 旋转（SFD 搜索区间
≈16 k 样点），**不包含**覆盖整个 CIR 并集的最终旋转 —— 后者在前端的
「SFD 锚定 + CFO 复估」段内，因此计在 `frontend_sfd_us` 里。为把这一项拆出来，
基准在计时矩阵之外对每个 fixture 额外跑 8 次 `acquire_preamble`，取父计时最小的
那一次（warm 快照），打印为 `# frontend_diag=...`。下表取三个 session 的区间：

| 项 | 64 SYNC | 256 SYNC |
|---|---|---|
| 最终 CFO 旋转覆盖区间 `[lo, hi)` | `[12013, 79405)` = 67,392 样点 | `[12013, 274477)` = 262,464 样点 |
| CFO 并集（kernel 实际读取） | 54,979 样点 | 250,051 样点 |
| 最终旋转耗时（`sfd_cfo_refit_us`） | **58–60 µs** | **227–228 µs** |
| 两次旋转合计（`cfo_rotate_us`） | 72–74 µs / 83,776 样点 | 240–241 µs / 278,848 样点 |
| 旋转吞吐 | ≈1.15 Gsample/s | ≈1.16 Gsample/s |
| CFO 拟合（`cfo_fit_us`） | 0 µs | 2 µs |
| 占 candidate 总时间 | 72.7 / 648.1 = **11.2%** | 240.3 / 1176.6 = **20.4%** |
| 对比内核节省 | 162 µs | 167 µs |

**结论**（把最终旋转从 `frontend_sfd_us` 里剥出来之后）：

- **64 SYNC**：内核省 **162.4 µs**；新增的 CFO 旋转花 **72.7 µs**（最终旋转 58.7 µs
  落在 SFD 段、bootstrap 旋转 14.0 µs 落在 CFO 段），两项相抵净赚 **89.8 µs**。
  扣除最终旋转后，共享窄窗 SFD 搜索本身省 **204.9 µs**；但整条 64-SYNC 跟踪多花
  **82.8 µs**，拟合/覆盖/外壳合计 **+4.2 µs**，最终 **−207.7 µs**。
- **256 SYNC**：内核省 **166.6 µs**，CFO 旋转花 **240.3 µs**（最终 227.3 + bootstrap
  13.0）→ 单这一对就是净亏 **73.7 µs**；扣除最终旋转后共享 SFD 搜索仍省 **179.8 µs**，
  但整条 256-SYNC 跟踪多花 **375.5 µs**，合计 **+277.4 µs**。

**所以 256 SYNC 的失败有两个独立原因，且都不是内核**：
1. **主因**：公共前端必须跟踪整条训练序列才能测周期和做 CFO 拟合。旧
   `refine_sync_origin` 只对 1 个 SYNC 做 ±8 refine（12.45 µs）；公共 `stage_timing`
   对 256 个 SYNC 各做 ±8 refine（387.93 µs）。每符号约 1.5 µs，与符号数近似线性。
2. **次因**：最终 CFO 旋转的成本正比于 CIR 并集长度，即正比于
   `(reps - skip) × 1016`。256 SYNC 时它（227 µs）已经超过整个内核节省（167 µs）。

### 4.4 为什么 64 SYNC 的 SFD 段反而更快

旧 `radar::search_sfd` 在 ±64 的 margin 上**逐整数起点**打分（129 次 × 8128 tap 相关，
实测 667.85 µs）。公共 `stage_sfd` 在跟踪到的预测位置做 ±64 窄窗，先 stride-8 粗扫
再 ±7 全速细化（约 32 次相关），命中即停；未命中才回退到 ±1..±4 符号。因此公共路径
**单次** SFD 搜索约 **172 µs**（诊断拆分：bootstrap 169–173 µs / final 169–175 µs），
比旧路径便宜约 **3.9×**。即使它跑了两遍、再加 tail 重跟踪（84–87 µs）和最终 CFO
旋转（58–60 µs），64 SYNC 下 SFD 段仍然净省 146 µs。256 SYNC 时 tail 重跟踪和
最终旋转变贵，这一项转为 +48 µs。

这是两个真实生产实现的算法差异（穷举 vs 预测窄窗），不是基准构造出来的差异。

### 4.5 新增：`timing_seed_refine_radius=8` / `timing_max_backtrack_symbols=0`

**改动**（`gr-uwb/include/gnuradio/uwb/uwb_radar_cir_core.h::prepare_radar_cir_core()`，
构建共享前端 profile 处）：

```cpp
p.timing_seed_refine_radius = (cfg.sync_refine_margin > 0)
    ? static_cast<size_t>(cfg.sync_refine_margin) : size_t(0);   // = 8
p.timing_max_backtrack_symbols = 0;
```

**改动前**：这两个字段保持 `Qm35825Profile` 的默认值（`timing_seed_refine_radius = 0`、
`timing_max_backtrack_symbols = 40`）。于是 `stage_timing` 走的是「legacy 首次种子
refine」路径：在种子周围做半径 `max(256, coarse_stride) = 256` 样点的全速搜索，
再叠加整 SYNC 网格探测，并允许最多回退 40 个符号。对 sensing 来说这是**捕获**
语义，而调用方其实已经给了可信的预测 SYNC 原点（公开粗 ROI）。

**改动后**：种子搜索变成局部 refine（半径 8），且不再跨符号回退 —— 与旧 radar 路径
「在 `sync_refine_margin` 内 refine、从不跨符号回退」的语义一致。窄搜索失败时
`stage_timing` 内部仍会回退到宽搜索，恢复能力不变；comm 路径用自己的默认值，不受影响。

**实测效果**（同一 fixture、同一二进制路径、同一协议）：

| 配置 | `frontend_timing` 改动前 | 改动后 | 变化 |
|---|---|---|---|
| 64 SYNC / 32 B | 128.56 µs | **95.82 µs** | −32.74 µs（−25.5%） |
| 256 SYNC / 127 B | 414.43 µs | **387.93 µs** | −26.50 µs（−6.4%） |

**它没有关闭 256/127 的缺口**：

- 256 SYNC 的 `frontend_timing` 仍然是 **387.93 µs**，而旧 radar 路径的
  `refine_sync_origin` 只要 **12.45 µs** —— 差距 **+375.48 µs**，是 256/127 总增量
  （+277.4 µs）的 **135%**，也就是说这一项比最终总增量还大，只是被内核的
  −166.6 µs 和共享 SFD 搜索的 −179.8 µs 部分抵消掉了。
- 256/127 的 total ratio 从上一版的 1.3063 变为 1.3084，**落在运行间漂移内**：
  control 臂代码完全没变，其 total 在两次运行间仍从 924.42 漂到 899.25 µs（−2.7%），
  64 SYNC 的 control 从 861.14 漂到 855.77 µs（−0.6%）。因此不能把这 0.2% 的
  ratio 变化解读为改善或恶化。
- 64/32 的 ratio 改善（0.7989 → 0.7573）主要来自 candidate 端的 **−39.9 µs**
  （timing −32.7、SFD −6.7、其余 −0.5），叠加 control 臂 **−5.4 µs** 的运行间漂移；
  漂移量只有 candidate 改善量的约 1/7，因此这个改善可以归因于本次改动。

**正确性**：§2.6 的守卫数值与改动前完全一致（peak_tap 18、raw CIR 相对 L2
8.827e-08 / 7.342e-08、`cfo_epoch == 2`、`cir_samples_per_symbol = 1016.000000`），
说明窄种子搜索在这两个 fixture 上没有改变 CIR。窄搜索失败时的宽搜索回退路径
本轮**未单独构造用例**（见 §9）。

**结论**：这是一次正确的、有收益的语义对齐（64/32 −25.5% 的前端 timing），
但它只是把 256 SYNC 的线性跟踪成本削掉 6.4%，**不改变 256/127 未达标的结论**。

---

## 5. 与方案 §8 逐条对照

1. **「116-tap 公共 CIR 估计比旧标量核快 ≥20%」** → **通过**。
   64 SYNC −92.6%，256 SYNC −78.6%。收益来源：(a) 稀疏三元码相关只遍历 127 个非零码
   位置 × 116 tap，而不是 1016 × 116 的稠密扫描；(b) float 累加而不是 double；
   (c) 相干平均与相关全部走 VOLK。两臂都对 tap 数线性，但公共核的常数项小约一个
   数量级。**256 SYNC 的 ratio 只有 4.7×（而 64 SYNC 是 13.4×）**，因为两臂都包含
   正比于重复数的相干平均项（64 SYNC 61k 样点、256 SYNC 278k 样点），这一项在
   256 SYNC 时占了两臂各自的大部分，把比值压向 1。
2. **「sensing 接收阶段整体快 ≥10%」** → **64/32 通过（−24.3%），256/127 未通过（+30.8%）**。
   未通过的原因见 §4.3 / §4.5，是本轮必须如实保留的结论。
3. **「comm 和 SIC-only 整体不得稳定退化 >5%」** → **本基准无法给出该判定**，见 §6、§9。

---

## 6. comm 臂：为什么没有 control/candidate 对照

`demodulate_one()` 在本 build 中**已经**改调 `acquire_preamble()`，旧的 comm 前导实现
被就地替换而不是保留为第二份。因此同一进程、同一 build 内不存在「旧 comm vs 新 comm」
两臂 —— 强行做只会得到 ratio = 1.000 的自我比较。跨 build 对照（`bd311cd` 的旧 build）
会引入 §2.4 明确要避免的跨 build 噪声，且需要另建目录与 fixture 复现，本轮未做。

本轮实际测到并如实报告的（§3.3）：

- `comm_fast_profile`（生产 fast 档：`rx_fast_sfd` / `rx_fast_cfo_rotation` /
  `rx_seed_refine_tail` / `rx_softchip_on_demand` / `rx_fused_payload_cmf` /
  `rx_fused_phr_cmf` 全开）：total **344.24 µs**，`stage_cir_us` **51.64 µs**；
- `comm_legacy_profile`（上述标志全关，即全流 soft-chip 旧路径）：total
  **1311.03 µs**，`stage_cir_us` **551.16 µs**。

这是**同一 build 内的 profile 对照**，不是 refactor 的 control/candidate 对照。
能得到的唯一结论是：comm 的 `stage_cir_us` 在本 build 中走的是公共内核，38 tap 时
成本 52 µs（fast 档）；legacy 全流档的 551 µs 里，差额约 500 µs 是整帧 soft-chip 扩展
（fast 档把 soft-chip 前缀截到 PHR 结束），不是内核本身 —— 这是从两档差额推断的，
没有单独拆出内核字段。**「comm 不退化 >5%」这一条本轮未验证。**

一个间接证据（非门槛判定）：公共内核在 116 tap 时比旧标量核快 4.7–13.4×，
38 tap 时内核成本只会更低；comm 前端的阶段顺序、阈值、回退路径与 refactor 前逐行
一致（契约 §2.1 要求），因此不存在可解释的 >5% 退化机制 —— 但这是推理，不是测量。

---

## 7. 复现步骤

```bash
cd /home/junqima/workspace/uwb-gnuradio/.worktrees/sensing-shared-cir

# 1) 编译（header-only + volk，不需要 GNU Radio runtime，不需要 cmake）
mkdir -p gr-uwb/build-sensing-shared-cir/bench
g++ -O3 -std=c++17 -march=native -I gr-uwb/include \
    tools/sic/benchmark_shared_cir.cc \
    -o gr-uwb/build-sensing-shared-cir/bench/benchmark_shared_cir \
    -lvolk -lpthread

# 2) 三 session 串行矩阵（runner 自己不做任何构建）
taskset -c 2 python3 tools/sic/run_shared_cir_bench.py \
    --binary gr-uwb/build-sensing-shared-cir/bench/benchmark_shared_cir \
    --out docs/performance/data_sensing_shared_cir_20261112 \
    --sessions 3 --groups 5 --iters 52 --warmup 40

# 3) 只重算汇总（不重跑测量）
taskset -c 2 python3 tools/sic/run_shared_cir_bench.py \
    --binary gr-uwb/build-sensing-shared-cir/bench/benchmark_shared_cir \
    --out docs/performance/data_sensing_shared_cir_20261112 \
    --sessions 3 --groups 5 --iters 52 --warmup 40 --reuse

# 4) 单 session 快速自检（会 abort 如果任一臂失败或数值守卫不过）
./gr-uwb/build-sensing-shared-cir/bench/benchmark_shared_cir \
    --groups 1 --iters 1 --warmup 32
```

单 session 约 12 s，三 session 约 40 s。

输出：
- `session{1,2,3}.csv` — 每 session 的 `#` 溯源头 + 26 行 summary（含 fixture hash、
  git SHA、亲和性、配置、阈值、`frontend_diag` 分解）；
- `session{1,2,3}.log` — 守卫行 + 人读汇总；
- `summary.md` — 合并 markdown 表；
- `combined.json` — 机器可读的合并结果与逐 session 溯源。

---

## 8. 测试状态

全量串行 `ctest`：**83 项，79 通过，3 失败，1 跳过**。

**3 项失败均为既有问题，已在未改动的基线上复现**（与本轮改动无关）：

| 失败项 | 说明 |
|---|---|
| `uwb_qa_uwb_pdu_rational_resampler_65_32_workers.cc` | 既有失败，基线复现 |
| `uwb_qa_uwb_fullpacket_cfo_diag.cc` | 既有失败，基线复现 |
| `uwb_qa_uwb_preamble_sic_core.cc` | 既有失败，基线复现 |

**本轮新增/更新的针对性 QA 全部通过**：

| 用例 | 状态 |
|---|---|
| `uwb_qa_uwb_sync_cir_core.cc`（公共 CIR 核：codes 9–12、38/116 tap、64/256 SYNC、skip10、整数/分数起点、非整数 P、clipped 边界、0 有效重复、容量不足、非有限输入、float64 参考） | 通过 |
| `uwb_qa_uwb_preamble_frontend.cc`（公共前导前端：final CFO 切换、区间补齐、缺 halo、过期 epoch、无 double rotation、失败关闭） | 通过 |
| `uwb_qa_uwb_radar_shared_cir.cc`（sensing 接公共前端 + 公共核：三个原点、raw/norm、失败路径） | 通过 |
| `uwb_qa_uwb_pdu_roi_cir_65_32.cc`（改用真实帧 fixture 后） | 通过 |

跳过项为 `uwb_qa_install_consumer`（按设计跳过）。

---

## 9. 未验证清单（明确不宣称）

1. **256 SYNC 的整体提速未达成**（+30.8%）。本轮不调负载、不降阈值、不改门控去凑门槛。
2. **方案 §7.7 的完整端到端功能矩阵未跑**：64/32 与 256/127 × work/native 两网格 ×
   单径与分数多径 × 两个目标码方向（9 受 10 干扰、10 受 9 干扰）× 目标/干扰 CFO
   独立取 0/±50 kHz × ISR 0/10/20。目前只有**零 CFO 的提速格**（本报告）加上
   QA 级别的 CFO / 距离轴 / 幅度相位检查（§8）。**非零 CFO 的感知质量、弱目标保护、
   PSLR/ISLR、主瓣宽度均未测量。**
3. **非零目标 CFO 的格未做性能测量**。方案 §8 明确「非零 CFO 的旧实现不是质量等效
   基线」，因此提速只在零 CFO 格衡量。
4. **comm 的 refactor control/candidate 对照未做**（§6），因此「comm 不退化 >5%」未验证。
5. **SIC-only（`PreamblePurpose::SyncSfdOnly`）端到端成本未测**。它走同一
   `acquire_preamble`，但本基准完全不含 SIC 路径，因此「SIC-only 不退化 >5%」未验证；
   需要 `benchmark_chain_workers` / 生产 pipeline 的 `sensing`/`comm` 任务矩阵。
6. **窄种子搜索失败后的宽搜索回退未单独构造用例**：`timing_seed_refine_radius=8`
   的回退路径由 `stage_timing` 内部保证，本轮只验证了它在两个正常 fixture 上不改变
   CIR，没有构造「种子偏移到窄窗之外」的负样本。
7. **未做分数多径 / 时延格**：fixture 是单径、零时延、无噪声。物理距离轴保持
   （`cir_axis = pre_guard`，peak_tap = 18 = `cir_pre + 2` 脉冲成形偏移）只在
   这一个几何上验证过。
8. **未做 65/48 与 65/32 native 网格**：全部在 998.4 MS/s work grid 上，不含重采样器。
9. **未做 GNU Radio scheduler / PDU / PMT / 队列 / paced 到达**：本基准是纯函数级
   离线测量，不含任何调度、拷贝、序列化开销。
10. **未做多 worker（N>1）**：全部单线程。
11. **未上板、未做 MATLAB 交叉验证**。
12. **未评估 mean ≤0.5 ms / P99 ≤1 ms / 200 窗/s 目标**：本报告只回答相对提速，
    绝对量级（64/32 648 µs、256/127 1177 µs）仍高于 0.5 ms 目标，且这是纯 DSP
    内核时间，不含 RX/队列/提交。
13. **只测了 code 9 / SFD `4z2` / Legacy 48-tap 脉冲 / 零 CFO**：codes 10–12、
    其他 SFD、Gaussian/Blackman 脉冲未测。
14. **`frontend_cfo_us` 的语义陷阱**：它不含最终 CFO 旋转（在 `frontend_sfd_us` 内）。
    §4.3 的拆分来自**单次未计时诊断调用**（8 次取父计时最小者），不是计时矩阵
    的一部分；用它做审计时请注意这一点。
15. **`cfo_rotated_samples` 未由生产字段导出**：`RadarCirResult` 没有该字段，
    §4.3 的样点数是基准用生产函数 `sync_cir::required_input_range` 复算的并集包络，
    生产实际覆盖还额外并入 final SFD 区间并加 ±128 halo（`covered_hi - covered_lo`）。
16. **`timing_seed_refine_radius` / `timing_max_backtrack_symbols` 未进 CSV 溯源头**：
    它们由 `prepare_radar_cir_core` 从 `sync_refine_margin` 派生，不是
    `RadarCirConfig` 字段，基准无法直接打印。本报告按源码记录为 8 / 0。

---

## 10. 一句话给评审

公共 CIR 内核的收益是真实的、量级明确的（4.7–13.4×，远超 20% 门槛）；
64/32 的整体提速（−24.3%）也真实，其中约 33 µs 来自本轮
`timing_seed_refine_radius=8` / `timing_max_backtrack_symbols=0` 的语义对齐。
但 **256 SYNC 的整体接收阶段仍然变慢 30.8%**，原因不在内核，而在公共前端
**随前导长度线性增长**的整条 SYNC 跟踪成本（387.93 µs vs 旧路径 12.45 µs）。
这 375 µs 是**采用 comm 检测前端的正确性成本**，不是内核缺陷：不跟踪整条训练序列
就无法测周期、无法做 CFO 拟合，而非零 CFO 补偿正是本轮要关闭的缺陷。
下一步若要关闭 256 SYNC 的缺口，具体方向是给 `CirOnly` 增加一个**种子化短跟踪模式**：
只跟踪尾部若干 SYNC 用于 CFO 拟合，再从 SFD 坐标重建整条网格（旧 radar 路径本来
就是这么做的），而不是继续优化内核或继续削种子搜索半径。**该模式尚未实现、尚未验证。**
