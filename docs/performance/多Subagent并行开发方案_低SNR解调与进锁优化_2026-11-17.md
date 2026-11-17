# 多 Subagent 并行开发与修改手册：低 SNR 解调与进锁优化（供 OpenCode 执行）

> **日期**：2026-11-17  
> **基线提交**：`feature/sic-research` @ `0cff03af`（已完成前端整数 `vpmaddwd` SIMD + `-12 dB` 自相关参数升级，52 项 CTest 51 通过 1 跳过）。  
> **问题与依据**：[低SNR解调各阶段错误分析与优化思路_2026-11-17.md](低SNR解调各阶段错误分析与优化思路_2026-11-17.md)、[低SNR进锁阶段优化与快速验证方案_2026-11-17.md](低SNR进锁阶段优化与快速验证方案_2026-11-17.md)、[AWGN_SNR仿真与阈值分析_2026-11-17.md](AWGN_SNR仿真与阈值分析_2026-11-17.md)。  
> **前端边界**：本手册**不再修改** `Acquisition::scan` 与前端环形缓冲（已稳定达到 `-12/-14 dB` `100/100` 捕获与 `< 125 ms` / 0.5 s 单核耗时），专注解决后端 `Timing / CFO / SFD / PHR / Payload` 及 `TargetLockCore` CLI 接入。

---

## 1. 总览：要解决的 5 类错误与优化目标

| 模块 | 实测错误源（见错误分析文档） | 本手册核心修改 | 预期目标 |
|---|---|---|---|
| **Timing** | ① `AcquireLegacy` 初搜卡单符号 `0.25` 门限，`-4 dB` `TIM=67/100`、`-5 dB` `100%` 失败；② `-10 ~ -14 dB` 无带限单符号峰值跌破 `0.06`（`-14 dB` `0/50`） | ① `AcquireLegacy` 局部窗口开放 `allow_par_fallback=true`；② 单符号失败后**按需触发 `K=64` 周期折叠互相关**（门限 `0.05`）+ **16 组×16 符号超级锚点**拟合 | 未锁定 `-3 ~ -8 dB` 零修改折叠即过；`-10 ~ -14 dB` `timing.ok >= 96%` 且高 SNR 无额外耗时 |
| **CFO** | ① SFD 锚定 `Tail Retrack` 写死 `0.20f` 单符号门限（[uwb_demod_core.h#L4215](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L4215)），`-3.5 dB` 引发大量 `CFO_FAILED`（`23/100`、`98/500`）；② `fit_cfo` 要求 `dt <= 1.5*1016` 且低 SNR 单符号解缠绕易跳周 | ① `Tail Retrack` 开放 `PAR >= 20`（`0.06f` 底线）及 16 超级符号备选相位提取；② `fit_cfo` 支持稀疏/超级锚点斜率估算（`dt <= 20*1016`）并支持低幅值时分 16 组相干聚合再拟合 | 消灭 `-3.5 ~ -4 dB` `CFO_FAILED` 尖峰；`-10 ~ -14 dB` CFO 拟合稳定不跳周 |
| **SFD** | ① `stage_sfd` 硬底线 `0.02f`（[uwb_demod_core.h#L1669](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L1669)）拦死 `-14 dB` 弱峰（中位 `0.0194`）；② `±4` 符号回溯范围偏窄，而每槽 `±64` 样点连续扫提高噪底 | **常规路径失败时**触发离散整数符号网格兜底（`k ∈ [-36, +36]`，每格点仅中心 `±1` 样点），保留 `8` 符号相干极性全对校验并下放底线至 `~0.008f` + 局部 PAR 校验 | `-10 dB` 消灭 `SFD=6/100`；`-12 ~ -14 dB` SFD 检出率 `>= 98%` 且纯噪 `0` 虚警 |
| **PHR & Payload** | ① `stage_payload_fused_cmf` 与 `stage_phr` 仅调用硬判决 `vitdec_cl3`（[uwb_demod_core.h#L3179](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L3179)、[#L3817](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L3817)），白丢 `~1.5–2 dB` 增益；② 低 SNR 硬判决不过 FCS 即落入 `15–16 ms` `stage_softchip_fallbacks` 全流 FIR 重算 | 在 `rxfusion` 新增 `decide_bpm_soft`，使 `stage_phr`、`stage_phr_fused_cmf`、`stage_payload_fused_cmf` 原地支持 `vitdec_cl3_soft`（Payload 先试硬判决，失败立即原地跑软判决，仅软判决仍败才触发外层重算） | `-4 ~ -10 dB` 数传 PRR 提升、大幅削减 15 ms `full_retry` 卡顿 |
| **Live 进锁与防误判** | ① `uwb_dw3000_live` 缺少 `--lock-fcs off` 与 `--stop-after-sfd` CLI；② 关闭 FCS 进锁时 [uwb_dw3000_live.cc#L686](../../gr-uwb/apps/uwb_dw3000_live.cc#L686) 会把前导包误记为 `INTERFERENCE ... identity_mismatch` | 新增 `--lock-fcs on\|off` 与 `--stop-after-sfd on\|off`；同步关闭 `--cir auto` 无效重试与 `identity_mismatch` 误统计 | 支撑 `-12/-14 dB` 无 FCS 稳锁测试（`ACQUIRE -> VERIFY -> LOCKED`），默认参数行为零变化 |

---

## 2. Subagent 并行调度编排与防冲突边界

### 2.1 公共工程约束（所有 Subagent 必读）

1. **Git Worktree 隔离**：所有并行 Subagent 在各自独立 worktree 工作（`.worktrees/<agent-name>`，分支 `exp/<agent-name>`），禁止互相直接写对方分支。
2. **仓库硬规则（`AGENTS.md`）**：
   - **禁止触碰** `gr-uwb/build-avx512-smtcheck`；
   - **禁止添加** `-march=native`，保持 `UWB_SIMD=AUTO|AVX2|AVX512|SCALAR` 可移植性；
   - 保留分数网格回退、`Full -> Bypass`（`grid_reject`）降级、`timed` 无 `--replay` 时 fail-closed；
   - **严禁误用夹具**：`tools/rx/proto_low_snr_signal.py` 只生成**纯前导**（无 SFD/PHR/Payload），只能用于 `Timing` 与 `--stop-after-sfd` 测试；凡验证 `SFD / PHR / Payload / FCS`，必须使用 `gr::uwb::mod::modulate_one` 生成完整 BPRF 帧！
3. **统一构建与单测命令**（当前 `i7-12700` 主机）：
   ```bash
   cmake -S gr-uwb -B gr-uwb/build-native -DCMAKE_BUILD_TYPE=Release \
     -DUWB_SIMD=AUTO -DUWB_BUILD_GNURADIO=OFF \
     -DENABLE_UHD_BACKEND=OFF -DUWB_BUILD_EXPERIMENTS=OFF
   cmake --build gr-uwb/build-native -j
   ctest --test-dir gr-uwb/build-native --output-on-failure
   ```
   验收标准：52 项中 51 项通过、1 项（`uwb_qa_native_install_consumer`）按设计跳过，0 失败。

### 2.2 波次划分（3 个 Wave，严格按文件/行段隔离）

```text
Base (0cff03af)
  ├── Wave 1（三路同时启动，零函数/文件冲突）
  │     ├── Agent-Live-LockCfg  ── 独占 uwb_dw3000_live.cc + qa_live_dw3000_console.py
  │     ├── Agent-Timing        ── 独占 uwb_demod_core.h: stage_timing (L408-L1283) + qa_uwb_radar_timing_core.cc
  │     └── Agent-Payload-Soft  ── 独占 uwb_rx_fused_cmf.h + uwb_demod_core.h (L2545-L3827) + fused QA
  │
  ├── [Integrator 合并 Wave 1 + 全量 CTest] -> tag: base-w1
  │
  ├── Wave 2（基于 base-w1 两路同时启动，均改 uwb_demod_core.h 正交函数段）
  │     ├── Agent-CFO           ── 独占 fit_cfo (L1296-L1433) + Tail Retrack (L4201-L4469) + qa_uwb_demod_core.cc
  │     └── Agent-SFD           ── 独占 stage_sfd (L1459-L1753) + qa_uwb_radar_sfd_core.cc
  │
  └── Wave 3（Integrator 合并 Wave 2 -> 跑完整回归矩阵与基准表）
```

> [!IMPORTANT]
> **为什么 `Agent-CFO` 与 `Agent-SFD` 放 Wave 2**：
> `Agent-Timing` 会在折叠路径下产出 16 组超级锚点（`peak_samples` 间距约 `16 * 1016`）并更新 `TimingResult`；`Agent-CFO` 需要适配稀疏锚点的 `unwrap_slope` 计算并放宽 Tail Retrack，`Agent-SFD` 需要消费 `TimingResult` 对齐后的格点做离散搜索。把它们放在 Wave 1 集成之后的 `base-w1` 上并行，可避免重复返工与结构体语义错位。

---

## 3. 各 Subagent 可执行详细设计 & 下发 Prompt

以下每节均包含**精确代码定位、关键不变量/避坑提示、直接复制即可下发给 OpenCode 的 Prompt**。

---

### Subagent 1：`Agent-Live-LockCfg`（Wave 1，无核心库冲突）

- **Worktree / 分支**：`.worktrees/low-snr-live-lock` / `exp/low-snr-live-lock`（基于 `0cff03af`）
- **独占文件**：`gr-uwb/apps/uwb_dw3000_live.cc`、`tools/rx/qa_live_dw3000_console.py`
- **禁止改动**：`gr-uwb/include/gnuradio/uwb/*.h`

#### 修改清单
1. **新增 CLI 选项**（`Options`、`--help`、参数解析）：
   - `--lock-fcs on|off`（默认 `on`）：映射到 `lock_cfg.require_fcs`（默认 `true` 保持现有行为完全不变）。
   - `--stop-after-sfd on|off`（默认 `off`）：映射到 worker 解调配置 `p.rx_stop_after_sfd`（默认 `false`）。
   - 非法值（非 `on`/`off`）复用现有错误退出码 `2` 报错。
2. **修复 `lock_fcs == false` 与 `stop_after_sfd == true` 下的 2 处逻辑联动**：
   - [uwb_dw3000_live.cc#L610-L612](../../gr-uwb/apps/uwb_dw3000_live.cc#L610-L612)：`retry` 判断加前置 `!p.rx_stop_after_sfd && ...`（既然主动停在 SFD，绝不能因 `!fcs_pass` 触发 `Full` 全流重试）。
   - [uwb_dw3000_live.cc#L686-L702](../../gr-uwb/apps/uwb_dw3000_live.cc#L686-L702)：仅当 `o.lock_fcs && !o.stop_after_sfd` 时，才对 `!identity` 的前导+SFD 成功包统计 `++interference_events` 与打印 `INTERFERENCE ... reason=identity_mismatch`；否则只靠 `TargetLockCore` 凭网格残差区分目标与干扰。
3. **补充 QA（[qa_live_dw3000_console.py](../../tools/rx/qa_live_dw3000_console.py)）**：
   - 校验 `--lock-fcs invalid` / `--stop-after-sfd invalid` 返回码 `2`。
   - 用已有测试夹具验证 `--lock-fcs off --stop-after-sfd on` 可正常启动、无崩溃并能完整退出。

#### 可下发 Prompt（`Agent-Live-LockCfg`）
```text
你是 Agent-Live-LockCfg，在独立 worktree `.worktrees/low-snr-live-lock`（分支 `exp/low-snr-live-lock`，基线 `0cff03af`）完成任务。

【修改范围（严禁越界）】
- 只允许修改：`gr-uwb/apps/uwb_dw3000_live.cc`、`tools/rx/qa_live_dw3000_console.py`。
- 严禁改动 `gr-uwb/include/gnuradio/uwb/*.h` 与 `Acquisition::scan` 算法参数，严禁改动 `gr-uwb/build-avx512-smtcheck`。
- 遵守根目录 `AGENTS.md`（不加 `-march=native`、`timed` 无 `--replay` 保持 fail-closed 等）。

【任务内容】
1. 在 `uwb_dw3000_live.cc` 的 `Options`、CLI `--help` 和参数解析中新增：
   - `--lock-fcs on|off`（默认 `on`，映射 `lock_cfg.require_fcs = (o.lock_fcs == "on")`）
   - `--stop-after-sfd on|off`（默认 `off`，映射 worker 的 `p.rx_stop_after_sfd`）
   非法取值必须报错并返回退出码 2（与现有 `--tone/--fused` 一致）。
2. 修复两处低 SNR 进锁误触发：
   - worker 中 `--cir auto` 重试条件（约 L610）仅在 `!p.rx_stop_after_sfd` 时生效；
   - 非 `target_slot` 的 `INTERFERENCE ... reason=identity_mismatch` 诊断计数（约 L686）仅在 `lock_cfg.require_fcs && !p.rx_stop_after_sfd` 时记录，避免 `--lock-fcs off` 时把进锁包误报为干扰事件。
3. 在 `tools/rx/qa_live_dw3000_console.py` 增加新参数非法值拒绝与 `--lock-fcs off --stop-after-sfd on` 冒烟用例。

【验收要求】
- 编译 `gr-uwb`（Release, `UWB_SIMD=AUTO`, `UWB_BUILD_GNURADIO=OFF`, `ENABLE_UHD_BACKEND=OFF`, `UWB_BUILD_EXPERIMENTS=OFF`）并跑全量 `ctest --output-on-failure`，确认 51 pass / 1 skip (`uwb_qa_native_install_consumer`) / 0 fail。
- 默认开关行为必须与基线完全一致。完成后在 `exp/low-snr-live-lock` 提交干净 commit 并汇报 `git diff --stat` 与测试输出。
```

---

### Subagent 2：`Agent-Timing`（Wave 1）

- **Worktree / 分支**：`.worktrees/low-snr-timing` / `exp/low-snr-timing`（基于 `0cff03af`）
- **独占文件/函数**：`gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 内的 `stage_timing`（约 L408–L1283，及紧邻可添加在 `detail::` 内的折叠辅助函数）、`gr-uwb/lib/qa_uwb_radar_timing_core.cc`
- **禁止改动**：`fit_cfo`/`stage_cfo`、`stage_sfd`、`stage_phr*`、`stage_payload*`、`demodulate_one` 下游函数及 `TimingResult` 既有字段删改。

#### 修改清单与关键防坑点
1. **`AcquireLegacy` 局部初搜放开 `allow_par_fallback`（解决 `-3 ~ -8 dB` 未锁定悬崖）**：
   - 在 `!bounded` 分支的 `narrow`（L751）与 `legacy_seed_radius`（L774）`local_best` 调用中，将 `allow_par_fallback` 改为 `true`（内部已实现 `count ∈ [64, 1024]`、`PAR >= 20`、`kSyncParFloor = 0.06f`）。
   - **防坑**：仅在有界局部窗内开启，**不要**给全窗口 `coarse_scan`（L785）或全网格探测（L833）开 PAR 兜底，防止无信号纯噪窗口虚警飙升。
2. **仅在常规单符号路径失败时触发 `K=64` 周期折叠兜底（解决 `-8 ~ -14 dB`）**：
   - 触发位置：当 `bounded` 分支在 `first_start < 0`（[uwb_demod_core.h#L622](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L622)）即将 `return false` 前，以及 `!bounded` 分支在（a）（b）（c）之后仍 `first_start < 0`（L839）时；**高 SNR 与能走通单符号的包完全零额外开销**。
   - **搜索窗口**：`bounded` 取 `seed ± roi`；`!bounded` 取 `seed ± std::min<int64_t>(margin, 8192)`（注意边界裁剪，确保折叠区 `[w0, w1 + K*1016)` 落在 `rx[0..n)` 内，若可用完整符号数 `< 64`，则自适应退化为 `K_eff = min(64, max_avail)`，至少 `>= 16`）。
   - **折叠度量**：
     - 构造长度 `fold_len = (j_hi - j_lo) + L` 的相干叠加缓冲 `folded[m] = sum_{k=0}^{K_eff-1} rx[j_lo + m + k * 1016]`（直接使用 `L = 1016` 实数或复模板做滑窗归一化相关，能量分母用 `pw_folded[j]` 滑动求和）。
     - **防坑（门限标度）**：折叠级门限**不能**沿用单符号 `0.25`，必须用噪底门限 **`0.05f`**（参考实测：纯噪 203,200 次最大 `0.0107`，`-14 dB` `K=64` 中位 `0.560`）。
3. **16 组×16 符号超级锚点拟合与 `TimingResult` 回填**：
   - 当本包是靠折叠兜底才定位到 `first_start` 时，逐符号跟踪门限自然也过不了；此时直接改为对 `256` 个符号分 **`G <= 16` 组（每组连续 `16` 个 SYNC 符号相干累加）**：
     - 对第 `g` 组，先以 `first_start + g * 16 * 1016` 为起点把连续 16 个符号叠加，然后在预期起点 `±8` 样点内搜索最大归一化相关峰，记录精确位置与**该组的相干复相关值 `corr_g`**；
     - 用所有通过局部峰检（或主组中最高峰们）的 `G` 个超级锚点做普通最小二乘线性拟合，得到 `measured_period` 与 `preamble_start_sample`；
     - 将这批锚点位置与相干复相关值 `corr_g` 填入 `out.peak_samples`、`out.peak_metrics`、`out.peak_corr`、`out.detected_peaks`，并标记 `out.ok = true`。
   - **防坑**：确保 `first_start` 定位的是**符号起点**而非峰值尾端偏移（与现有 `local_best` 的 `pos = j` 对齐一致，`peak_samples[g] = pos_g + (L - 1)`，与现有 `fit_cfo` / `stage_cir_softchips` / `sfd_start` 坐标系严格对齐！）。

#### 可下发 Prompt（`Agent-Timing`）
```text
你是 Agent-Timing，在独立 worktree `.worktrees/low-snr-timing`（分支 `exp/low-snr-timing`，基线 `0cff03af`）完成任务。

【修改范围（严禁越界）】
- 只允许修改：`gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 中的 `stage_timing`（L408–L1283 及其在 `detail::` 内的私有折叠辅助函数）以及 `gr-uwb/lib/qa_uwb_radar_timing_core.cc`。
- 严禁修改 `fit_cfo`/`stage_cfo`、`stage_sfd`、`stage_cir_softchips`、`stage_phr*`、`stage_payload*` 与 `demodulate_one` 的其它阶段代码。
- 遵守 `AGENTS.md`（保留分数网格/Full->Bypass/无 `-march=native` 等）。

【任务内容】
1. `AcquireLegacy` 局部窗口放开 PAR 兜底：在 `stage_timing` 的 `!bounded` 分支下，`narrow` 与 `legacy_seed_radius` 的局部 `local_best` 调用开启 `allow_par_fallback = true`（`PAR >= 20`, `kSyncParFloor = 0.06f`）。不要对宽 `coarse_scan` 开启 PAR 兜底。
2. 按需 `K=64` 周期折叠兜底（仅在已有 `LockedBounded` 与 `AcquireLegacy` 单符号搜索全部失败时触发，高 SNR 路径零开销）：
   - 在 `bounded` 下搜索 `seed ± roi`，`!bounded` 下搜索 `seed ± min(margin, 8192)`（注意缓冲区边界保护，若不足 64 符号则取可用整数符号 `K_eff >= 16`）；
   - 按标称周期 `L = 1016` 相干折叠 `K_eff` 个符号，用现有同步模板计算滑动归一化互相关；
   - 折叠门限使用按噪底标定的 `0.05f`（不要沿用 `0.25f`），定出精确的 `first_start`。
3. 折叠命中后的 16 超级锚点周期拟合：
   - 从 `first_start` 起每 16 个 SYNC 符号相干叠加为一个超级符号（共 `G <= 16` 组），在每组预期位置 `±8` 样点精寻峰，用 `G` 个锚点最小二乘拟合 `measured_period` 与 `preamble_start_sample`；
   - 注意坐标对齐契约：`first_start` 为符号起始索引，`peak_samples` 存 `pos + (L - 1)`，`peak_corr` 存对应超级组复相关累加值，`detected_peaks` 写入有效锚点数。

【验收要求】
1. Release 构建 + 全量 `ctest --output-on-failure` 全绿（51 pass / 1 skip）。
2. 在 `qa_uwb_radar_timing_core.cc` 新增单测覆盖：
   - 工作/原生网格下 `-4 dB` 与 `-8 dB` 未锁定态 `stage_timing` 通过（无需折叠或快速通过）；
   - `-12 dB` 与 `-14 dB`（50 帧）下 `stage_timing` 通过率 `>= 48/50` 且 `|preamble_start_sample - true_start| <= 2`；
   - 100 段纯 AWGN 窗内折叠兜底虚警率 `== 0`。
3. 提交干净 commit 并附真实测试输出。
```

---

### Subagent 3：`Agent-Payload-Soft`（Wave 1）

- **Worktree / 分支**：`.worktrees/low-snr-payload-soft` / `exp/low-snr-payload-soft`（基于 `0cff03af`）
- **独占文件/函数**：
  - `gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h`（新增 `decide_bpm_soft`）
  - `gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 中的 `DemodScratch::fused_phr`（加缓存 `llr0/llr1`）、`detail::decode_payload_soft`、`decode_phr_decisions_soft`、`stage_phr`、`stage_phr_fused_cmf`、`stage_payload_fused_cmf`（以及 `demodulate_one` 中调用 `stage_phr_fused_cmf`/`stage_payload_fused_cmf` 传入 `phr_llr0/llr1` 的数行胶水；**严禁**修改 `stage_timing`/`fit_cfo`/`stage_sfd`）
  - `gr-uwb/lib/qa_uwb_rx_fused_cmf.cc`、`gr-uwb/lib/qa_uwb_rx_fused_phr.cc`

#### 修改清单与关键防坑点
1. **`rxfusion::decide_bpm_soft`（精确对齐 `bprf_demod_soft` 符号约定）**：
   - 对照 [uwb_demod_core.h#L2545-L2599](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L2545-L2599) 与 [uwb_rx_fused_cmf.h#L528-L560](../../gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h#L528-L560)：
     - `b0 = max(|m[0]|, |m[2]|)`（`best0` 为对应有符号值）
     - `b1 = max(|m[1]|, |m[3]|)`（`best1` 为对应有符号值）
     - `llr_g0[s] = b0 - b1`（`> 0` 倾向 `g0 = 0`）
     - `llr_g1[s] = (b0 >= b1) ? best0 : best1`（`> 0` 倾向 `g1 = 0`）
   - 保留原 `decide_bpm` 供现有 callers 逐位一致。
2. **`stage_phr` 与 `stage_phr_fused_cmf` 软判决 Viterbi**：
   - 新增 `decode_phr_decisions_soft(llr0, llr1, g0, g1, soft_chips, ns_sfd, out)`：调用 `detail::vitdec_cl3_soft(llr0.data(), llr1.data(), 21, decoded)`（随后复用与 `decode_phr_decisions` 完全相同的 SECDED 逻辑和 `mean_tail < 0.15 * mean_body` 护栏——注意提取共用的 SECDED 校验/解析 helper，**不要复制代码**）。
   - `stage_phr` 改用 `detail::bprf_demod_soft` + `decode_phr_decisions_soft`。
   - `stage_phr_fused_cmf` 改用 `rxfusion::decide_bpm_soft` + `decode_phr_decisions_soft`，并把 `21` 组 `llr0/llr1` 缓存在 `scratch.fused_phr` 中（注意 `reset_decisions()` 时同步清理有效标记，防止跨包脏读！）。
3. **`stage_payload_fused_cmf` 原地软判决兜底**：
   - 新增共享 helper `detail::finish_rs_crc_payload(...)` 与 `detail::decode_payload_soft(llr0p, llr1p, llr0, llr1, psdu_bits, out, sub)`，复用现成的 `vitdec_cl3_soft`、`rs_decode_stream` 和 `crc16_802154`（计时同样归入 `sub->viterbi_us / rs_us / crc_us`）。
   - 在 `stage_payload_fused_cmf` 中：
     - 先跑 `decode_payload_hard(...)`；
     - 若失败或 `!out.fcs_pass`，**不直接 `return false` 退回 15 ms full retry**，而是从 `scratch.fused.metrics` 调用 `rxfusion::decide_bpm_soft` 得到 payload `llr0/llr1`（PHR 部分优先取 `phr_llr0/phr_llr1`，若空缺则调用 `detail::bprf_demod_soft(prefix_soft_chips, ...)`），原地跑一次 `decode_payload_soft`；
     - 仅当原地软判决也失败时才 `return false`（并由外层 `demodulate_one` 保留原 `stage_softchip_fallbacks` 兜底，含 `±2` 长度试错和正反极性翻转）。
   - **重要提示（与 `stage_payload_fcs` 对照一致性）**：检查 `stage_phr_fused_cmf` 与 `stage_payload_fused_cmf` 在硬判决路径上的输出，确认在高 SNR（如 Golden 127 B 测试用例）下**输出逐字节一致、耗时无退化**。

#### 可下发 Prompt（`Agent-Payload-Soft`）
```text
你是 Agent-Payload-Soft，在独立 worktree `.worktrees/low-snr-payload-soft`（分支 `exp/low-snr-payload-soft`，基线 `0cff03af`）完成任务。

【修改范围（严禁越界）】
- 只允许修改：`gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h`、`gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 中的 `DemodScratch::fused_phr`、L2936–L3827（PHR/Payload 解码函数）以及 `demodulate_one` 中传入 fused PHR LLR 的调用行；以及 `gr-uwb/lib/qa_uwb_rx_fused_cmf.cc`、`gr-uwb/lib/qa_uwb_rx_fused_phr.cc`。
- 严禁修改 `stage_timing`、`fit_cfo`/`stage_cfo`、`stage_sfd`、`stage_cir_softchips` 或 `uwb_dw3000_live.cc`。

【任务内容】
1. 在 `uwb_rx_fused_cmf.h` 新增 `rxfusion::decide_bpm_soft(...)`：
   - 保持与 `detail::bprf_demod_soft` 完全等价的 LLR 定义：`llr_g0 = b0 - b1`、`llr_g1 = (b0 >= b1) ? best0 : best1`（正值倾向 bit 0）。
   - 保留原 `decide_bpm` 不变。
2. PHR 软判决化（消除 -8~-10 dB PHR 长度滑移）：
   - 将 `decode_phr_decisions` 抽取共享 SECDED + 尾能量护栏 helper，不重复代码；新增 `decode_phr_decisions_soft` 调用 `vitdec_cl3_soft`。
   - `stage_phr` 改用 `bprf_demod_soft` + `decode_phr_decisions_soft`；`stage_phr_fused_cmf` 改用 `decide_bpm_soft` + `decode_phr_decisions_soft`，并缓存 21 符号 `llr0/llr1` 到 `scratch.fused_phr`（`reset_decisions()` 同步失效）。
3. `stage_payload_fused_cmf` 增加原地软判决兜底：
   - 新增 `detail::decode_payload_soft(...)`（同样共用 RS+CRC helper 与 `PayloadSubTimers` 打点）。
   - 在 `stage_payload_fused_cmf` 先走原有 `decode_payload_hard`（高 SNR 零额外耗时），若返回 false 或 `!out.fcs_pass`，原地用 PHR LLR + `decide_bpm_soft` 产出的 payload LLR 调用 `decode_payload_soft`；仅当仍失败才返回 false。

【验收要求】
1. Release 构建 + 全量 `ctest --output-on-failure` 全绿（51 pass / 1 skip）。
2. 在 `qa_uwb_rx_fused_cmf.cc` / `qa_uwb_rx_fused_phr.cc` 新增验证：
   - 验证 `decide_bpm_soft` 产出的 LLR 与 `bprf_demod_soft` 对同一输入信号符号/排序一致；
   - 构造仅靠硬判决会失败但软判决 Viterbi 可过 FCS 的低 SNR 合成测试（用 `gr::uwb::mod::modulate_one` 生成完整帧），证明 `stage_payload_fused_cmf` 原地成功且 `stage_softchip_fallbacks == 0`。
3. 提交干净 commit 并附测试结果。
```

---

### [Wave 1 集成检查点]（主控执行）
按顺序 merge 至集成分支 `exp/low-snr-integration`（假设形成标签 `base-w1`）：
1. `Agent-Live-LockCfg` -> build & `ctest`
2. `Agent-Timing` -> build & `ctest`
3. `Agent-Payload-Soft` -> build & `ctest`

确认 51/51 可运行测试全过后再派发 Wave 2。

---

### Subagent 4：`Agent-CFO`（Wave 2，基于 `base-w1`）

- **Worktree / 分支**：`.worktrees/low-snr-cfo` / `exp/low-snr-cfo`（基于 `base-w1`）
- **独占文件/函数**：`gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 中的 `fit_cfo`（L1296–L1433）以及 `demodulate_one` 内的 Tail Retrack + `tail_cfo` 段（L4201–L4469）；`gr-uwb/lib/qa_uwb_demod_core.cc`。
- **禁止改动**：`stage_timing`、`stage_sfd`、`stage_phr*`、`stage_payload*`。

#### 修改清单与关键防坑点
1. **适配 `stage_timing` 超级锚点的稀疏 `unwrap_slope` 估计（解决 `dt > 1.5 * 1016` 被跳过问题）**：
   - 见 [uwb_demod_core.h#L1340-L1361](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L1340-L1361)：当 `stage_timing` 走折叠兜底返回 16 超级锚点时，相邻 `peak_samples` 间隔约 `16 * 1016 = 16256 > 1.5 * 1016`，导致 `adj_sum` 为 `0`、`unwrap_slope_rad_per_sample = 0.0`。
   - **改法**：若 1-符号相邻对（`dt <= 1.5 * L`）数量不足，回退统计更宽间距的相邻锚点对（如 `dt <= 20 * L`），对每对计算加权平均相位斜率（注意：先用首对小间隔或按 `std::arg(c1 * std::conj(c0)) / dt` 取中值/一致性方向，只要 `|CFO| < fs / (2 * 16 * 1016) ≈ ±30.7 kHz`，16-符号跨度内无 `±π` 卷绕歧义！）。
   - **低平均幅值密集峰自动聚合成超级块**：若 `have_corr` 且 `nfit >= 32` 密集单符号峰的平均幅度极低，可先每 16 个相邻 `peak_corr` 矢量平均后算 `arg` 再线性拟合（注意仅在存在多符号相位抖动或先用邻对估过一次粗斜率并预先按粗斜率对齐 16 符号内部小旋转后再相干累加，避免高 CFO 下 16 符号相干抵消！）。
2. **解除 `Tail Retrack` 写死的 `0.20f` 阻塞（直接消灭 `-3.5 dB` 大量 `CFO_FAILED`）**：
   - 问题位置：[uwb_demod_core.h#L4215](../../gr-uwb/include/gnuradio/uwb/uwb_demod_core.h#L4215)（`cfo_anchor_profile.radar_verification_threshold = 0.20f`）与 L4324–L4358（`tail_timing.detected_peaks >= 4`）。
   - **改法**：
     - Tail Retrack 起点已由 `bootstrap_sfd` 钉在 `±8` 样点，几乎无虚警风险；允许 `tail_timing` 走 `LockedBounded` 相同的 `PAR >= 20`（`0.06f` 底线）判据，而不是 `0.20f` 一刀切；
     - 并且因为 `Agent-Timing` 已有折叠+超级锚点兜底，当尾部 40 符号仍不足 4 峰时，利用 `bootstrap_sfd_start` 与 `period_hint` 构造尾部每 16 符号（或 8 符号）相干复相关超级块送给 `fit_cfo`，保证只要 `bootstrap_sfd_ok` 成立就不再轻易卡死 `!res.cfo.ok`。

#### 可下发 Prompt（`Agent-CFO`）
```text
你是 Agent-CFO，在独立 worktree `.worktrees/low-snr-cfo`（分支 `exp/low-snr-cfo`，基线 `base-w1`）完成任务。

【修改范围（严禁越界）】
- 只允许修改：`gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 中的 `fit_cfo`（L1296–L1433）及 `demodulate_one` 内 SFD 锚定 Tail Retrack（L4201–L4469）；以及 `gr-uwb/lib/qa_uwb_demod_core.cc`。
- 严禁修改 `stage_timing`、`stage_sfd`、`stage_phr*`、`stage_payload*` 或 `uwb_dw3000_live.cc`。

【任务内容】
1. 适配稀疏/超级锚点的 `fit_cfo` 解缠绕：
   - 修复 L1340 `max_adj_dt = 1.5 * sym` 对 16-符号超级锚点（间隔 ~16*1016）失效的问题：当短间距有效对为 0 时，支持用 `dt <= 20 * sym` 的相邻对估计 `unwrap_slope_rad_per_sample = arg(c1 * conj(c0)) / dt`（在 `|CFO| <= 15 kHz` 范围内 16 符号相偏 `< 1.54 rad`，无模糊）。
   - 对于大量低信噪比单符号 `peak_corr`（如 `nfit >= 32` 且低相关），可先按粗相邻斜率做去斜或直接在 16 符号组内复相干合并后再做 `unwrap + LS` 拟合（注意不改变高 SNR `cfo_hz` 精度：高 SNR 下 `cfo_hz` 误差必须保持不变）。
2. 消除 Tail Retrack 的 `0.20f` 刚性 `CFO_FAILED` 悬崖（L4201–L4469）：
   - Tail Retrack 允许使用 `PAR >= 20`（`0.06f` 底线）的 `LockedBounded` 局部重跟，避免 `-3.5 ~ -4.0 dB` 因单峰落在 `0.18~0.20` 之间导致 `detected_peaks < 4` 而报 `CFO_FAILED`；
   - 若尾部逐符号仍不足 4 峰，按 `bootstrap_sfd_start` 与 `period_hint` 对尾部 SYNC 做 8/16 符号分组相干累加提取至少 4 个锚点供 `tail_cfo` 拟合。

【验收要求】
1. Release 构建 + 全量 `ctest --output-on-failure` 全绿（51 pass / 1 skip）。
2. 在 `qa_uwb_demod_core.cc` 新增验证：
   - 存在 `±2 kHz` CFO 的 `-10 dB` 与 `-14 dB` 仿真帧，SFD 后 Tail CFO 拟合不返回 `CfoFailed` 且残余频率误差满足解调要求（`|err| < 50 Hz`）；
   - 高 SNR 带 CFO（如 `±2 kHz @ +10 dB`）逐包精度与基线相比零退化。
3. 提交干净 commit 并附测试输出。
```

---

### Subagent 5：`Agent-SFD`（Wave 2，基于 `base-w1`）

- **Worktree / 分支**：`.worktrees/low-snr-sfd` / `exp/low-snr-sfd`（基于 `base-w1`）
- **独占文件/函数**：`gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 中的 `stage_sfd`（L1459–L1753）；`gr-uwb/lib/qa_uwb_radar_sfd_core.cc`。
- **禁止改动**：`stage_timing`、`fit_cfo`、`demodulate_one` 其他阶段、`uwb_dw3000_live.cc`。

#### 修改清单与关键防坑点
1. **保留原有快路径，仅在常规 `stage_sfd` 失败时启动「离散整数符号网格兜底」**：
   - 先完整跑完现行 L1734–L1741（`k ∈ [0, ±4]`、`kSfdNarrowHalf = 64`）；若 `sfd_passed == true`，直接返回，高 SNR / 中等 SNR 与多径容忍度与基线**逐位一致**。
   - 当且仅当 `!sfd_passed` 时，进入低 SNR 离散符号网格兜底（需满足 `use_real_sfd_fast` 或常规 `sfd_dot`，以覆盖 `real` 和 `complex` 模板）：
     - 候选偏移网格：整数符号偏移 `k ∈ [-36, +36]`（若范围越界自动跳过），在每个符号格点 `expected + k * period` 附近仅评估 `δ ∈ {-1, 0, +1}`（共 `73 * 3 = 219` 个点，或先按 `δ=0` 遍历 `73` 个格点再对 top-1 格点精修 `±1`，将噪声最大期望压到最低！）；
     - 注意 `expected = timing.preamble_start_sample + preamble_repetitions * period`（已考虑符号长度，确保正偏移与负偏移均落在 `[0, n - sfd_len]` 内）。
2. **放宽 `0.02f` 硬底线 + 离散网格 PAR 与 8 符号极性全校验**：
   - 在离散网格兜底分支上：
     - 用其余 `70+` 个离散格点（排除主峰邻近 `±2` 个符号格点）的背景均值估算局部 PAR；
     - 判通条件：
       1. 峰值底线由 `0.02f` 放宽至 **`0.008f`**（实测 `-10 dB` 最小 `0.0409`、`-14 dB` 最小 `0.0151`、`-16 dB` 最小 `0.0088`）；
       2. 离散网格 **`par >= 5.5`**（注：此前离散网格 `med_PAR` 为 `8.3 ~ 8.8`，需在纯噪 200+ 窗上实测分布确认此 PAR+极性组合虚警为 `0`，必要时可微调至 `6.0`）；
       3. **必须通过 `8` 符号逐符号相干极性全匹配校验**（复用 L1691–L1719 已有代码：`sign_ok == n_check`；这是低 SNR 下抗噪声虚警的硬锁：8 个随机符号独立同相的概率仅 $2^{-7} \approx 0.78\%$，叠加 PAR 门限虚警可压至零！）。
   - 将 L1691–L1719 的“8 符号极性全匹配校验”抽成一个独立局部 lambda，两处复用，不复制代码。

#### 可下发 Prompt（`Agent-SFD`）
```text
你是 Agent-SFD，在独立 worktree `.worktrees/low-snr-sfd`（分支 `exp/low-snr-sfd`，基线 `base-w1`）完成任务。

【修改范围（严禁越界）】
- 只允许修改：`gr-uwb/include/gnuradio/uwb/uwb_demod_core.h` 中的 `stage_sfd`（L1459–L1753）以及 `gr-uwb/lib/qa_uwb_radar_sfd_core.cc`。
- 严禁修改 `stage_timing`、`fit_cfo`、`stage_phr*`、`stage_payload*` 与 `demodulate_one` 的其它阶段代码。

【任务内容】
1. 保持原有 `stage_sfd`（`k ∈ [0, ±4]`、`kSfdNarrowHalf = 64`）逻辑逐位不变作为主路径，仅在主路径 `!sfd_passed` 时触发低 SNR 离散整数符号网格兜底。
2. 离散网格兜底：
   - 沿 `expected + k * period` 扫描整数符号格点 `k ∈ [-36, +36]`（越出 `[0, n - sfd_len]` 的裁剪跳过），并在主格点及紧邻 `±1` 样点定位最优峰；
   - 背景均值取排除主峰格点 `±2` 符号以外的所有离散格点指标均值计算 `par`；
   - 将 `0.02f` 硬底线在该兜底分支放宽至 `0.008f`，要求同时满足 `peak >= 0.008f`、`par >= 5.5`（需通过纯噪虚警测试标定）以及 L1691–L1719 的 `8` 符号逐符号相干极性全匹配（请抽为共用 lambda，避免代码重复）。

【验收要求】
1. Release 构建 + 全量 `ctest --output-on-failure` 全绿（51 pass / 1 skip）。
2. 在 `qa_uwb_radar_sfd_core.cc` 新增验证（使用完整含 `4z2` SFD 的波形）：
   - `-10 dB / -12 dB / -14 dB`（各 >= 50 帧，含随机 `±10` 符号起始偏移扰动）下 `stage_sfd` 通过率 `>= 96%` 且定位精确；
   - 200 个纯 AWGN 窗上 `stage_sfd` 虚警 `== 0`。
3. 提交干净 commit 并附测试结果。
```

---

## 4. Wave 3：主控集成与快速端到端验证矩阵（Integrator 执行）

依次将 `Agent-CFO`、`Agent-SFD` 合入 `exp/low-snr-integration`，每次合并均编译并跑 `ctest`，全部通过后执行以下 **3 组最小无冗余对比测试**（全部用临时 `/tmp/low_snr_eval` 目录，测完清理）：

1. **高/中/低完整帧对比矩阵（验证 Payload/PHR 软判决 + Tail Retrack + AcquireLegacy PAR）**：
   - 使用调用 `gr::uwb::mod::modulate_one` 的完整帧生成器（127 B PSDU, 6.81 Mb/s, code 10, 256 SYNC, `4z2` SFD, 经 `48/65` 重采样，542 LSB RMS）生成 **`+4 dB`、`-3.5 dB`、`-4.0 dB`、`-8.0 dB`、`-10.0 dB`** 各 `200` 包（1 ms 周期），在默认 `--cir full --fused on --taps eq16 --tone off` 下分别对比 **`--sync acquire` 与 `--sync locked`**（或对比集成前后二进制）：
   - **关键观察指标**：
     - `+4 dB`：PRR 保持 `100%`，`demod_mean` 相比基线不退化；
     - `-3.5 / -4.0 dB` `acquire`：`CFO_FAILED` 与 `TIMING_FAILED` 应大幅归零；
     - `-4 ~ -10 dB`：`full_retry`（`stage_softchip_fallbacks`）显著减少，`-10 dB` PRR 较基线 `75%` 明显提升。
2. **`-12 dB` 与 `-14 dB` 完整帧端到端进锁闭环验证**：
   - 用完整帧 `-12 dB` 与 `-14 dB`（各 `200` 包，1 ms 周期）跑：
     ```bash
     ./gr-uwb/build-native/apps/uwb_dw3000_live \
       --replay /tmp/low_snr_eval/full_m12.sc16 --seconds 0 --period-ms 1.0 \
       --mode continuous --sync locked --workers 4 \
       --cir full --fused on --taps eq16 --tone off \
       --lock-fcs off --stop-after-sfd on
     ```
   - **验收指标**：`raw_drop_samples=0`、`acq_seeds` `>= 198/200`、`lock_entries=1`、`reacquire_entries=0`、`final_state=LOCKED`。
3. **纯 AWGN 虚警回归（0.5 s = 368.64M 样点，分别跑 `--tone on` 与 `--tone off`）**：
   - **验收指标**：`acq_seeds=0`、`candidates=0`、`completed=0`、`frontend_us <= 125 ms`、`raw_drop_samples=0`。

---

## 5. 验收完成后收尾

1. 主控确认 `git diff --stat` 仅含预期源码与 QA 文件（无调试残留和 `/tmp` 大文件）。
2. 将最终实测对比数据整理归档成开发报告（`docs/performance/低SNR解调与进锁优化开发报告_2026-11-17.md`），再合回 `feature/sic-research`。
