# 多 Subagent 并行开发方案：Ninja 编译加速与头文件解耦重构（2026-11-21）

- **适用仓库与基准**：`/home/junqima/workspace/openUWB`（基于 `main @ 91f0b23` / `feature/openuwb-rx-sic` 现行目录结构）
- **环境现状**：本机已安装 `/home/junqima/.local/bin/ninja`；`CMakePresets.json` 已声明 `cmakeMinimumRequired 3.20`；仓库已完成 Phase 0–4 域分层（`lib/CMakeLists.txt` 单向静态库 DAG）。
- **执行策略**：两阶段推进 + 多 Subagent 零冲突并行（Wave 1 构建底座升级 → Wave 2 高扇出头文件并行下沉 → Wave 3 PHY/Runtime 拆分与全局验收）。
- **合规边界**：严格遵守仓库根目录 [AGENTS.md](../../AGENTS.md)：
  - 严格保持 `openuwb_support → openuwb_dsp → openuwb_phy_hrp → {openuwb_sic, openuwb_sensing, openuwb_twr_core}` 与 `openuwb_runtime → openuwb_phy_hrp + openuwb_sic + openuwb_sensing + openuwb_radio` 单向依赖，**禁止反向依赖**，只有 `openuwb_radio` 可链接 `UHD::UHD`。
  - 保留 `include/gnuradio/uwb/*.h` 过渡兼容转发头、不修改既有 QA 容差、不放宽 `UwbSimd.cmake` 的 ISA/ODR 约束、禁止 `-march=native`。
  - 全量验证必须使用 `env -u LD_LIBRARY_PATH`；覆盖 `native-avx2`、`no-uhd`、`native-avx512`（仅编译）及 `uwb_qa_native_install_consumer`。

---

## 1. 背景诊断与目标

### 1.1 当前编译痛点量化盘点

对照成熟通信项目（如 **srsRAN Project**、**OpenAirInterface (OAI)**），当前 OpenUWB 的编译慢并非源自代码规模本身，而是源自 **“极度头重脚轻（Header-only 膨胀）+ 串行式传统构建器 + 63 个独立测试二进制”**：

1. **Header-only 严重失衡**：
   - `include/openuwb/` 下共有 **97 个头文件**，而 `lib/` 下仅有 **21 个 `.cc` 源文件**。
   - 体积 **> 30 KB** 的巨型头文件多达 **22 个**，核心代表：
     - `include/openuwb/dsp/rational_resampler_core.h`：**136 KB**（被 **22 处**直接/间接包含）
     - `include/openuwb/phy/hrp/tx/hrp_mod_core.h`：**53 KB**（被 **31 处**直接/间接包含）
     - `include/openuwb/phy/hrp/rx/rx_fused_cmf.h`：**81 KB**、`demod_fec.h`：**73 KB**、`demod_timing.h`：**69 KB**、`demod_softchips.h`：**35 KB**、`rx_sync_opt.h`：**34 KB**
     - `include/openuwb/runtime/scheduled_extractor_core.h`：**49 KB**、`target_lock_core.h`：**39 KB**、`timed_rx_plan.h`：**23 KB**
     - `include/openuwb/dsp/detector_core.h`：**46 KB**
     - `include/openuwb/twr/` 下 `twr_timestamp.h`（111 KB）、`twr_config_effective.h`（87 KB）、`twr_config_validator.h`（82 KB）、`twr_frame.h`（68 KB）等。
   - 任何一次对上述公共头文件的微调，都会触发 `apps/`、63 个 `tests/unit/` 可执行文件及 benchmarks 的**大规模扇出重编**。
2. **传统构建器与无 PCH**：
   - [CMakePresets.json](../../CMakePresets.json) 全部 4 个预设均使用 `"generator": "Unix Makefiles"`，递归 make 的依赖检查与调度开销大，且未启用预编译头（PCH）和编译器缓存（ccache）。
   - 63 个 Boost.Test 单测各自独立编译 `<boost/test/unit_test.hpp>`、STL 与 SIMD 头文件，重复前端词法/语法分析开销巨大。
3. **默认构建范围过重**：
   - 4 个 CMake 预设均默认 `"UWB_BUILD_EXPERIMENTS": "ON"`，每次全量编译都会连带编译历史对比与诊断 benchmark 目标。

### 1.2 两阶段总体目标

| 阶段 | 核心目标 | 预期量化收益 | 风险等级 |
| :--- | :--- | :--- | :---: |
| **阶段 1**<br>Ninja + PCH + 预设分层 | 不改业务 C++ 代码，仅重构 CMake 构建底座与开发预设 | 全量编译明显提速；无改动增量检查降至毫秒级；Boost.Test 单测前端解析开销大幅下降 | **极低**（纯构建配置） |
| **阶段 2**<br>头文件声明/实现分离 | 将高扇出重型算法从 `.h` 下沉到 `lib/**/*.cc` 编译进静态库 | 修改高频算法实现（如重采样、调制、定时/解调）时，**只重编 1–2 个 `.o` + 链接**，彻底消除全树重编与跨 TU SIMD ODR 风险；热路径性能 **零退化** | **中**（需守住内联边界、DAG 与测试契约） |

---

## 2. 技术原理详解

### 2.1 阶段 1 原理：为什么 Ninja + PCH + 分组预设能立竿见影

1. **Ninja 构建图扁平化**：
   - `Unix Makefiles` 为每个子目录生成嵌套 Makefile，依赖扫描和进程调度串行环节多；`Ninja` 使用单一扁平 DAG 描述全仓库所有 `.o` 与链接规则，增量构建时 `stat` 检查可在数十毫秒内完成，且对多核编译槽位的调度利用率显著更高。
2. **预编译头（`target_precompile_headers`）消除重复前端解析**：
   - 本仓库 63 个单测几乎都包含 `<boost/test/unit_test.hpp>`、`<vector>`、`<complex>`、`<algorithm>` 及 `openuwb/phy/hrp/common/profile.h`。
   - **关键约束（遵循 [UwbSimd.cmake](../../cmake/Modules/UwbSimd.cmake)）**：GCC/Clang 要求共用 PCH 的目标必须具备完全一致的关键编译宏与 `-m` 指令集标志。因此不能让普通目标与 `uwb_simd_apply()` 目标混用同一个 PCH，必须拆分为：
     - **通用 QA PCH 原型目标**（无 `-mavx2/-mfma` 强制标志）：供普通单测 `REUSE_FROM`；
     - **SIMD QA PCH 原型目标**（挂接 `uwb_simd_apply()`）：供调用了 `uwb_simd_apply()` 的 9 个单测 `REUSE_FROM`。
3. **开发预设瘦身（配合已落地的 CTest `LABELS`）**：
   - 新增日常开发预设 `dev-avx2`（`Ninja` + `AVX2` + `UWB_BUILD_EXPERIMENTS=OFF`），配合 `ctest -L rx_sic` 只编译和验证核心 20 项，避开历史诊断程序与高并发偶发 flake 目标（`uwb_qa_uwb_fixed_tone_cancel_source.cc`）。

### 2.2 阶段 2 原理：头文件解耦、SIMD ODR 消除与性能零退化防线

1. **从 Header-only 转为“纯接口 `.h` + 编译单元 `.cc`”**：
   - 将结构体/配置/函数声明留在 `include/openuwb/<domain>/*.h`，函数实现体下沉到 `lib/<domain>/*.cc` 并编译进对应静态库（`openuwb_dsp`、`openuwb_phy_hrp`、`openuwb_runtime` 等）。
   - **以 [demod_pipeline.h](../../include/openuwb/phy/hrp/rx/demod_pipeline.h)（15 行）与 [uwb_demod_pipeline.cc](../../lib/phy/hrp/uwb_demod_pipeline.cc) 为现成样板**：消费者只依赖声明，不直接引入内部实现与 `<immintrin.h>`。
2. **消除跨 TU 的 SIMD ODR 隐患**：
   - 目前 [rational_resampler_core.h](../../include/openuwb/dsp/rational_resampler_core.h#L84-L89) 依赖 `uwb_simd_config.h` 强制所有 include 它的下游目标都带 `-mavx2 -mfma`，否则直接 `#error` 报错。
   - 当内核实现下沉到 `lib/dsp/uwb_rational_resampler_core.cc`（属于 `openuwb_dsp`，已调用 `uwb_simd_apply(openuwb_dsp)`）后，外部消费者和 `install_consumer` 无需包含 `<immintrin.h>`，也无需匹配特定 SIMD 编译参数，从根本上消除 ODR 风险。
3. **非模板 vs 模板处理策略**：
   - 经核查，`RationalResamplerLmCore`、`HrpModConfig/modulate_one`、`stage_timing`、`rx_fused_cmf` 等**绝大部分核心类与函数均是非模板类型**，可直接常规拆分为 `.h` 声明 + `.cc` 实现。
   - 极少数内部模板（如 `demod_fec.h:130` 的 `template <typename Metric, typename ExtendPath>` 仅用于 Viterbi 内部扩展）直接下沉到对应 `.cc` 的匿名命名空间中；极少数外部泛型的工具函数（如 `qa_uwb_refactor_contracts.cc` 中调用的 `resample_sc16_window` 模板或微小访问器）保留 `inline`/头文件模板声明。
4. **热路径性能零退化准则**：
   - **保留在 `.h` 的内容**：热路径极短内联函数（如单样点转换、常数计算 `expected_output_length`、结构体 getter）。
   - **移入 `.cc` 的内容**：大粒度批处理入口（`process_sc16`、`process_fc32`、`flush`、`modulate_one`、`modulate_prefix_waveform`、`stage_timing`、`stage_cfo`、`stage_sfd`、`stage_payload`、`rx_fused_cmf`、`TargetLockCore::observe` 等）。在千点/万点级批处理入口处，单次跨 TU 函数调用开销仅数纳秒（`< 0.01%`），不会影响微秒/毫秒级吞吐；若基准测试发现极个别场景因跨 TU 损失优化，可在 Release 预设评估开启 `CMAKE_INTERPROCEDURAL_OPTIMIZATION`（ThinLTO/LTO）。

---

## 3. 分阶段实施规划

### 阶段 1：构建底座升级（Ninja + PCH + ccache + dev-avx2 预设）

#### 具体改动清单
1. **`CMakeLists.txt`**：
   - 升级 `cmake_minimum_required(VERSION 3.20)`；
   - 增加 `UWB_USE_CCACHE`（自动探测 `ccache`）和 `UWB_ENABLE_PCH`（默认 `ON`）选项；
   - 保持 `UWB_BUILD_EXPERIMENTS`、`ENABLE_UHD_BACKEND`、`UWB_SIMD` 语义完全兼容。
2. **`CMakePresets.json`**：
   - 将现有 4 个预设（`native-avx2`、`native-avx512`、`scalar-debug`、`no-uhd`）的 `"generator"` 改为 `"Ninja"`；
   - 新增 `dev-avx2` 配置/构建/测试预设：`Ninja` + `Release` + `AVX2` + `ENABLE_UHD_BACKEND=AUTO` + `UWB_BUILD_EXPERIMENTS=OFF`。
3. **`tests/CMakeLists.txt`**：
   - 当 `UWB_ENABLE_PCH` 为 `ON` 时，定义 `uwb_qa_pch_plain`（空源码静态库承载基础 PCH）与 `uwb_qa_pch_simd`（调用 `uwb_simd_apply()` 承载 SIMD 测试 PCH）；
   - 在 `uwb_register_qa()` 与 `uwb_add_pure_twr_test()` 中按是否受 `uwb_simd_apply()` 控制挂载 `target_precompile_headers(... REUSE_FROM ...)`。
   - **注意**：`uwb_simd_apply()` 的调用位于 `UWB_UNIT_QA_SOURCES` 循环之后（第 203–211 行），因此 PCH 绑定应放在 `uwb_simd_apply()` 之后统一遍历绑定，避免普通 PCH 错绑到 SIMD 目标。
4. **文档同步**：
   - 更新 [原生UHD与C++构建使用说明.md](../../docs/原生UHD与C++构建使用说明.md) 与 [开发状态.md](../../开发状态.md) 中的预设说明（提示首次切换 Ninja 需重建旧 Makefile build 目录）。

---

### 阶段 2：重型头文件拆分（按 2A → 2B → 2C 三批推进）

#### 批次 2A：高扇出基础设施（`openuwb_dsp` + `openuwb_phy_hrp` TX）

1. **拆分 `rational_resampler_core.h`（136 KB）与 `resampler_transposed.h`（23 KB）**：
   - **新建源码**：`lib/dsp/uwb_rational_resampler_core.cc`、`lib/dsp/uwb_resampler_transposed.cc`，注册到 `lib/dsp/CMakeLists.txt` 的 `openuwb_dsp`（继承 `uwb_simd_apply(openuwb_dsp)`）。
   - **头文件清理**：
     - [include/openuwb/dsp/rational_resampler_core.h](../../include/openuwb/dsp/rational_resampler_core.h) 与 [include/openuwb/dsp/resampler_transposed.h](../../include/openuwb/dsp/resampler_transposed.h) 移除 `<immintrin.h>`、`UWB_SIMD_LEVEL` `#error` 守卫及所有非平凡成员函数体；
     - 保留 `AlignedAllocator`、类定义、`ProcessResult`/`ProfileStats` 及短小 `inline` 辅助方法（如 `expected_output_length` 静态公式）。
   - **核查点**：检查 `lib/dsp/uwb_rational_resampler_avx512.cc` 与 `lib/dsp/uwb_resampler_transposed_avx512.cc` 对内部辅助符号的引用（必要时通过 `lib/dsp/detail_*.h` 内部头共享给同库 `.cc`，不暴露到公共头）。

2. **拆分 `hrp_mod_core.h`（53 KB）**：
   - **新建源码**：`lib/phy/hrp/uwb_hrp_mod_core.cc`，注册到 `lib/phy/hrp/CMakeLists.txt` 的 `openuwb_phy_hrp`。
   - **头文件清理**：
     - [include/openuwb/phy/hrp/tx/hrp_mod_core.h](../../include/openuwb/phy/hrp/tx/hrp_mod_core.h) 保留常量（`kPulseTaps` 等）、`HrpModConfig`/`HrpModScratch`/`PulseEvent` 声明及公共 API 声明（`modulate_one`、`modulate_prefix_waveform`、`modulate_prefix_events`、`append_ieee_fcs` 等）；
     - 将 `kStsDrbgBprf` 表、脉冲成形查找表和实现体全部移入 `uwb_hrp_mod_core.cc`。
   - **已知契约警告（关键！）**：
     - [tests/unit/twr/qa_uwb_twr_frame.cc](../../tests/unit/twr/qa_uwb_twr_frame.cc#L1834-L1855) 会直接扫描源码文件中的 `append_ieee_fcs` 字符串，并硬编码检查 `h.file.find("/tx/hrp_mod_core.h") != std::string::npos`。
     - 若将 `append_ieee_fcs` 实体移入 `lib/phy/hrp/uwb_hrp_mod_core.cc`，必须同步更新 `qa_uwb_twr_frame.cc` 该扫描逻辑（将 `hrp_mod_core.h` 中的声明与 `uwb_hrp_mod_core.cc` 中的定义/内部调用合理区分），或把仅 10 行的 `append_ieee_fcs` 保留为 `.h` 内联函数（推荐后者或同步更新 QA 并保持严格生产调用点检查不变）。

#### 批次 2B：PHY RX 解调子阶段（`openuwb_phy_hrp`）

- **现状**：[demod_pipeline.h](../../include/openuwb/phy/hrp/rx/demod_pipeline.h) 仅 15 行；但 [demod_core.h](../../include/openuwb/phy/hrp/rx/demod_core.h) 聚合引入了 `demod_timing.h`（69 KB）、`demod_softchips.h`（35 KB）、`demod_payload.h`（28 KB）、`demod_phr.h`（27 KB）、`demod_sfd.h`（26 KB）、`demod_sync.h`（25 KB）、`demod_fec.h`（73 KB）、`rx_fused_cmf.h`（81 KB）、`rx_sync_opt.h`（34 KB）。
- **拆分落位**（全部注册至 `lib/phy/hrp/CMakeLists.txt` 的 `openuwb_phy_hrp`）：
  1. `lib/phy/hrp/uwb_demod_timing.cc` ← 承接 `demod_timing.h`、`demod_sync.h`、`rx_sync_opt.h`、`rx_matched_filter.h` 实现；
  2. `lib/phy/hrp/uwb_demod_cfo_sfd.cc` ← 承接 `demod_cfo.h`、`demod_sfd.h`、`rx_cfo_opt.h`、`rx_phase_sfo.h` 实现；
  3. `lib/phy/hrp/uwb_demod_softchips_payload.cc` ← 承接 `demod_softchips.h`、`demod_phr.h`、`demod_payload.h`、`rx_fused_cmf.h`、`rx_softchip_opt.h`、`demod_execution.h` 实现；
  4. `lib/phy/hrp/uwb_demod_fec.cc` ← 承接 `demod_fec.h`（包括将第 130 行内部 `template <typename Metric, typename ExtendPath>` 下沉到 `.cc` 匿名命名空间）。
- **兼容规则**：
  - 保留 `demod_core.h` 中所有公开 stage 函数签名（供 `qa_uwb_demod_core.cc`、`qa_uwb_rx_fused_cmf.cc`、`qa_uwb_rx_fused_phr.cc`、`qa_uwb_rx_phase_sfo.cc`、`qa_uwb_cfo_derotate.cc` 等单阶段 QA 调用），仅把函数体移入 `.cc`。
  - 若 `lib/dsp/uwb_demod_sic_avx512.cc` 或 `lib/sic/*.cc` 引用了 RX 头中的内部小工具，将其抽为纯声明/小内联或归入 `openuwb_dsp` 避免违反 `dsp -> phy_hrp` 单向依赖。

#### 批次 2C：Runtime / DSP / TWR 重型头按需下沉

1. **DSP 前端**：`detector_core.h`（46 KB）→ 实现迁入 `lib/dsp/uwb_detector_core.cc`（`openuwb_dsp`）。
2. **Runtime 核心**：
   - `target_lock_core.h`（39 KB）→ `lib/runtime/uwb_target_lock_core.cc`
   - `timed_rx_plan.h`（23 KB）→ `lib/runtime/uwb_timed_rx_plan.cc`
   - `scheduled_extractor_core.h`（49 KB）→ `lib/runtime/uwb_scheduled_extractor_core.cc`
   - `live_packet_processor.h`（16 KB）→ `lib/runtime/uwb_live_packet_processor.cc`（注意 `PacketSlotPool::WorkerLoop::run<ProcessFn>` 是模板方法，保留在 `.h` 中；`PacketProcessor` 构造与 `process()` 等重方法下沉到 `.cc`）。
3. **TWR 核心（可选延后批次）**：如后续进入 TWR 密集开发，再将 `twr_timestamp.h`（111 KB）、`twr_config_effective.h`（87 KB）、`twr_config_validator.h`（82 KB）、`twr_frame.h`（68 KB）的方法体迁入 `lib/twr/`（`openuwb_twr_core`）。

---

## 4. 多 Subagent 并行开发分工与文件所有权

为实现安全、零冲突并行，按 **Wave 1 → Wave 2 → Wave 3** 拓扑调度多个 Subagent（建议使用独立的 `worktree` / `branch` 模式）。

### 4.1 并行拓扑图

```
Wave 1（构建基础设施，单 Subagent 或主协调人先落盘）
  └─ Subagent-BUILD : 阶段 1（CMakeLists.txt + CMakePresets.json + tests/CMakeLists.txt PCH）
         │
         ▼ 合入并确认 Ninja + PCH 全量绿灯（后续所有 Subagent 基于此加速底座开发）

Wave 2（批次 2A，两路严格零文件重叠并行）
  ├─ Subagent-DSP-RESAMP : 拆分 rational_resampler_core.h + resampler_transposed.h -> lib/dsp/
  └─ Subagent-PHY-MOD    : 拆分 hrp_mod_core.h -> lib/phy/hrp/uwb_hrp_mod_core.cc
         │
         ▼ 两路各自通过编译/单测/benchmark 对拍后顺序合入

Wave 3（批次 2B + 2C，三路严格零文件重叠并行）
  ├─ Subagent-PHY-TIMING : 拆分 demod_timing/sync/cfo/sfd -> lib/phy/hrp/uwb_demod_{timing,cfo_sfd}.cc
  ├─ Subagent-PHY-PAY    : 拆分 demod_softchips/phr/payload/fec/fused_cmf -> lib/phy/hrp/uwb_demod_{softchips_payload,fec}.cc
  └─ Subagent-RUNTIME    : 拆分 detector_core.h -> lib/dsp/ 与 target_lock/timed_rx_plan/live_packet_processor -> lib/runtime/
         │
         ▼ 三路通过后合入（注：Wave 3 中两路 PHY 若都要改 lib/phy/hrp/CMakeLists.txt，由协调人合并或仅 1 行冲突手工消解）

Wave 4（集成与全局验收 Subagent）
  └─ Subagent-VERIFY     : 三预设全量构建 + 63项 CTest + install_consumer + 性能 ABBA 对比 + 更新构建/性能报告
```

### 4.2 严格文件所有权矩阵（Wave 2 & Wave 3）

| 目标文件 / 目录 | Wave 1 (`BUILD`) | Wave 2 (`DSP-RESAMP`) | Wave 2 (`PHY-MOD`) | Wave 3 (`PHY-TIMING`) | Wave 3 (`PHY-PAY`) | Wave 3 (`RUNTIME`) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| `CMakeLists.txt`、`CMakePresets.json` | **WRITE** | READ | READ | READ | READ | READ |
| `tests/CMakeLists.txt` | **WRITE** | READ | READ | READ | READ | READ |
| `include/openuwb/dsp/{rational_resampler_core,resampler_transposed}.h` | — | **WRITE** | — | — | — | — |
| `lib/dsp/uwb_{rational_resampler_core,resampler_transposed}.cc` | — | **CREATE** | — | — | — | — |
| `lib/dsp/CMakeLists.txt` | — | **WRITE** | — | — | — | **WRITE** (Wave 3) |
| `include/openuwb/phy/hrp/tx/hrp_mod_core.h` | — | — | **WRITE** | — | READ | — |
| `lib/phy/hrp/uwb_hrp_mod_core.cc` | — | — | **CREATE** | — | — | — |
| `tests/unit/twr/qa_uwb_twr_frame.cc`（仅检查/必要时适配 FCS 扫描） | — | — | **WRITE** | — | — | — |
| `include/openuwb/phy/hrp/rx/{demod_timing,demod_sync,demod_cfo,demod_sfd,rx_sync_opt,rx_cfo_opt,rx_phase_sfo,rx_matched_filter}.h` | — | — | — | **WRITE** | READ | — |
| `lib/phy/hrp/uwb_demod_{timing,cfo_sfd}.cc` | — | — | — | **CREATE** | — | — |
| `include/openuwb/phy/hrp/rx/{demod_softchips,demod_phr,demod_payload,demod_fec,demod_execution,rx_fused_cmf,rx_softchip_opt}.h` | — | — | — | READ | **WRITE** | — |
| `lib/phy/hrp/uwb_demod_{softchips_payload,fec}.cc` | — | — | — | — | **CREATE** | — |
| `lib/phy/hrp/CMakeLists.txt` | — | — | **WRITE** (W2) | **WRITE** (W3) | **WRITE** (W3) | — |
| `include/openuwb/dsp/detector_core.h` + `lib/dsp/uwb_detector_core.cc` | — | — | — | — | — | **WRITE/CREATE** |
| `include/openuwb/runtime/{target_lock_core,timed_rx_plan,scheduled_extractor_core,live_packet_processor}.h` | — | — | — | — | — | **WRITE** |
| `lib/runtime/{uwb_target_lock_core,uwb_timed_rx_plan,uwb_scheduled_extractor_core,uwb_live_packet_processor}.cc` + `lib/runtime/CMakeLists.txt` | — | — | — | — | — | **CREATE/WRITE** |

> **并行防冲突规则**：
> 1. Wave 2 开始前必须先合入 Wave 1，确保所有 Wave 2 Subagent 均在 Ninja + PCH 环境下编译测试；
> 2. Wave 3 中 `PHY-TIMING` 与 `PHY-PAY` 禁止修改对方所有权范围内的 `.h` 文件；若双方需要共享公共小内联函数，仅允许由 `PHY-TIMING` 在 `demod_scratch.h` / `demod_result.h` 以外的独立内部头或协调人合入阶段处理；
> 3. 每个 Subagent 在各自分支完成后，必须在提交前至少跑通 `dev-avx2` 全量编译与相关单测，确保不向主集成分支引入半成品。

---

## 5. 全链路验收标准与测量合同

每阶段/每批次合并后，由协调人或 `Subagent-VERIFY` 执行以下硬门禁（全程 `env -u LD_LIBRARY_PATH`）：

1. **三预设全量构建与单测通过**：
   - `native-avx2`：配置、构建并跑通全部 63 项 CTest（除无安装前缀跳过的 `uwb_qa_native_install_consumer` 外 0 失败）；
   - `no-uhd`：配置、构建并跑通全部 CTest；
   - `native-avx512`：全量编译 0 警告新增/0 错误（特别核验 `openuwb_dsp` 的 AVX-512 `.cc` 与拆出后的基线 `.cc` 无符号冲突或 ISA 越界）；
   - `scalar-debug`：构建并跑通 `-L rx_sic`。
2. **安装消费者独立性验证（阶段 2 必跑）**：
   - 执行临时前缀安装与 [run_native_install_consumer.sh](../../tests/integration/install_consumer/run_native_install_consumer.sh)：
     ```sh
     cmake --install build/native-avx2 --prefix /tmp/openuwb_install_check
     ctest --test-dir build/native-avx2 -R uwb_qa_native_install_consumer --output-on-failure
     ```
     确认安装后的公共头不泄漏构建期内部路径，且链接 `openuwb::*` / `uwb_native` 无缺符号。
3. **编译速度量化对比（记录到验收报告）**：
   - 对比基线（Unix Makefiles）与阶段 1（Ninja + PCH）、阶段 2（头文件拆分后）的以下 4 个指标：
     1. 全量冷编译时间（`cmake --build --preset native-avx2 -j`）；
     2. 修改 `lib/phy/hrp/uwb_demod_timing.cc`（或 `demod_timing.h`）后的增量编译时间与重编目标数（Ninja 输出的 `[N/M]`）；
     3. 修改 `lib/dsp/uwb_rational_resampler_core.cc` 后的增量编译时间与重编目标数；
     4. `dev-avx2` 配合 `-L rx_sic` 的日常编译+测试闭环时间。
4. **运行时性能零退化验证（阶段 2 必跑）**：
   - `benchmark_sc16_burst_core`、`benchmark_hrp_mod`、`openuwb_rx --replay testdata/uwb_radar/tx_737p28.sc16 --sic auto` 串行同机 ABBA 对照重构前基线，确保 `resample_mean`、`sic_mean`、`demod_mean` 无系统性退化。
