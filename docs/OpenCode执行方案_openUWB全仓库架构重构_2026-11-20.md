# openUWB 全仓库架构重构执行方案（OpenCode Executable Plan）

> 日期：2026-11-20  
> 基线分支 / 提交：`feature/sic-research` @ `5ce858a`  
> 目标新分支：`refactor/openuwb-arch`  
> 适用对象：OpenCode / 自动化编码代理 / 维护者

---

## 0. 项目定位与已锁定的 6 项架构决策

本次重构将仓库从历史遗留的 `gr-uwb`（单一 RX 接收工具）升级为 **`openUWB`** —— 一个完整的原生 C++17 UWB 基带实现与上层应用平台。后续的 **TX 基带、RX 基带、TWR、Radar/Sensing、Simple RX、Simple TX、SIC** 均作为基于统一基带库的 Runtime Pipeline 与 Application。

以下 6 项决策已锁定，执行过程中**不得擅自更改**：

1. **命名体系采用 `openuwb`**：
   - 顶层项目与 CMake 包名：`OpenUWB`（target 前缀 `openuwb_`）。
   - 公共头文件根路径：`include/openuwb/<domain>/...`。
   - 命名空间：`namespace openuwb`（子域如 `openuwb::support`、`openuwb::dsp`、`openuwb::phy::hrp`、`openuwb::sic`、`openuwb::radio`、`openuwb::runtime`）。
   - 可执行程序统一前缀 `openuwb_*`（例如 `openuwb_rx`、`openuwb_tx`、`openuwb_pipeline_runner`、`openuwb_capture`、`openuwb_twr_demo`），过渡期保留 `uwb_dw3000_live`、`uwb_uhd_sc16_pipeline_runner` 等兼容别名/软链。
2. **去 GNU Radio 化采用“过渡兼容 + 明确 TODO”**：
   - 取消 `gr-uwb/` 嵌套外壳，CMake 顶层移至仓库根目录。
   - 过渡期保留薄转发头文件 `include/gnuradio/uwb/*.h` 与 `namespace gr { namespace uwb { using namespace ::openuwb; } }` 兼容别名，以及 `uwb_core` / `uwb_native` CMake ALIAS target。
   - 所有兼容层文件、CMake ALIAS、以及尚未迁移完的脚本路径必须标注统一标签 `TODO(openuwb-compat-removal): ...`，并在 `开发状态.md` 中登记清除 TODO。
3. **`phy/` 预留 IEEE 802.15.4a / 4z / 4ab HRP 扩展位**：
   - 目录结构采用 `include/openuwb/phy/hrp/{common,tx,rx}/` 与 `lib/phy/hrp/{common,tx,rx}/`。
   - 波形/配置契约层预留标准修订枚举（`HrpStandardRev::Rev4a` / `Rev4z` / `Rev4ab`），当前实现落地 4a/现有 DW1000/DW3000/QM35825 profile，为 4z STS 与 4ab 预留扩展接口与字段，不引入空壳实现。
4. **C++17 与核心库依赖红线不变**：
   - 编译标准锁定 **C++17**（`cxx_std_17`），不升级 C++20。
   - 核心基带库（`support`、`dsp`、`phy`、`sic`、`sensing`、`twr_core`）坚持**零新增重外部依赖**（仅 C++ 标准库 + 可选 `Volk`；日志自研零依赖 `openuwb/support/log.h`）。
   - 单元测试继续沿用 **Boost.Test**（不借重构更换测试框架）。
   - `UHD::UHD` 仅允许被 `openuwb_radio` 可选链接；无 UHD 环境必须可正常构建并运行全部非硬件单测。
5. **App 统一配置与日志接口**：
   - `apps/` 与 `runtime/` 之间预留纯 C++ `Config` 结构体（现阶段由 CLI 参数填充，预留 `load_from_file(...)` 扩展点以便后续无痛接入 JSON/YAML/TOML，当前不引入第三方配置解析库）。
   - 日志系统按 [开发建议_统一日志与调试接口_2026-11-20.md](file:///home/oi/Desktop/uwb-gnuradio/docs/开发建议_统一日志与调试接口_2026-11-20.md) 实现 `openuwb/support/log.h`：
     - 纯 DSP 库（`dsp`/`phy`/`sic`）**禁止直接 I/O**（不写 `stdout`/`stderr`）；
     - 所有诊断日志（`OPENUWB_LOG_ERR/WARN/INFO/DEBUG`）默认输出到 **`stderr`** 或用户注册的 sink；
     - `uwb_dw3000_live` / `openuwb_rx` 的 `Reporter` **独占 `stdout`**，既有事件行格式（`PACKET` / `SUMMARY` / `HEARTBEAT` / `FINAL` / `COST_TOTAL` / `LOCK_STATS` / `RX_GAP` / `SOURCE_ERROR` / `WORKER_ERROR`）**逐字符保持不变**。
6. **分支与清理策略**：
   - 从 `5ce858a` 切出新分支 `refactor/openuwb-arch` 执行本次重构。
   - **本分支不做 SIC 接入 `live_rx`**：仅将现有 `uwb_sic_*` 完整归位到 `openuwb_sic` 库并保证原有 SIC QA 全绿，SIC 接入留到重构合入后的独立分支。
   - **不带入历史/废弃文件**：在新分支清理根目录无用历史文件与废弃构建目录，仅保留有用的核心文档（完整历史可在 `feature/sic-research` 与 `archive/gnuradio-adapter` 追溯）。

---

## 1. 不可违背的工程红线（Hard Constraints）

OpenCode 在每个 Phase 执行期间**必须严格遵守**：

1. **零行为变更原则**：架构重构 commit 不得修改任何 DSP 算法数值逻辑、浮点累加顺序、重采样边界裁剪、分数网格、融合失败回退逻辑。
2. **Fail-Closed 契约**：`timed` 模式无 replay 必须保持 fail-closed；无硬件证据不得宣称硬件/ToA 测距有效。
3. **热路径性能保护**：`uwb_demod_*.h`、`uwb_rx_*.h`、`uwb_rational_resampler_*.h`、`uwb_cir_fir_simd.h` 等 SIMD/内联热路径核**禁止盲目搬入 `.cc`**（防止打断内联与 AVX2/AVX-512 向量化）；仅拆分控制流、配置类、CLI/Reporter 和巨型非热路径实现。
4. **保留 Git 历史**：`git mv` 纯搬迁必须单独 commit，**禁止在同一个 commit 中既移动文件又大改内容**。
5. **洁净环境验证**：所有构建与测试命令必须使用 `env -u LD_LIBRARY_PATH`，禁止把全局 `install` 作为普通测试前置条件，禁止放宽任何 QA 数值容差。
6. **严禁单次会话“大爆炸”**：严格按 **Phase 0 → Phase 1 → Phase 2 → Phase 3 → Phase 4** 逐阶段提交并跑通 Gate，上一 Phase 未全绿前严禁进入下一 Phase。

---

## 2. 目标目录结构与 CMake 依赖 DAG

### 2.1 目标目录树

```text
.                                    # 仓库根目录（取消 gr-uwb/ 外壳）
├── CMakeLists.txt                   # 顶层构建入口（project(OpenUWB LANGUAGES CXX)）
├── CMakePresets.json                # 标准预设：native-avx2 / native-avx512 / scalar-debug / no-uhd
├── .clang-format                    # 从 gr-uwb/gr-uwb/.clang-format 提升至此
├── AGENTS.md
├── LICENSE
├── README.md
├── 开发状态.md
├── cmake/                           # FindVolk、SIMD 探测、OpenUWBConfig.cmake.in
├── include/
│   ├── openuwb/                     # 公共契约头文件（按域分层）
│   │   ├── support/                 # log.h, simd.h, simd_config.h, types.h, checked_math.h, linear_fit.h, sc16_scale.h, defaults.h
│   │   ├── dsp/                     # resampler/, matched_filter.h, cir_fir_simd.h, autocorr_gate.h, detector_core.h,
│   │   │                            #   preamble_verifier_sc16.h, fixed_tone_cancel.h, tone_canceller.h, window_crop.h
│   │   ├── phy/
│   │   │   └── hrp/                 # 预留 802.15.4a / 4z / 4ab HRP
│   │   │       ├── common/          # standard_rev.h, profile.h, codec.h, frame.h
│   │   │       ├── tx/              # mod_core.h (后续扩展 4z/4ab TX 基带)
│   │   │       └── rx/              # demod_pipeline.h, demod_core.h, sync/sfd/timing/cfo/phr/payload/fec/bpm/softchips/result/scratch
│   │   ├── sic/                     # sic_unified.h, discrete_cir_sic.h, preamble_sic.h, fullpacket_sic.h, demod_sic_avx512.h
│   │   ├── sensing/                 # radar_cir_core.h, radar_cir_estimator.h, radar_meta.h, radar_sfd.h, radar_timing.h
│   │   ├── twr/                     # types.h, frame.h, math.h, protocol_time.h, timestamp.h, tof_input.h,
│   │   │                            #   capability_evidence.h, config.h (拆分子头), core.h, phy.h, fake_link.h, test_output.h
│   │   ├── radio/                   # sc16_stream_source.h, fixed_tone_cancel_source.h, uhd_backend_config.h,
│   │   │                            #   uhd_burst_backend.h, fake_burst_backend.h, echo_burst_backend.h, echo_multitx.h, replay_source.h
│   │   └── runtime/                 # config.h (统一 Config 预留), metrics.h, packet_processor.h, acquisition.h,
│   │                                #   target_lock.h, acquisition_tracker.h, timed_rx_plan.h, scheduled_extractor.h,
│   │                                #   scheduled_dump_io.h, echo_scheduler.h, live_pipeline.h, scheduled_pipeline.h
│   └── gnuradio/uwb/                # [过渡兼容层] 薄转发头 + namespace gr::uwb 别名（全量标注 TODO(openuwb-compat-removal)）
├── lib/                             # 与 include/openuwb/ 镜像，仅放库源码（零 qa_*.cc）
│   ├── support/                     # uwb_simd_runtime.cc, uwb_kernel_backend.cc, log.cc (如需非内联 sink)
│   ├── dsp/                         # resampler avx512, fixed_tone_cancel_core.cc
│   ├── phy/hrp/                     # demod_pipeline.cc (及后续 tx/common .cc)
│   ├── sic/                         # sic_unified.cc, discrete_cir_*.cc, preamble_sic_core.cc, fullpacket_sic_core.cc, demod_sic_avx512.cc
│   ├── sensing/                     # (header-only 或 INTERFACE 库，预留 .cc)
│   ├── twr/                         # twr_core.cc, twr_fake_link.cc, twr_phy.cc, twr_phy_impairment.cc
│   ├── radio/                       # uhd_burst_backend.cc, uhd_dry_run.cc, fixed_tone_cancel_source.cc
│   └── runtime/                     # uhd_sc16_pipeline.cc (拆分为 scheduler/worker/source), live_packet_processor.cc 等
├── apps/                            # 极薄应用入口（CLI -> Config -> Runtime Pipeline -> Reporter）
│   ├── openuwb_rx/                  # 原 uwb_dw3000_live.cc 拆分为 main.cc, cli_options.cc, console_reporter.cc
│   ├── openuwb_tx/                  # 预留 Simple TX 骨架（TODO 或最小占位，按需启用）
│   ├── openuwb_pipeline_runner/     # 原 uwb_uhd_sc16_pipeline_runner.cc + pipeline_result_json.h
│   ├── openuwb_capture/             # uwb_uhd_sc16_capture.cc, uwb_offline_postprocess_dump.cc, fixed_tone_cancel_replay.cc
│   └── openuwb_twr/                 # twr_fake_demo.cc, twr_native_phy_demo.cc
├── tests/                           # 独立测试树（从 lib/ 和 apps/ 迁出）
│   ├── unit/                        # 原 gr-uwb/lib/qa_*.cc（按 support/dsp/phy/sic/sensing/twr/radio/runtime 分目录）
│   ├── integration/                 # install_consumer/, qa_uwb_uhd_sc16_pipeline_runner_exit.sh, Python 端到端单测
│   └── benchmarks/                  # 原 gr-uwb/apps/bench_*.cc, benchmark_*.cc, twr_m2a_bench.cc
├── tools/                           # 离线分析、回放、ABBA 基准脚本（rx/, sic/, perf/, performance/, twr/）
│   └── _common/paths.py             # 新增：统一构建产物路径发现，消除各脚本硬编码 gr-uwb/build-*
├── python/openuwb/                  # 原 gr-uwb/python/uwb/（保留 twr_config.py，兼容 import uwb TODO）
├── UWB_demodulation/                # MATLAB 参考实现（保持不动）
├── docs/                            # 仅保留有用核心文档
└── testdata/                        # 本地数据（.gitignore）
```

### 2.2 CMake Target 单向依赖 DAG

| Target | 类型 | 依赖（严格单向） | 禁止依赖 |
| :--- | :--- | :--- | :--- |
| `openuwb_support` | STATIC | `Threads`, 可选 `Volk` | 任何业务库、`UHD` |
| `openuwb_dsp` | STATIC | `openuwb_support` | `phy`, `sic`, `radio`, `runtime`, `UHD` |
| `openuwb_phy_hrp` | STATIC | `openuwb_dsp` | `sic`, `radio`, `runtime`, `UHD` |
| `openuwb_sic` | STATIC | `openuwb_phy_hrp` | `radio`, `runtime`, `UHD` |
| `openuwb_sensing` | INTERFACE / STATIC | `openuwb_phy_hrp` | `radio`, `runtime`, `UHD` |
| `openuwb_twr_core` | STATIC | `openuwb_phy_hrp` | `radio`, `runtime`, `UHD` |
| `openuwb_twr_fake_link` | STATIC | `openuwb_twr_core` | `UHD` |
| `openuwb_twr_phy` | STATIC | `openuwb_twr_core`, `openuwb_dsp` | `UHD` |
| `openuwb_radio` | STATIC | `openuwb_dsp`, 可选 `UHD::UHD` | `runtime`, `apps` |
| `openuwb_runtime` | STATIC | `openuwb_phy_hrp`, `openuwb_sic`, `openuwb_sensing`, `openuwb_radio` | `apps` |
| `uwb_core` *(ALIAS)* | COMPAT | 聚合 `openuwb_support` + `dsp` + `phy_hrp` + `sic` + `sensing` | `TODO(openuwb-compat-removal)` |
| `uwb_native` *(ALIAS)* | COMPAT | 聚合 `openuwb_runtime` + `openuwb_radio` | `TODO(openuwb-compat-removal)` |

---

## 3. 现有头文件与源码归属映射表

OpenCode 在 Phase 2 搬迁文件时**严格按此表执行**：

| 目标域 | 归入的现有头文件（原 `gr-uwb/include/gnuradio/uwb/`） | 归入的现有源码（原 `gr-uwb/lib/`） |
| :--- | :--- | :--- |
| `support/` | `api.h`, `uwb_defaults.h`, `uwb_gr_complex.h`, `uwb_simd.h`, `uwb_simd_config.h.in`, `uwb_kernel_backend.h`, `uwb_linear_fit.h`, `uwb_radar_checked_math.h`, `uwb_sc16_scale.h` + **新增 `log.h`** | `uwb_simd_runtime.cc`, `uwb_kernel_backend.cc` |
| `dsp/` | `uwb_rational_resampler_core.h`, `uwb_rational_resampler_avx512.h`, `uwb_resampler_transposed.h`, `uwb_resample_window.h`, `uwb_window_crop_core.h`, `uwb_cir_fir_simd.h`, `uwb_matched_filter.h`, `uwb_autocorr_gate.h`, `uwb_detector_core.h`, `uwb_preamble_verifier_sc16.h`, `uwb_fixed_tone_cancel_core.h`, `uwb_tone_canceller.h` | `uwb_rational_resampler_avx512.cc`, `uwb_resampler_transposed_avx512.cc`, `uwb_fixed_tone_cancel_core.cc` |
| `phy/hrp/common/` | `uwb_phy_profile.h`, `uwb_hrp_codec.h` + **新增 `standard_rev.h`（预留 4a/4z/4ab）** | （纯头或后续提取） |
| `phy/hrp/tx/` | `uwb_hrp_mod_core.h` | （后续扩展 4z/4ab TX） |
| `phy/hrp/rx/` | 全部 `uwb_demod_*.h`（不含 `uwb_demod_sic_avx512.h`）、全部 `uwb_rx_*.h`、`uwb_softchip_fir_diag.h` | `uwb_demod_pipeline.cc` |
| `sic/` | `uwb_sic_unified.h`, `uwb_discrete_cir_sic.h`, `uwb_preamble_sic_core.h`, `uwb_fullpacket_sic_core.h`, `uwb_demod_sic_avx512.h` | `uwb_sic_unified.cc`, `uwb_discrete_cir_estimator.cc`, `uwb_discrete_cir_reconstruct.cc`, `uwb_preamble_sic_core.cc`, `uwb_fullpacket_sic_core.cc`, `uwb_demod_sic_avx512.cc` |
| `sensing/` | `uwb_radar_cir_core.h`, `uwb_radar_cir_estimator.h`, `uwb_radar_meta_core.h`, `uwb_radar_sfd_core.h`, `uwb_radar_timing_core.h` | — |
| `twr/` | 全部 `uwb_twr_*.h`（其中 `uwb_twr_config.h` 拆分为多文件并保留聚合入口） | `uwb_twr_core.cc`, `uwb_twr_fake_link.cc`, `uwb_twr_phy.cc`, `uwb_twr_phy_impairment.cc`（`twr_m1b_test_support.h` 迁入 `tests/unit/twr/`） |
| `radio/` | `uwb_sc16_stream_source.h`, `uwb_fixed_tone_cancel_source.h`, `uwb_uhd_backend_config.h`, `uwb_uhd_burst_backend.h`, `uwb_fake_burst_backend.h`, `uwb_echo_burst_backend.h`, `uwb_echo_multitx.h` | `uwb_uhd_burst_backend.cc`, `uwb_uhd_dry_run.cc`, `uwb_fixed_tone_cancel_source.cc` |
| `runtime/` | `uwb_uhd_sc16_pipeline.h`, `uwb_target_lock_core.h`, `uwb_qm35_acquisition_tracker.h`, `uwb_timed_rx_plan.h`, `uwb_scheduled_extractor_core.h`, `uwb_scheduled_dump_io.h`, `uwb_echo_scheduler_core.h` + **从 `apps/` 提升**：`uwb_live_packet_processor.h`, `uwb_live_acquisition.h`, `fullpacket_rx_pipeline.h` + **新增 `config.h`, `metrics.h`** | `uwb_uhd_sc16_pipeline.cc`（拆分为 `uhd_device_source.cc` + `scheduled_pipeline.cc`） |

---

## 4. 新分支清理清单（不带入历史/无用文件）

### 4.1 根目录：`git rm` 移除清单（仅在 `refactor/openuwb-arch` 执行）
- `Codex_UWB解调器性能优化记录.md`
- `DW1000真实捕获解调_进度与阻塞分析.md`
- `OPENCODE_START.md`
- `UWB_Preamble_SIC_GNU_Radio_Design.md`
- `UWB模拟域SYNC极性编码干扰抑制分析.md`
- `X410_737.md`
- `开发方案_GNURadio软件升采样_737.28M到998.4M.md`
- `开发方案_UWB实时解调.md`
- `开发需求参考.md`
- `快速脚本.txt`
- `design_tx_pulse.py`（如确需保留可移入 `tools/tx/`，默认不带入）
- `read_uwb_packet.m`（历史 MATLAB 临时脚本，默认不带入）
- `gr-uwb/gr-uwb/.clang-format`（先 `git mv` 到根目录 `.clang-format`，再删空目录 `gr-uwb/gr-uwb/`）

### 4.2 根目录与 `gr-uwb/` 本地未跟踪产物清理（不提交）
- 删除本地临时文件：`all_frames_cir.mat`、`frame_summary.csv`、`live_dw3000_10ms.log`、`probe_tmp.txt`。
- 删除 `gr-uwb/build*` 全部历史构建目录（后续统一使用根目录 `build/<preset>`，并确保 `.gitignore` 覆盖 `/build/`）。

### 4.3 `docs/` 保留的核心文档清单
- **必须保留**：
  - `docs/README.md`（同步更新索引）
  - `docs/架构_解调与窗口执行.md`
  - `docs/原生UHD与C++构建使用说明.md`
  - `docs/开发建议_统一日志与调试接口_2026-11-20.md`
  - `docs/OpenCode执行方案_openUWB全仓库架构重构_2026-11-20.md`（本方案）
  - `docs/DPDK_X410_CG600启用.md`、`docs/本机UHD_DPDK_AVX512问题与绕过.md`（硬件环境排障刚需）
  - `docs/decoder_CFO_SFD码栅格与成形群时延.md`（核心算法原理）
  - `docs/performance/` 下关键基线报告与最新低 SNR/锁定规划
  - `docs/sic/`、`docs/twr/` 中与当前实现对应的核心契约文档
- **移除的过时文档**：
  - `docs/原生接收与GNU_Radio构建使用说明.md`（仅 209B 跳转存根）
  - `docs/archive/`、`docs/phase1/`、`docs/速度测试_2026-11-11.md`、`docs/当前代码路径现状与后续开发基线.md`、`docs/jam_estimator_drop_20260920.md`（历史快照已留在 `feature/sic-research`）

---

## 5. 分阶段执行任务书（Phase 0 → Phase 4）

### Phase 0：建分支、清理历史噪音、固化基线与预设
- **目标**：建立干净的 `refactor/openuwb-arch` 分支与可复现基线，零代码逻辑改动。
- **操作步骤**：
  1. `git checkout -b refactor/openuwb-arch 5ce858a`（先确认当前工作区未提交文档按需带入或另存）。
  2. 按第 4 节执行 `git rm` 清理根目录历史文件和过时文档；将 `gr-uwb/gr-uwb/.clang-format` 移至根 `.clang-format`。
  3. 更新 `.gitignore`，忽略 `/build/` 与 `/gr-uwb/build*/`。
  4. 新增 `CMakePresets.json`（先指向 `gr-uwb` 或待 Phase 3 指向根目录，定义 `native-avx2`、`native-avx512`、`scalar-debug`、`no-uhd`）。
  5. 在当前未改动代码上跑一次完整基线测试并记录通过数与关键回放输出（保存到临时日志，不入库）。
- **Gate 验收**：
  - `env -u LD_LIBRARY_PATH ctest --test-dir <build> --output-on-failure` 全绿；
  - `git status` 干净，无遗留杂物。

### Phase 1：剥离测试/基准 + 落地日志底座 + 提升 Live 核心头
- **目标**：解开“测试混在 `lib/`、基准与核心头混在 `apps/`”的结构死结，落地 `openuwb/support/log.h`。
- **操作步骤**：
  1. **Commit 1A（纯搬迁）**：`git mv` 将 `gr-uwb/lib/qa_*.cc` 与 `twr_m1b_test_support.h` 移至 `tests/unit/`（按域建子目录）；将 `gr-uwb/apps/bench_*.cc`、`benchmark_*.cc`、`twr_m2a_bench.cc` 移至 `tests/benchmarks/`；将 `install_consumer/` 与 `qa_uwb_uhd_sc16_pipeline_runner_exit.sh`、`test_*.py` 移至 `tests/integration/`。
  2. **Commit 1B（更新 CMake）**：调整 CMake 使其从 `tests/` 注册全部 CTest 用例与基准目标，保持测试名与数量不变。
  3. **Commit 1C（日志底座）**：新增零依赖头文件 `uwb_log.h`（支持 `ERR`/`WARN`/`INFO`/`DEBUG`、惰性求值宏、`OPENUWB_LOG_LEVEL` / `UWB_LOG_LEVEL` 环境变量、默认 `stderr` + mutex、支持自定义 sink），并补单测 `qa_openuwb_log.cc`。
  4. **Commit 1D（提升 Live 头 & 拆分 `uwb_dw3000_live.cc`）**：将 `uwb_live_packet_processor.h`、`uwb_live_acquisition.h`、`fullpacket_rx_pipeline.h` 提升到库层；将 `uwb_dw3000_live.cc` 拆为 `cli_options`（填充统一 `LiveRxConfig`）、`console_reporter`（独占 `stdout`）、`timed_replay_source` 与薄 `main.cc`；将 `uwb_uhd_sc16_pipeline.cc` 中的 16 处 `std::cerr` 迁移至 `OPENUWB_LOG_*`。
- **Gate 验收**：
  - 全量 CTest（含新增日志单测）100% 通过；
  - `env -u LD_LIBRARY_PATH python3 tools/rx/qa_live_dw3000_console.py` 通过，确认 `stdout` 文本协议零破坏；
  - `install_consumer` 构建测试通过。

### Phase 2：按域拆分库与头文件 + 预留 HRP 4a/4z/4ab
- **目标**：消除 84 文件平铺与 6818 行巨型头文件，建立第 2.2 节的单向依赖 DAG。
- **操作步骤**：
  1. **Commit 2A（纯搬迁）**：按第 3 节映射表，将头文件与 `.cc` 按域迁入 `include/openuwb/{support,dsp,phy/hrp/{common,tx,rx},sic,sensing,twr,radio,runtime}/` 与 `lib/<domain>/`。
  2. **Commit 2B（过渡兼容头）**：在 `include/gnuradio/uwb/` 下为每个旧头文件生成薄转发头（包含 `#include <openuwb/...>` 与 `// TODO(openuwb-compat-removal): remove legacy forwarding header`）。
  3. **Commit 2C（独立 CMake target DAG）**：在 `lib/<domain>/CMakeLists.txt` 中建立独立 static library，严格按第 2.2 节声明 `target_link_libraries`，并导出 `uwb_core` / `uwb_native` 兼容 ALIAS（标注 `TODO(openuwb-compat-removal)`）。
  4. **Commit 2D（预留标准位与拆巨型文件）**：
     - 在 `include/openuwb/phy/hrp/common/standard_rev.h` 预留 `Rev4a / Rev4z / Rev4ab` 枚举与配置占位；
     - 拆分 `uwb_twr_config.h`（6818 行）为若干职责清晰的子头并保留聚合入口；
     - 拆分 `uwb_uhd_sc16_pipeline.cc`（2387 行）为 `radio/uhd_device_source.cc` 与 `runtime/scheduled_pipeline.cc`；
     - 在 `include/openuwb/runtime/config.h` 预留 `RxConfig` / `TxConfig` / `TwrConfig` / `RadarConfig` 统一结构体基类或命名空间约定。
- **Gate 验收**：
  - 分别使用 `native-avx2`、`native-avx512`、`no-uhd` 三个预设编译并通过全量 CTest；
  - 验证纯基带 target（`openuwb_support`/`dsp`/`phy_hrp`/`sic`）符号表与链接项中**零 `UHD` 依赖、零 `std::cout`/`std::cerr` 调用**。

### Phase 3：顶层去 `gr-uwb/` 外壳 + 命名空间切换 `openuwb`
- **目标**：仓库根成为正式 `OpenUWB` CMake 工程，主命名空间切换为 `openuwb`，保留过渡兼容别名。
- **操作步骤**：
  1. **Commit 3A（顶层提升）**：将 `gr-uwb/CMakeLists.txt`、`gr-uwb/cmake/`、`gr-uwb/apps/`、`gr-uwb/python/` 提升至仓库根目录，清理空 `gr-uwb/` 目录；更新 `CMakePresets.json` 指向根目录。
  2. **Commit 3B（命名空间切换）**：库内部实现迁移至 `namespace openuwb`，并在兼容层提供：
     ```cpp
     // TODO(openuwb-compat-removal): remove legacy gr::uwb alias after downstream migration
     namespace gr { namespace uwb { using namespace ::openuwb; } }
     ```
  3. **Commit 3C（应用目标命名 + 兼容别名）**：注册 `openuwb_rx`、`openuwb_pipeline_runner`、`openuwb_capture`、`openuwb_twr_fake_demo`、`openuwb_twr_native_phy_demo`；通过 CMake post-build symlink 或兼容 wrapper 保留 `uwb_dw3000_live` 与 `uwb_uhd_sc16_pipeline_runner`（标注 `TODO(openuwb-compat-removal)`）。
  4. **Commit 3D（`tools/` 路径统一）**：新增 `tools/_common/paths.py`，批量更新 `tools/` 下 42 个脚本默认指向新构建路径（保留环境变量覆盖和旧路径回退 + TODO）；更新 `tests/integration/install_consumer` 同时验证 `find_package(OpenUWB)` 与兼容目标。
- **Gate 验收**：
  - 根目录 `cmake --preset native-avx2 && cmake --build --preset native-avx2` 与 `env -u LD_LIBRARY_PATH ctest --preset native-avx2 --output-on-failure` 全绿；
  - `tools/rx/qa_live_dw3000_console.py`、`tools/rx/run_distance_replay.py`（关键基线夹具）结果与 Phase 0 基线**逐包一致**；
  - `install_consumer` 端到端测试通过。

### Phase 4：性能回归校验与文档收尾
- **目标**：证明重构零性能退化、零精度退化，收口文档与 TODO。
- **操作步骤**：
  1. 串行执行 ABBA 对比基准（对比 `5ce858a` 基线与当前重构 HEAD 的 `bench_demod_sic_kernels`、`benchmark_sc16_pipeline_steady`、以及 `uwb_dw3000_live` 回放吞吐/单包耗时），记录提交、CPU、ISA、构建参数、worker/queue、夹具哈希。
  2. 更新 `README.md`、`AGENTS.md`、`开发状态.md`、`docs/README.md`、`docs/原生UHD与C++构建使用说明.md`、`docs/架构_解调与窗口执行.md`，集中列出所有 `TODO(openuwb-compat-removal)` 清单与后续 SIC / Simple TX 接入入口。
- **Gate 验收**：
  - 吞吐与延迟无统计显著退化；
  - 全量 CTest、回放 QA、install_consumer 全部通过；
  - 准备好供主分支合并，随后另开分支合入 SIC。

---

## 6. 给 OpenCode 的单 Phase 执行 Prompt 模板

每次只让 OpenCode 执行**一个 Phase**，避免上下文膨胀或大爆炸改动：

```text
请严格按照 `docs/OpenCode执行方案_openUWB全仓库架构重构_2026-11-20.md` 执行 **Phase <N>**。

强制约束：
1. 只执行 Phase <N> 范围内的改动，不要提前做后续 Phase，也不要把 SIC 接入 live_rx。
2. 严禁修改任何 DSP 算法数值逻辑或放宽 QA 容差；保持 Reporter stdout 协议逐字符不变。
3. 文件移动（git mv）与内容修改必须拆成独立 commit，保留 Git rename 历史。
4. 所有兼容层（转发头、namespace gr::uwb 别名、CMake ALIAS、旧二进制别名）必须标注 `TODO(openuwb-compat-removal)`。
5. 所有构建和测试必须使用 `env -u LD_LIBRARY_PATH`。
6. 完成后汇报：每个 commit hash、改动摘要、Gate 验收命令及实际输出（如实报告任何失败或跳过项）。
```
