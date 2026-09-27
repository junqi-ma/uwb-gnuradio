# Wave 0 基线报告（CIR jamming 实时链路性能优化）

- 日期：2026-09-27
- commit：`b897677`（`perf/cir-jamming-opt`，与 `feature/uwb-monostatic-radar` 同点）
- 主机：i7-12700（8P+4E，20 逻辑核；**kernel cmdline `isolcpus=2,3,4` → 用户态仅可见 17 核**；`scaling_governor=performance`；Docker 里 `Hyper-V 全虚拟化` 历史，本次为 native runlevel 5）
- 构建：`CMAKE_BUILD_TYPE=Release`，无 `-march` 后缀（基准 `CMAKE_CXX_FLAGS` 为空），volk machine `avx2_64_mmx`，GR 3.10.12.0
- 负载：桌面（WebKit/Firefox/idle <2%）；loadavg 0.7–1.4；后台 ctest 已结束

## 原始输出

- `radar_pipeline_run1.txt` / `run2.txt`（rounds=16, pdus=64）
- `resampler_probe.txt`（all 300000000 1048576 default 1）
- 均从 `/tmp/opencode/wt-baseline/gr-uwb/build` 运行于 workspace root（testdata 可见）。

## QA 环境注记（采纳开发状态.md / 测试报告_UWB_Radar_PDU速率优化.md 的既有结论）

`uwb_qa_uwb_pdu_rational_resampler.cc` 吞吐阈值 QA **在改动前即失败**（本机 pdus/s≈216 vs 阈值 400，
in_MS/s≈33 vs 阈值 50；独立 2 次运行复现 215.0/216.6/216.2）。历史记录（开发状态.md
"吞吐 575 pdus/s"、docs 评估 §1"575 PDU/s, 2.9×"）来自另一台宿主/配置。因此：

- **所有阈值型吞吐 QA（含 ×1.2 production bar）在本机改动前即不成立**，不能作为阶段回归门。
- 波动确定性 QA：`uwb_benchmark_hrp_mod` 串跑时通过、`ctest -j17` 时偶发失败（>1000 us 门槛）。
  `uwb_qa_uwb_scheduled_extractor.cc(559) released>=20`、`qa_uwb_auto_scheduled_extractor_sc16.cc
  acquire_lock_mixed` 同样仅在 `-j17` 并发时失败，串行即过。
- **阶段验收规则**：排除这 4 个测试后跑 `ctest -j17`（基准中 39/43 通过）；对这 4 个用
  `ctest -j1` 串跑判定（基准中全过，仅 PDU-resampler 吞吐阈值除外，见上）。

## 基线核心数字（两轮均值，us）

| 项 | run1 | run2 | 备注 |
|---|---|---|---|
| SFD search ±64 | 1447.8 | 1449.4 | 占 radar_cir_one 的 ~86% |
| SYNC refine ±8 | 23.3 | 23.0 | |
| estimateCir 54-rep（平均） | 406.6 | 406.1 | 阶段一目标点 |
| radar_cir_one 全 3 步 | 1680.7 | 1678.3 | |
| 65/48 quality RX window | 1450.4 | 1434.0 | |
| 65/48 realtime RX window | 1038.8 | 1035.3 | |
| 65/48 quality full TX+tail | 4178.8 | 4087.3 | 波动大（±1%），后续以多轮均值口径 |
| SC16→FC32 UC200 window | 19.0 | 18.7 | |
| SC16→FC32 CG400 window | 12.3 | 12.2 | |

| PDU/block 级（n=64，run1/run2） | mean | 备注 |
|---|---|---|
| PDU 65/48 FC32 quality | 2005 / 2008 | handler≈1848，fir≈1430–1450，publish≈260 |
| PDU 65/48 SC16 quality | 2008 / — | sc16=25–26 us |
| PDU 65/48 FC32 realtime | 1500 / 1502 | fir≈1067 |
| CirEstimator serial | 2002 / 2016（service 1809/1818） | 54-rep 内核 406 vs block service 1818，差≈出现率/批量/抓取+发布 |
| CirWriter serial (116 taps+jsonl) | 252 / 250 | 阶段二基点 |
| 饱和 e2e UC200 65/48 SC16 | 504.6 / 501.7 pulse/s | drop 全 0，wm 6–7 |
| 饱和 e2e CG400 65/32 SC16 | 497.4 / 500.0 | |

Resampler 连续流（bash 级 NuGet 未涉及；多核 worker 路径 *未* 开）：
- builtin GR：in 38.7 MS/s（default）、kernel macroblock in 37.0/35.5（realtime/quality）
- block 65/48：in 38.2/40.0（quality/realtime），out 51.7/54.1 MS/s
- 本机发挥不出 575 pdus/s 的历史值；PDU 65/48 p50 ≈ 2000 us → 单 worker 上限 ≈ 500 pdus/s
- `benchmark_resampler_65_48` 的 `std::stoull` 崩溃只在 `all` 不给 target 时出现（默认 target=2×10^9 时间过长被我用较小 target 规避）；不影响正式用法（显式带 target）。

## CPU 拓扑注记

`isolcpus=2,3,4 nohz_full=2,3,4 rcu_nocbs=2,3,4` 为本机为 UHD/DPDK work 常驻隔离，
基线/后续性能 run 均在同一 17 核用户域运行，单线程 run 默认稳态（loadavg<1）。

## 供后续 agents 对比的最小回归口径

1. qa（正确性）：`uwb_qa_uwb_radar_cir_estimator`、`_block`、`uwb_qa_uwb_radar_cir_core`、
   `uwb_qa_uwb_cir_writer`、`uwb_qa_uwb_pdu_sc16_scale_65_48`、`uwb_qa_uwb_radar_e2e` 必须 pass。
2. perf 微基准：`benchmark_radar_pipeline`（estimator/estimator-block/writer 段、radar_cir_one 段）
   多轮均值，波动带 run1↔run2 差值（<0.5% 为典型，饱和 e2e ±2%）。
3. 吞吐阈值 4 特殊 QA 不进门槛（文档已知问题，除非另一台机器复测）。
