# UWB 自发自收 Radar 开发报告 —— Step 12：X410 分层验证 CLI（x410_uwb_radar_validate）

日期：2026-09-05
对应需求：docs/phase1/开发计划_UWB自发自收Radar.md §15（Step 12）
参考实现：docs/phase1/开发报告_UWB自发自收Radar_Step9-10.md、
gr-uwb/include/gnuradio/uwb/uwb_echo_burst_backend.h（Step-10 burst 语义词汇）

## 1. 概述

按开发计划 §15 交付独立可执行的 X410 分层验证 CLI：不依赖 Step-11 源码/CMake/头文件，
仅作为"证据采集 + 阶梯门控 + JSONL 报告"的可执行框架，用于后续用户在 X410 上
安排硬件执行。本报告全部结果来自本机实测（Python 3.10.12 / numpy 1.21.5，
本机无 UHD/PyUHD）。

**声明：本机没有 X410 与 UHD 运行时。Step 12 不声称任何硬件层验收；所有硬件层
"未验证"状态见第 6 节。**

## 2. 交付物（仅新增文件，未改动任何既有文件）

| 文件 | 说明 |
| --- | --- |
| gr-uwb/apps/x410_uwb_radar_validate.py | Step-12 CLI（1615 行，`python3 -m py_compile` 通过） |
| gr-uwb/apps/x410_uwb_radar_validate_selftest.py | 标准库 unittest 自测（555 行，33 个用例） |
| docs/phase1/开发报告_UWB自发自收Radar_Step12.md | 本报告 |

不修改：Step-11 后端、lib/CMake、公共头文件、gr-uwb/apps/CMakeLists.txt、开发状态.md。
`git status` 确认仅上述 2 个新 app 文件为未跟踪（??）；`git diff --check` 干净。

## 3. 运行方法

```bash
# dry-run（无设备、无输出目录创建，打印 plan JSON，退出码 0）
python3 gr-uwb/apps/x410_uwb_radar_validate.py --dry-run

# 单阶段（本机无 UHD → 预期退出码 2）
python3 gr-uwb/apps/x410_uwb_radar_validate.py --stage smoke --output <dir>

# 全部自测
python3 gr-uwb/apps/x410_uwb_radar_validate_selftest.py
```

脚本化适配器仅限自测：需环境变量 `UWB_RADAR_VALIDATE_TEST_ADAPTER=1`，
否则 CLI 以 `scripted_adapter_requires_env` 拒绝（退出码 1），evidence_class="test"。

## 4. 实测结果

### 4.1 自测（unittest）

```
Ran 33 tests in 5.0s
OK
```

连续 3 次全绿。覆盖：dry-run 契约、CLI 参数校验（rate 契约锁定、pri 整数 tick 网格、
low-rate/soak PRI 范围、rx 窗口上限、tx 超出下一 PRI、fragment 上限）、阶梯门控
（缺失/不可读/损坏记录、错误前驱、未通过前驱、scripted 证据不得 gate 硬件、等级跳越禁止）、
无 UHD 路径（smoke 退出码 2 + 检查 code=uhd_unavailable + 单一 stage_result/summary +
report.jsonl 与 stdout 逐行一致）、脚本化 7 阶段链、外部适配器子进程协议、JSONL schema。

### 4.2 dry-run plan（实测关键字段）

```
rate.hz                  = 737280000.0     （契约锁定，--rate 750e6 → rate_contract_fixed 拒绝）
windows.pri_ticks        = 3686400         （--pri-s 0.005 s）
windows.rx_window_native = 58593           （pre 1475 + SYNC64 48018 + SFD 6003 + range 74 + tail 3023）
environment.uhd_module_probed = False      （uhd_import = "never performed in dry-run"）
hardware_execution_pending    = True
backend_mode                  = builtin_uhd_python
```

### 4.3 无 UHD 硬件路径（builtin 适配器，实测）

`--stage smoke --output <dir>`：
- 退出码 **2**（environment/device unavailable），无 traceback 进入报告；
- JSONL 行类型 `plan → gate → stage_result → summary`（各一次，单出口 run_stage）；
- stage_result.error 的 checks 携带 `code=uhd_unavailable`（environment_or_device 检查），
  deferred 列表逐字记录生产 CIR 统计/zero_delay_tap 推迟项；
- `report.jsonl` 与 stdout 逐行一致（自测断言）。

### 4.4 阶梯门控（实测拒绝码）

`missing_stage_record`（等级跳越禁止）、`stage_record_unreadable`、`wrong_predecessor_in_record`
（low-rate 用 smoke 记录）、`predecessor_stage_not_passed`、
`predecessor_evidence_not_hardware`（scripted/test 证据不得 gate 硬件阶段）、
`predecessor_backend_mismatch`。退出码 1，`emit_cli_error` 输出 error+summary 两行。

### 4.5 脚本化 7 阶段链（selftest，UWB_RADAR_VALIDATE_TEST_ADAPTER=1）

smoke → single → low-rate（--pri-s 0.1，4 脉冲）→ soak（0.2 s @ 0.01 s）→
calibration（4 脉冲，--write-rx-iq）→ cable（4 脉冲）→ ota（4 脉冲）全部 pass，
evidence_class="test"、hardware_verified=false。校准诊断实测：
每脉冲 metric≈1.0、median_leakage_lag=37 native（=脚本化回环 delay_native_samples 37）、
lag_spread=0，`calibration.json` 写入；ota 记录带 `ota_observation_only=true` 与
"OTA OBSERVATION ONLY: never present…" 警告。故障注入：overflow/short_rx@index0
→ 退出码 3（no_device_errors / rx_samples_exact 失败，fail-fast）。

### 4.6 JSONL 证据 schema（SCHEMA_REPORT = x410_uwb_radar_validate.report.v1）

行类型：`plan` / `gate` / `event`（probe） / `pulse` / `progress` / `stage_result` /
`summary`；每行含 schema、seq、ts_utc、run_id、type、level、stage、backend。
磁盘契约（disk_contract，plan 中逐字声明）：`run.json`（本次补齐写入）、
`report.jsonl`、`rx_iq/pulse_<id>.sc16`（--write-rx-iq 时）、`calibration.json`
（calibration 阶段）；cir.cf32/cir.jsonl 为 Step-11 生产链产物，不由本 CLI 写出。

## 5. 开发过程中修复的关键问题（本机自测驱动）

1. `stage_burst_plan` 缺 smoke 分支 → KeyError('smoke') 内部错误（exit 4）；
2. 自测 `run_scripted` helper 签名错误导致 17 个用例 ValueError；
3. `make_adapter` 给 ExternalAdapter 传 acfg 应为 `info["config"]`（KeyError 'command'）；
4. 校准诊断全窗 argmax 在 SYNC 周期性多峰下选中窗口外最后一处匹配 → 改为在
   [pre−96, pre+sync+96] 区域内求峰（metric 1.0 / lag=37 / spread=0 复现）；
5. 补齐 `run.json` 写入（兑现 plan disk_contract 声明）。

## 6. 未验证层清单（必须如实声明）

本机无 X410/UHD，以下**全部未验证**，硬件执行等待用户安排：

- **X410 上全部 7 个阶段**（smoke/single/low-rate/soak/calibration/cable/ota 的真实硬件执行）；
- `builtin_uhd_python` PyUHD 适配器路径（USRP 打开、737.28 MS/s 精确速率锁定、
  SC16 流、TX/RX 调度、error 映射 uhd_* 全系列）；
- 外部适配器对接真实 Step-11 UhdBurstBackend runner（本机仅验证 JSON 子进程协议与
  测试 stub）；
- 生产 CIR 统计（ok/sfd_failed/timing_failed/cir_failed）与工作域 zero_delay_tap /
  分数延迟：**明确推迟到 Step-11 C++ CIR 链**（每份 staged 报告的 deferred 列表逐字记录）；
- 校准诊断仅是 native pre-CIR 单 SYNC 模板相关性的**诊断性证据**，无生产 CIR 权威；
- OTA 观测永远不得作为 timed-I/O、SFD/CIR 或 RF 动态范围验收
  （每条 ota 记录带 ota_observation_only=true）。

## 7. 遗留与下一步

- 用户在 X410 环境安排硬件执行：smoke 起步，逐步通过阶梯；PyUHD 适配器需要
  `pip install uhd`（或系统 PyUHD 绑定）后才可用；
- 外部适配器与 Step-11 runner 的联调（协议已定，schema x410_uwb_radar_validate.adapter.v1）；
- CIR 生产链（Step-11）接通后，用 `--template` + 校准诊断输出的
  derived_num_delay_samps / derived_cal_delay_native 回填 num_delay_samps / cal_delay。
