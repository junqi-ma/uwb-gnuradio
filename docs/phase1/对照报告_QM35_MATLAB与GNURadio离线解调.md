# QM35 MATLAB 与 GNU Radio 离线解调对照

> 日期：2026-08-12  
> 输入：`F:\UWB基带数据\qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat`

## 1. 对照条件

两边均使用：

- 原始 SC16：737.28 MS/s；
- `t0=3543552`，`T=3686400`（5 ms）；
- 窗口：`pre/capture/post=30000/160000/10000`，共 200000 原生率样本；
- `quality_minorder` 2707 taps，PDU `upfirdn(65,48)` 到 998.4 MS/s；
- QM35：code 9、64 SYNC、4z2 SFD、6.81 Mb/s。

MATLAB 使用 `testdata/analyze_qm35_sc16_matlab.m`；GNU Radio 使用当前保留的
`testdata/offline_qm35_gr_demod_results.csv`（no-lock，98 个已发布结果）。

## 2. 总结果

| 结果 | MATLAB | GNU Radio offline |
|---|---:|---:|
| 调度窗口/结果 | 99/99 | 98/98（末窗未发布） |
| FCS pass | **98** | **97** |
| FCS rate | **99.0%** | **99.0%** |
| timing failed | 0 | 0 |
| payload failed | 0（slot 41 返回坏 FCS） | 1（slot 41） |
| fcs failed | 1 | 0 |

98 个共同 slot 的逐包交集：

- 双方 FCS pass：97；
- MATLAB pass、GNU Radio fail：0；
- 双方 fail：1（slot 41）；
- GNU Radio pass、MATLAB fail：0。

MATLAB 的 98 个成功包均为 54-byte PSDU，payload 完全相同，FCS 接收值与计算值均为
`0x0E22`。

## 3. timing_failed 历史根因与修复

旧默认 `stride=16` 的 GNU Radio 有 13 个 `timing_failed`：

```text
28, 30, 31, 34, 35, 46, 47, 49, 53, 56, 66, 69, 93
```

MATLAB 全部检测到 64 个 SYNC，全部 FCS pass。对这些窗口直接检查 GNU Radio
模板相关可见：

- 真实全采样峰 metric：约 0.279–0.323；
- stride-16 最近采样位置 metric：约 0–0.004；
- stride-16 粗搜的最大点通常落在远处噪声峰，无法进入真实峰附近的 fine refine。

因此失败不是空 slot、能量不足或 schedule 超出窗口，而是 **stride-16 对窄相关峰的
抽样混叠/漏检**。周期 grid probe 只能覆盖接近整数 SYNC 周期的 seed 偏差，不能覆盖
当前随 slot 漂移的任意余数。

已完成同一捕获的 coarse stride 扫描：`1–7`、`10–15` 均为 0 漏检，旧默认 `16` 为
13/99（13.1%），`24` 为 15/99，`32` 为 11/99。漏检率不单调，证明根因是粗采样相位
与窄相关峰的确定性混叠，而非单纯的计算量不足。该捕获上 `stride=14` 为零漏检中最快
（timing 中位 2154 µs，仅比 stride=16 慢约 3.7%）；完整数据见
`testdata/qm35_timing_stride_sweep/README.md`。改默认值前仍应以另一段实测数据复验。

## 4. CFO 修复结果

原实现把由初始 timing 追踪到的 64 个峰直接用于 CFO。异常 slot 中，该追踪会从较晚
SYNC 起步，再把 SFD/数据区误当作尾部 SYNC；即使随后 SFD 已把 `detected_start` 修正，
CFO 仍沿用错误峰列，因而会得到负值或异常大值。

现实现与 MATLAB `compensateCarrierOffset.m` 对齐：

- 先用 bootstrap CFO 完成 SFD 全采样率定位；
- 从 SFD 反推的 preamble origin 重追踪 64 个 SYNC；
- 丢弃前 24 个 SYNC，保留 40 个峰做 `unwrap(angle())` 线性最小二乘拟合；
- 在最终 CFO 补偿后重新执行 SFD，再进入 CIR/软芯片解调。

完整 99-slot 复跑中，98 个共同 slot 的 GNU Radio–MATLAB CFO 差异为：均值
`0.000052 Hz`、RMSE `0.000263 Hz`、最大绝对值 `0.001259 Hz`。原先失败的 slot 12、17、18
分别恢复为 `2415.402`、`1847.424`、`1804.336 Hz`，与 MATLAB 一致并通过 FCS。

slot 41 是唯一双方都失败的包。两边 CFO 接近（GNU +2189 Hz、MATLAB +2189 Hz），
PHR 均得到 54 bytes，但 MATLAB payload 后半段损坏且 FCS `received=0x4E12`、
`calculated=0x529B`。该包更符合真实碰撞/干扰，而不是解调器系统性错误。

## 5. Packet detection 统计（当前默认 stride=14）

完整重跑的前导检测结果（99 个调度窗、98 个已发布结果）如下：

- `preamble_detected=98/98`，`timing_failed=0`；
- `full_64_sync=98/98`，无 partial-SYNC；
- timing metric：`min=0.263`、`P05=0.271`、`median=0.309`、`P95=0.327`、`max=0.336`；
- 98 个共同 slot 的 packet start 与 MATLAB **98/98 完全一致**（最大绝对误差 0 样本）；
- 相对固定调度预测的起点漂移：`offset=-180.8 + 42.176 samples/slot`（998.4 MS/s），
  这说明 no-lock 下时钟偏差被前导检测正确吸收，而不是漏检。

离线脚本现会自动打印这组统计，且独立于 PHR/payload/FCS 成败。

## 6. 分阶段耗时统计（4 workers，no-lock，98 个已发布结果）

`median` 和 `P95` 表示稳定成本；`max` 会受操作系统调度抢占影响。此组 QM35
flowgraph 使用雷达给出的 `t0/T` 做周期截取，**没有执行能量门或 `UwbDetector` 的独立
coarse detector**；因此这两项必须记为 `NOT_RUN`，不能把 scheduled extractor 的时间冒充为
能量检测。

| 阶段 | median (µs/包) | P95 (µs/包) | 计时范围 |
|---|---:|---:|---|
| 能量门 | `NOT_RUN` | `NOT_RUN` | 当前 radar-scheduled 路径未接入 |
| Detector coarse | `NOT_RUN` | `NOT_RUN` | 当前 radar-scheduled 路径未接入 |
| **65/48 上采样 FIR** | **2186** | **2681** | `process+flush`，不含 PMT 创建/发布 |
| packet_detect 总计 | **2214** | **2663** | 下两项的总计（含少量计时量化差） |
| └ stride-14 前导粗搜 | 1436 | 1609 | 1016-sample 相关的粗扫描 |
| └ 细化 + 64-SYNC 跟踪 | 774 | 907 | local refine、grid probe、backtrack、period fit |
| CFO | 312 | 404 | 丢弃前 24 SYNC 后的拟合与补偿 |
| SFD | **4471** | **5211** | 最大的解调阶段 |
| CIR | 978 | 1259 | 包含其下的 soft FIR 等嵌套项 |
| NS-SFD / PHR / payload | 42 / 12 / 55 | 51 / 31 / 109 | |
| worker queue delay | 29 | 61 | |
| **解调总计（不含上采样）** | **8148** | **11837** | |

CIR 内部计时：soft FIR median 为 773 µs、estimate 41 µs、postprocess 153 µs；这些是
CIR 的嵌套明细，不能再与主表相加。上采样已通过 PDU metadata 逐包传递到结果，因此可与
packet detection 在同一份 CSV 中做分位数统计。

此次完整运行的 99 个上采样 PDU FIR 总和为 227.2 ms（98 个已发布结果对应 225.0 ms）；
解调 worker 总和为 851.3 ms，4 workers 的理想下界为 212.8 ms。整条 flowgraph 的
process wall 为 5.37 s（另有 8.55 s SC16 preload）；它仍包含 source、周期截取、PMT、
消息队列和脚本 idle-drain 等待，不能归因给任何单一算法阶段。

## 7. 结论与下一步

原始捕获、窗口长度和 PDU 65/48 路径足以支持 **98/99 FCS**。当前 GNU Radio
在已发布的 98 个结果中为 **97 FCS pass**；唯一损失为：

1. slot 41 的真实 payload 干扰。

**CFO 和前导粗搜索已不再是这批数据的正确率瓶颈。** 下一步应验证 schedule lock 对
长捕获漂移的收敛，以及真实实时源下的端到端吞吐。

## 8. 产物与复现

- MATLAB 驱动：`testdata/analyze_qm35_sc16_matlab.m`
- MATLAB CSV：`testdata/qm35_matlab_compare/matlab_results.csv`
- MATLAB MAT：`testdata/qm35_matlab_compare/matlab_results.mat`
- GNU Radio CSV：`testdata/offline_qm35_gr_demod_results.csv`

从仓库根目录运行：

```bash
/mnt/f/MATLAB/bin/matlab.exe -batch "addpath('testdata'); analyze_qm35_sc16_matlab"
```

## 9. SC16 周期截取与 SC16→FC32 PDU 上采样

新增 `UwbScheduledExtractorSc16` 后，实时友好的数据格式路径为：

```text
SC16 流 → SC16 周期窗口 PDU → 65/48 PDU FIR（窗口内转 FC32）→ FC32 解调 PDU
```

它不在连续 737.28 MS/s 流上创建 FC32 副本；65/48 resampler 接收交织
`s16vector`，只在已截取的窗口内转换为 FC32，输出仍为 FC32。完整 QM35 复跑中，
99 个周期窗均被截取、99 个 PDU 均完成上采样并被 demod 接收；SC16 全流 preload 为
1.46 GiB，而旧 FC32 preload 为 2.93 GiB。已发布的 97 个结果与 MATLAB packet start
**97/97 精确一致**，FCS 为 96 pass，唯一已发布失败仍为 slot 41。

复现 SC16 前端：

```bash
PYTHONPATH=gr-uwb/build/test_modules LD_LIBRARY_PATH=gr-uwb/build/lib \
  python3 testdata/offline_qm35_gr_demod.py --workers=4 --no-lock --sc16-front-end
```

当前 SC16 extractor 有意只实现 radar `t0/T` 的 `EverySlot` 周期截取；不包含能量门、
模板验证或 schedule-lock。它们属于另一路 detector/lock 功能，后续若需 SC16 实时 lock，
应在此类上复用现有 lock tracker，而非回退到全流 FC32。

## 10. SC16 输入处理速度分析（同机、4 workers、no-lock）

为避免把脚本的 3 s idle-drain 等待算作算法性能，离线脚本新增 `active_pipeline_s`：从
flowgraph 启动到 99 个窗口均完成“截取 + 上采样 + demod”的时间。以下为同一 QM35
文件连续复跑的一次对照，结果会受主机调度影响，应以方向与重复测量的中位值为准。

| 指标 | SC16 前端 | FC32 前端 | 结论 |
|---|---:|---:|---|
| 连续流驻留格式 | SC16，1.46 GiB | FC32，2.93 GiB | SC16 减半 |
| SC16→全流 FC32 转换 | 0.00 s | 1.23 s | SC16 消除该步骤 |
| source staging 到 `/dev/shm` | 0.72 s | 1.20 s | 少写一半字节 |
| 上述预处理合计 | **0.72 s** | **2.43 s** | SC16 少 1.71 s |
| `active_pipeline_s` | **0.65 s** | **1.40 s** | **约 2.1×** |
| 活跃吞吐 | **151.2 窗/s** | **70.5 窗/s** | 仍低于 200 窗/s 目标 |
| 65/48 FIR 总计 | 212.1 ms | 211.9 ms | 基本相同 |
| demod median | 7.972 ms | 7.920 ms | 基本相同 |
| 完整脚本总时间（含 idle-drain） | 9.33 s | 11.37 s | SC16 少约 18% |

结论：SC16 路径的收益**不是** FIR 或解调算法变快；FIR 仍在窗口内转 FC32 执行，
demod 也仍是 FC32。收益来自不再把整个 0.5 s / 737.28 MS/s 流扩展为 FC32，因而减少了
内存占用、格式转换、`/dev/shm` 写入量与前端内存带宽压力。当前单机 151 窗/s 尚未达到
5 ms 周期所需的 200 窗/s；下一步应以多次 active-pipeline 复跑的 P50/P95 为准，并隔离
SC16 extractor、PMT 创建和 resampler handler 的独立耗时。

## 11. SC16 端到端耗时占比（99 窗完整 profile）

以下将墙钟拆成互不重叠的阶段，因此百分比可相加为 100%。其中 `idle-drain` 是脚本为
收集尾部 message_debug 结果保留的等待，不是接收机计算，必须单独看待。

| 端到端墙钟阶段 | 时间 | 占总时间 9.39 s |
|---|---:|---:|
| 从 `/mnt/f` 读取原始 SC16 | 4.76 s | **50.7%** |
| 写入 `/dev/shm` 供 GNU Radio source | 0.71 s | 7.6% |
| 活跃流水线：extract → resample → demod 完成 | 0.70 s | 7.5% |
| 脚本 idle-drain / EOS 等待 | 3.22 s | **34.3%** |

活跃流水线内部存在 source、extractor worker、resampler handler 与 4 个 demod worker
并行，故**不能**把下面的累计 CPU/handler 时间相加后当作 0.70 s 的墙钟占比：

| 活跃阶段 profile | 累计时间 | 相对活跃墙钟 | 说明 |
|---|---:|---:|---|
| SC16 extractor stream process | 5.2 ms | 0.7% | 其中 memcpy 3.5 ms |
| SC16 extractor PDU build/publish | 11.7 ms | 1.7% | worker 线程 |
| resampler handler 总计 | 254.8 ms | 36.4% | 串行消息 handler |
| └ SC16→FC32 窗口转换 | 7.9 ms | 1.1% | handler 内部 |
| └ 65/48 FIR | 212.7 ms | 30.4% | handler 内部 |
| └ FC32 PDU build/publish | 31.0 ms | 4.4% | handler 内部 |
| demod worker 计算总和 | 822.1 ms | 117.4% | 4 workers 并行，非墙钟占比 |
| demod 理想 4-worker 下界 | 205.5 ms | 29.4% | `822.1 / 4` |

因此活跃路径的可见串行主项是 **PDU 65/48 resampler handler（约 255 ms）**；其余约
445 ms 的活跃墙钟包含 `/dev/shm` source 吞吐、GNU Radio buffer/消息派发、各阶段重叠与
尾包关键路径。端到端总时间的最大项仍是外部读盘与脚本 drain，并非 UWB 算法本身。

## 12. QM35 每包 SFD 实际相关次数

`stage_sfd` 现逐包输出实际执行的相关计数，而非由搜索范围推算。一次相关定义为一次
对 QM35 `4z2`、**8128-sample** SFD 模板的归一化复数点积；coarse 与 fine 若探测同一
位置仍各算一次，因为代码确实执行了两次点积。

在启用顺序回溯后的 SC16 全量复跑的 98 个已发布解调结果中：

| SFD pass | 搜索窗口总数 | coarse 相关 | fine 相关 | 总相关 | 每包平均 |
|---|---:|---:|---:|---:|---:|
| bootstrap SFD | 185 | 925 | 2768 | 3693 | 37.7 |
| final SFD | 98 | 490 | 1470 | 1960 | 20.0 |
| **合计** | — | — | — | **5653** | **57.7** |

每包总数的分布为：`min=40`、`P05=40`、`median=40`、`P95=143`、`max=193`。
bootstrap 先在 `expected` 位置检查；未过阈值时依次检查向前 1…10 个 symbol，并在首个
合格窗口停止。典型直接命中的包为 bootstrap `5 + 15 = 20` 次、final 20 次，即总共
**40** 次相关；平均仅 **57.7** 次，约对应 `57.7 × 8128 ≈ 469,146` 次复数模板乘加。
若 0…10 均不通过阈值，才会进入历史 32-symbol 全局搜索兜底。

## 13. 初始 preamble 预测的 SFD 起点偏差

为验证 SFD 双 pass 的设计动机，新增逐包字段：初始 preamble/SYNC train 推得的 SFD
`expected`，以及最终 CFO 对齐后的 SFD 起点。差值定义为：

```text
delta = final CFO-aligned SFD start − initial preamble-predicted SFD start
```

SC16 全量复跑的 97 个已发布结果中，`delta` 分布为：

| delta（SYNC symbol；每 symbol = 1016 samples） | 包数 |
|---:|---:|
| 0 | 74 |
| -1 | 1 |
| -2 | 5 |
| -3 | 6 |
| -4 | 5 |
| -5 | 1 |
| -6 | 3 |
| -7 | 1 |
| -8 | 1 |

故 74/97（76.3%）初始预测已准确；**23/97（23.7%）预测偏晚 1–8 个 SYNC**。以样本表示，
分布为 `min=-8128`、`P05=-5283`、`median=0`、`P95=0`、`max=0`、`mean=-911.3`。
负号说明初始 timing 把较晚的 SYNC 当作 preamble 尾部，继而将预期 SFD 放到真实位置之后。

bootstrap SFD 相关定位与最终 CFO 对齐 SFD 的差异为 **0 samples（97/97 exact）**。因此这批
实测数据支持当前机制：用 bootstrap SFD 纠正初始 preamble origin，重跟踪 64 个 SYNC 并
重估 CFO，最后再做单窗口 final SFD；而不是扩大 final SFD 搜索范围。

## 14. SFD 顺序回溯策略（已实现）

候选优化策略为：先在 initial preamble 预测位置匹配；若 SFD metric 未过 `0.3`，每次只
向前回溯一个 SYNC symbol（1016 samples），最大回溯 10 次；首个过阈值候选即用于
bootstrap。实现将该策略设为默认，`sfd_max_backtrack_symbols=10`；若快速路径未命中，
才启用历史 32-window 全局最大值搜索兜底。

在最新 SC16 全量结果的 98 个已发布解调中：

- 最大实际回溯需求为 **8**，故回溯上限 10 覆盖 **98/98**；
- 首个过阈值候选与当前全局最大值选择的正确回溯量 **98/98 一致**；
- `false_early=0`：没有包在真实 SFD 之前出现超过 0.3 的错误候选；
- 平均检查窗口数为 **1.89**（中位数 1，最大 9）。

每个完整 SYNC train 的候选窗口实际执行 20 次 SFD 相关（5 coarse + 15 fine）。因此：

| 策略 | bootstrap 平均相关 | final SFD | 每包总相关 | 相对当前 |
|---|---:|---:|---:|---:|
| 当前：固定扫描 0…32 | 660 | 20 | 680 | — |
| 顺序回溯，最大 10（本数据期望） | 37.8 | 20 | **57.8** | **-91.5%** |
| 顺序回溯，最坏检查 0…10 | 220 | 20 | 240 | -64.7% |

全量回归结果为 98 个已发布解调结果、97 个 FCS 通过、1 个既有 `payload_failed`；与修改前
的解调结果一致。实际平均每包相关数为 **57.7**（而非固定 680），验证了顺序早停已生效。
仍建议在另一段实测数据上持续检查 `false_early`；异常深度 timing seed 则由保留的 fallback
覆盖。
