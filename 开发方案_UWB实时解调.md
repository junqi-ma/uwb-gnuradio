# UWB Detector 后实时解调开发方案

更新时间：2026-08-09

## 1. 目标

在现有 `UwbDetector` 和 `UwbScheduledExtractor` 输出的 packet/window PDU 基础上，尝试实现 QM35825 UWB 包的 CPU 实时解调。

第一阶段只支持一个固定 PHY profile：

```text
设备：QM35825 radar
fs：998.4 MHz
Mean PRF：62.4 MHz BPRF
code_index：9
preamble_repetitions：16（以真实配置为准）
sfd_mode：4z2
data_rate：6.81 Mb/s
输入：已截取约 190～205 us CF32 PDU
```

“实时”分两级：

1. **持续实时**：100/200 packet/s 长时间运行，解调队列不增长、0 job drop；
2. **低延迟实时（进阶）**：200 packet/s 时端到端 p99 latency <5 ms。

第一阶段目标是持续实时。不要在 1 GS/s 连续流上直接执行完整解调；完整流只做 detection/scheduled extraction，解调只处理稀疏 PDU。

---

## 2. 总体架构

```text
X410 continuous IQ
        │
        ├─ UwbDetector / UwbScheduledExtractor
        │             │
        │             ├─ raw PDU → Async PacketWriter（独立保存）
        │             │
        │             └─ raw PDU → UwbRealtimeDemodulator
        │                              │
        │                              ├─ frame PDU
        │                              ├─ diagnostics PDU
        │                              └─ status/result message
        │
        └─ 输入stream始终由前端高速消费
```

关键原则：

- 解调失败不能影响原始 IQ 保存；
- 解调队列满时只丢 demod job，不阻塞 X410/Extractor；
- 每个输入 slot 都产生一个 result：成功、PHR失败、FCS失败、碰撞或队列丢弃；
- 严重干扰窗口仍保留供离线 MATLAB/SIC；
- 不允许解调器回退到全流或全窗口朴素相关。

---

## 3. GNU Radio block 设计

### 3.1 Block 类型

新增：

```text
gr::uwb::UwbRealtimeDemodulator
```

采用 **message/PDU block**：

```text
stream input：0
stream output：0
message input：packet（meta + c32vector）
message output：frame、diagnostics、status
```

理由：packet边界已由Detector/Extractor确定，不需要 `sync_block` 或 `tagged_stream_block`。message handler只做输入校验和有界入队，禁止在handler内同步完成全解调。

实现前必须按 `AGENTS.md`：

1. 搜索本地 GNU Radio message/PDU、异步worker和有界队列范例；
2. 说明消息线程、worker和输出顺序语义；
3. QA先覆盖队列、stop/EOS和消息送达；
4. worker热路径复用scratch，避免逐packet扩容。

### 3.2 建议文件

```text
gr-uwb/include/gnuradio/uwb/uwb_realtime_demodulator.h
gr-uwb/include/gnuradio/uwb/uwb_demod_core.h
gr-uwb/include/gnuradio/uwb/uwb_phy_profile.h
gr-uwb/lib/uwb_realtime_demodulator.cc
gr-uwb/lib/qa_uwb_demod_core.cc
gr-uwb/lib/qa_uwb_realtime_demodulator.cc
gr-uwb/grc/uwb_realtime_demodulator.block.yml
```

`uwb_demod_core` 必须与 GNU Radio 无关，所有算法阶段可单独测试和benchmark。

### 3.3 Worker模型

- 构造期创建2～4个worker；
- 每个worker独占FIR、模板和预分配scratch；
- job保存PDU引用和必要metadata，不重复复制输入；
- 输出默认按完成顺序；如上层要求schedule顺序，再增加小型reorder buffer；
- 队列有固定容量和high-watermark统计；
- stop时排空或明确标记未处理job。

容量关系：

```text
service_rate = worker_count / mean_time_per_packet
```

要求 `service_rate ≥ 2 × packet_rate`，保留至少2倍余量。

---

## 4. 固定 PHY Profile

第一阶段禁止“自动识别所有UWB模式”。使用显式结构：

```text
sample_rate
mean_prf
code_index
preamble_repetitions
sfd_mode
data_rate
max_psdu_bytes
cir_skip_initial_repetitions
cir_repetitions
```

所有参数必须与 `UWB_demodulation/run_decode_uwb_radar.m` 和真实QM35825配置一致。模板、PN序列、SFD和译码表在构造期生成并缓存。

后续再增加DW1000 code-10或其他SFD profile，不能在第一阶段混入。

---

## 5. 解调流水线

严格对照 `UWB_demodulation/decode_uwb.m`，按以下阶段实现：

### Stage 1：输入与局部定时

- 读取PDU predicted/detected start metadata；
- 只在预测起点附近做code-9 coarse-to-fine；
- 跟踪重复SYNC峰；
- 输出精确preamble start、period和metric；
- seeded ROI失败时返回结构化错误，实时路径不允许fallback到整窗口全搜索。

### Stage 2：载波偏移补偿

- 由重复SYNC相位估计CFO；
- 跳过已知接收机启动瞬态的初始repetitions；
- 对活动frame窗口做一次复数旋转；
- 与MATLAB的CFO、峰相位和补偿后IQ对照。

### Stage 3：SFD定时细化

- 固定使用QM35825的4z2 SFD；
- 不运行MATLAB中的多SFD auto-selection；
- 在局部ROI内细化SFD起点与极性。

### Stage 4：CIR与soft chips

- 对照 `estimateCirAndSoftChips`；
- 只用配置的后几次SYNC repetition估计CIR；
- matched filter只计算first-path附近短窗口；
- 输出soft chips、samples/chip、first path和可选CIR diagnostics。

### Stage 5：定位SFD

- 对soft-chip流执行NS-SFD相关；
- 输出SFD end、polarity和metric；
- 失败时停止后续PHR/payload计算。

### Stage 6：BPRF PHR

对照：

```text
helperUWBBPRFDemod
helperUWBConvDec
helperUWBPHRDecode
```

实现：

- BPM位置和BPSK极性软判决；
- constraint-length-3卷积译码；
- 19-bit PHR；
- SECDED纠错/双错检测；
- PSDU length和data rate校验。

PHR失败立即结束，禁止按错误长度继续访问payload。

### Stage 7：Payload与FCS

严格按MATLAB顺序：

```text
BPRF BPM-BPSK demod
→ rate-1/2 convolutional decode
→ RS(63,55) decode
→ PSDU bits/bytes
→ IEEE 802.15.4 CRC16/FCS
```

输出payload bytes、received/calculated FCS和`fcs_pass`。

### Stage 8：结构化结果

任何阶段失败都输出result，不静默丢包。

---

## 6. 输出接口

### `frame` PDU

Payload：`u8vector`，包含解码后的PSDU bytes。

Metadata至少包含：

```text
packet_id / schedule_index
predicted_start_sample
detected_start_sample
preamble_metric
cfo_hz
sfd_start / sfd_metric / polarity
first_path_sample
phr_ok / secded_corrected
psdu_length
fcs_pass
received_fcs / calculated_fcs
demod_latency_us
worker_id
queue_delay_us
```

### `status` message

每个输入job输出一次：

```text
success
invalid_input
timing_failed
cfo_failed
sfd_failed
phr_failed
payload_failed
fcs_failed
queue_full
internal_error
```

### `diagnostics` PDU

通过配置控制：

```text
none：仅frame/status
metrics：CFO、CIR摘要、各阶段metric
full：soft chips、CIR等调试数组
```

生产默认`metrics`。禁止默认每包输出大型CIR/soft-chip数组，以免形成新的消息带宽瓶颈。

---

## 7. MATLAB Golden Reference

新增导出脚本：

```text
testdata/export_realtime_demod_golden.m
testdata/verify_cpp_realtime_demod.m
```

Claude负责从 `decode_uwb.m` 导出逐阶段参考：

```text
window IQ
preamble start/peaks/period/metric
CFO
SFD start/end/polarity
CIR摘要
soft chips
coded PHR / decoded PHR
PSDU length
payload bits/bytes
FCS
```

避免提交大量MAT文件。建议格式：

```text
manifest.csv
*.cfile / *.f32 / *.bin
payload.bin
expected_metadata.csv
```

测试集至少包括：

- 干净QM35825 radar；
- 不同幅度；
- AWGN与多个SNR；
- CFO正负偏移；
- timing offset和chunk/window偏移；
- 简单多径；
- code-10通信不重叠；
- code-9/code-10部分和完全碰撞；
- PHR错误、FCS错误和截断window。

---

## 8. 正确性验收

| 阶段 | 最低要求 |
|---|---|
| Timing | packet/preamble start与MATLAB一致；明确0/1-based换算 |
| CFO | 数值误差在MATLAB定义的容差内，补偿后峰相位一致 |
| SFD | start/end/polarity逐packet一致 |
| CIR | first path一致；归一化CIR误差有明确阈值 |
| Soft chips | 符号、长度和主要metric对齐 |
| PHR | coded/decoded bits逐bit一致，SECDED结果一致 |
| Payload | bits和bytes逐bit/逐byte一致 |
| FCS | received/calculated FCS及pass完全一致 |
| 失败路径 | MATLAB失败样本不能被C++错误标记为成功 |

所有index字段必须注明：原始PDU坐标、cropped坐标或absolute stream坐标。

---

## 9. 性能测试

新增benchmark模式：

```text
demod-core
demod-stage-profile
demod-pdu
scheduled-demod-e2e
```

每次报告：

```text
packets/s
mean / p50 / p95 / p99 latency
各stage耗时
worker utilization
queue mean / high watermark
success / PHR pass / FCS pass
job drop / exception
memory allocation次数（可测时）
```

测试场景：

| 场景 | 要求 |
|---|---|
| 100 packet/s，clean | 持续10分钟，0 drop |
| 200 packet/s，clean | 持续10分钟，0 drop |
| 200 packet/s，2000 comm/s窗口来源 | capture不受阻塞，demod queue稳定 |
| AWGN/CFO/multipath矩阵 | 正确率与MATLAB趋势一致 |
| collision | 输出失败/诊断，不丢原始IQ |
| 慢diagnostics sink | 不反压X410采集支路 |

阶段性能目标：

```text
持续处理能力 ≥400 packet/s
200 packet/s时0 job drop
平均CPU解调时间尽量 <5 ms/packet/worker
p99端到端延迟初期 <20 ms，进阶 <5 ms
```

---

## 10. 优化原则

按profile结果优化，不提前引入GPU。

优先：

1. detector metadata提供seed，消除全窗口搜索；
2. frame crop后再做重计算；
3. 模板、PN、SFD、译码表全部预计算；
4. 每worker复用scratch/FIR；
5. 使用VOLK/SIMD处理相关、能量、复数旋转和短点积；
6. PHR失败早退出；
7. packet级多worker并行，避免单packet内部复杂线程同步；
8. diagnostics按需输出。

禁止：

- 1 GS/s连续流全速率完整相关；
- 每包重建reference/template；
- message handler内同步解调；
- 解调队列反压ScheduledExtractor；
- 未profile就引入GPU/CUDA；
- 为追求速度偏离MATLAB算法却不做逐阶段对照。

MATLAB Coder/MEX可用于验证数值kernel和估算上限，但生产C++不能依赖MATLAB runtime或许可证。

---

## 11. 开发阶段

### R0：可行性与规格冻结

- 确认真实QM35825 PHY参数和窗口长度；
- MATLAB逐阶段计时；
- 导出3～5个clean golden vectors；
- 定义C++ profile、结果schema和容差。

完成标准：同一输入、同一参数、每阶段预期输出明确。

> **✅ R0 完成（2026-08-09）**
> - **PHY profile 修正**：QM35825 为 **64 SYNC**（原 16 是错误配置）。`uwb_phy_profile.h` 冻结 fs=998.4e6 / code 9 / 64 SYNC / 4z2 SFD / 6.81 Mb/s，实测 `chips_per_symbol=508`（1016/2）、`HRPCodes(9)` 为 127 长 Ipatov 序列、`BuildSampledCode()` 生成 508 长 spread 流。
> - **Golden 导出**：`testdata/realtime_demod_golden/`（含 `generate_and_export_golden.m`、manifest.csv、window.cfile、stage_{timing,cfo,sfd,cir,softchips,phr,payload}.mat）。MATLAB `decode_uwb` 对 64-SYNC code-9 信号完整解调：period=1016、64 peaks、metric=1.0、CFO=0、SFD corr=1.0、psdu=127、**FCS pass**（recv=calc=0x584b）。
> - **Result schema**：`uwb_demod_result.h` 定义 7 阶段输出 + `DemodStatus` 枚举 + `DemodTolerance`。
> - **空 core 接口**：`uwb_demod_core.h` 声明 stage_timing/cfo/sfd/cir_softchips/ns_sfd/phr/payload_fcs + `demodulate_one` + worker scratch。
> - **Benchmark 骨架**：`benchmark_detector demod-core|demod-stage-profile|demod-pdu|scheduled-demod-e2e` 报告 profile 与 golden 路径，7 阶段 0/7 实现。
> - **MATLAB 检测器约束**：`detectRepeatedPreamble` 在长静默文件上自适应能量门控失效；golden 用"内存生成 + 立即解调"绕开。C++ R1 用 detector 提供的 timing seed，不依赖 MATLAB 能量门。

### R1：Timing + CFO + SFD

- C++ core；
- 局部seeded检测；
- CFO补偿；
- 4z2 SFD；

- MATLAB逐阶段QA。

完成标准：clean样本全部对齐，失败有明确状态。

### R2：CIR + Soft Chips

- first-path/CIR；
- soft-chip生成；
- 不同SNR/CFO/multipath QA；
- stage profile。

### R3：PHR

- BPRF demod；
- convolutional decode；
- SECDED；
- PSDU length。

完成标准：PHR bits和MATLAB逐bit一致。

### R4：Payload + FCS

- payload BPM-BPSK；
- convolutional + RS；
- CRC16；
- bytes PDU。

完成标准：clean样本payload/FCS完全一致。

### R5：GNU Radio异步block

- message/PDU接口；
- worker池、有界队列、stats；
- stop/drain、异常和queue-full QA；
- GRC/Python bindings。

### R6：端到端实时

```text
X410/ScheduledExtractor
→ Raw Writer
→ Realtime Demodulator
```

- 100/200 packet/s soak；
- latency/queue/drop；
- raw capture不受demod影响；
- MATLAB抽样复核。

### R7：鲁棒性与扩展

- AWGN/CFO/multipath/collision统计；
- code-10/DW1000 profile；
- SC16输入；
- 必要时评估GPU/FPGA；
- 在线SIC单独立项。

---

## 12. 协作方式

Claude 和 Grok 按统一开发流程协作，不预先划分各自负责的代码目录或实现内容。每个阶段都应共同阅读需求、共同确认接口，并由同一套 golden reference、QA 和性能指标验收。

协作流程：

1. 先共同冻结 QM35825 PHY profile、输入窗口格式、结果 schema、坐标定义和容差；
2. 以 MATLAB `decode_uwb.m` 及 `UWB_demodulation/` 作为唯一算法参考；
3. 每完成一个解调阶段，同时补充参考数据、C++实现、QA和benchmark；
4. C++与MATLAB结果不一致时，先定位参数、索引、坐标或算法定义问题，禁止直接放宽容差；
5. 原始IQ保存链路和实时解调链路必须独立，解调队列不得反压采集；
6. 每个阶段使用独立 commit，提交信息明确阶段、测试命令和结果；
7. 阶段结束前共同运行完整 CTest、MATLAB 对照和性能测试，再进入下一阶段。

建议的阶段交接物：

```text
PHY profile
golden vectors / manifest
阶段接口与metadata schema
纯C++ core与GNU Radio block
QA结果
性能结果（latency / queue / CPU / drop）
MATLAB逐阶段对照报告
```

两者可以并行推进算法、C++、测试和文档，但不能形成互不验证的独立实现；所有合入内容都必须通过同一套正确性和实时性验收。

---

## 13. 建议提交顺序

1. `Export QM35825 realtime demod golden vectors`
2. `Define fixed QM35825 PHY profile and result schema`
3. `Implement seeded timing CFO and SFD core`
4. `Estimate CIR and soft chips for scheduled windows`
5. `Decode BPRF PHR with SECDED`
6. `Decode BPRF payload RS and FCS`
7. `Add asynchronous UWB realtime demodulator block`
8. `Benchmark 200 packet per second realtime demodulation`
9. `Validate realtime C++ demodulation against MATLAB`

每个meaningful modification后运行对应QA；每阶段结束运行完整CTest。

---

## 14. 第一阶段明确不做

- 不解调连续1 GS/s全流；
- 不同时支持所有UWB PHY；
- 不实时解调全部2000 communication packet/s；
- 不在线执行完整SIC；
- 不用解调成功与否决定是否保存原始radar slot；
- 不依赖MATLAB runtime作为生产组件；
- 不在正确性对齐前做激进近似。

---

## 15. 最终验收

- QM35825固定profile的timing/CFO/SFD/PHR/payload/FCS与MATLAB逐阶段一致；
- clean测试包payload和FCS完全正确；
- 100/200 packet/s持续运行，队列稳定、0 job drop；
- 每个输入slot都有成功或失败result；
- 解调失败/collision不影响raw IQ保存；
- p95/p99 latency、worker利用率和queue水位可观测；
- 全部CTest通过；
- MATLAB自动验证脚本通过；
- `git diff --check`无输出，不提交临时文件和大体积中间数据。

第一执行步骤：**Claude先完成R0的PHY冻结、stage计时和golden导出；Grok同时只搭建结果schema、空demod core接口和benchmark框架，不在golden确定前实现算法。**
