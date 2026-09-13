# UWB Radar PDU 模式速率优化验收整改指南

> 面向执行者：OpenCode
>
> 依据：`docs/phase1/开发指南_UWB_Radar_PDU模式速率优化.md`
>
> 审计对象：`docs/phase1/测试报告_UWB_Radar_PDU速率优化.md` 及提交
> `5f3ece0`、`8c7f04e`、`c44a7a6`、`12efa89`
>
> 整改日期：2026-09-13

## 1. 审计结论

当前产物只能判定为：

> **C++ PDU fixed 原型和 200 Hz 短跑 smoke 已通过；完整功能、数值兼容、ROI
> 正确性和长时间 X410 验收未通过。**

不得继续把 M0–M4 整体写为“已完成并验收”。在本指南的阻断项关闭前，状态应调整为：

| 项目 | 当前可接受状态 |
|---|---|
| M0 | 100/200 Hz 基线已采集，完整基线待补 |
| M1 | C++ PDU fixed 实现完成，数值与完整功能待验收 |
| M2 | ROI 机制实现完成，正确性待验收 |
| M3 | 持久 worker 代码与 QA通过，200 Hz 仅短跑通过 |
| M4 | 复制审计完成 |
| 200 Hz | smoke 通过，soak 未通过 |
| 500 Hz | 压力测试，未验收 |

本整改不推翻现有 message/PDU 架构。继续复用：

```text
UwbRealtimeEchoTimer + UhdBurstBackend
  → native SC16 PDU
  → UwbPduRationalResamplerCcf65_32
  → UwbRadarCirEstimator
  → writer / UDP / servo
```

## 2. P0 阻断项：统一 SC16 与旧 Python FC32 的幅度契约

### 2.1 问题

当前 PDU 65/32 对 SC16 输入执行：

```cpp
float(s16_i), float(s16_q)
```

没有乘 `1/32768`。旧 Python UHD 路径输出约 `[-1, 1]` 的归一化 complex64，导致
C++ PDU 路径的：

- raw CIR taps；
- `cir_peak_metric`；
- `raw_l2_norm`；
- `cir.cf32`；
- UDP UCR2 中的 CIR taps 和 peak metric

比旧链约大 32768 倍。`normalized_taps` 形状不变不能证明外部契约兼容，因为 writer
和 UDP 都仍消费 raw taps。

现有 `test_65_32_sc16_input_matches_fc32` 把 FC32 构造成 `float(int16)`，只证明当前
实现自洽，没有对照旧 Python 的 `float(int16)/32768` 语义。

### 2.2 修改要求

先搜索并审计 65/48、65/32、SC16 dump/decode 的既有幅度约定，确定统一契约。雷达
fixed app 的验收目标是保持旧 Python 链数值行为，推荐在 SC16→FC32 转换处使用：

```cpp
constexpr float kSc16ToFc32 = 1.0f / 32768.0f;
fc32 = gr_complex(float(i) * kSc16ToFc32,
                  float(q) * kSc16ToFc32);
```

如果通信链历史上依赖未归一化的整数幅度，不能直接静默改变通用 block；应增加明确
且默认兼容的 scale policy，例如：

```text
Sc16ScalePolicy::RawInteger
Sc16ScalePolicy::UnitRange
```

雷达 `cpp-pdu` 必须显式选择 `UnitRange`。metadata 中记录实际 scale，例如：

```text
input_sample_format=sc16
input_iq_scale=32768
output_sample_format=fc32
output_iq_scale=1
```

禁止只在 `cir_peak_metric` 字段除以 32768，而仍让 raw taps 保持错误量纲；缩放必须
在进入算法前统一，或者对所有 raw 输出采用同一、可证明等价的缩放。

### 2.3 必须新增的 QA

使用同一组 int16 IQ 码字构造两条输入：

```text
A: s16vector(int16 IQ)
B: c32vector(complex(int16 I, int16 Q) / 32768)
```

两条路径通过同一 65/32 和 CIR 后比较：

- resampler 输出逐样本；
- `status`、坐标 metadata；
- raw CIR taps；
- normalized CIR taps；
- `cir_peak_metric`、`raw_l2_norm`；
- `peak_tap`、`cir_origin_sample`。

再增加 writer/UDP consumer QA，证明：

- `cir.cf32` 的 raw taps 与旧链同量纲；
- `cir_norm.cf32` 一致；
- UCR2 payload 与正确的 raw/normalized 选择一致；
- UCR2 header 中 peak metric 与 payload 的量纲一致。

通过标准：逐样本容差沿用现有 resampler/CIR golden，不能只比较 peak tap。

## 3. P0 阻断项：完成 M2 ROI 正确性验收

### 3.1 同输入 full-vs-auto 对照

必须使用完全相同的 native RX PDU，而不是两次独立硬件抓取，分别运行：

```text
--publish-native 0
--publish-native -1
```

至少覆盖：

- preamble 64、128；
- 当前默认 pulse shape；
- calibration delay 的允许最小/默认/最大值；
- CIR 当前 `pre=16/post=100/skip=10`；
- predicted timing；
- require-SFD 模式。

逐帧比较：

- status；
- SFD/sync/CIR origin；
- peak tap；
- raw taps；
- normalized taps；
- metric 和 norm；
- 所有长度与坐标 metadata。

### 3.2 require-SFD 安全策略

当前 `cir_publish_native()` 主要按 predicted-timing CIR 访问范围推导。必须单独覆盖：

```text
sfd_search_margin
sync_refine_margin
resampler FIR delay/flush
calibration 漂移
最后一个 CIR repetition 和 tap window
```

在严格上界推导和 QA 完成前，`--require-sfd --publish-native -1` 必须自动退回 full
window，或者拒绝启动并给出清晰错误。不得继续用未经验证的 auto ROI。

### 3.3 metadata 一致性

继续区分：

- `rx_samples_requested/received`、`sample_count`：物理 UHD 窗；
- `published_samples`：PDU payload 前缀；
- `input_sample_count`：resampler 实际读入点数；
- `full_output_sample_count`：由实际 PDU 输入产生的输出长度。

增加校验，禁止 metadata 宣称的可访问 capture/post 区间超出 payload，而消费者又未
显式按 ROI 规则解释。若保留物理几何字段，应同时输出明确的 payload 有效范围。

## 4. P0 阻断项：补齐 PDU block 的 queue-full 与生命周期 QA

### 4.1 EchoTimer queue-full

现有 QA已覆盖 partial、late、timeout、overflow、broken-chain、stop 和固定 scratch，
但没有独立、可识别的 schedule FIFO `queue_full` 压力用例。

新增测试：

1. 使用可控阻塞的 Fake backend 占住 radio worker；
2. queue capacity 设为 1 或 2；
3. 快速提交超过容量的 schedule PDU；
4. 验证 handler 不阻塞；
5. 验证 `schedules_dropped`、queue HWM 和 `queue_full` status；
6. 释放 backend 后验证已接受任务顺序正确；
7. stop 后无死锁、无悬挂 worker。

### 4.2 end-to-end PDU QA

新增一个不需要硬件的端到端流图测试：

```text
FakeBurstBackend
  → UwbRealtimeEchoTimer
  → PDU 65/32
  → Radar CIR estimator
  → message_debug/writer
```

与旧 Python 语义等价输入或保存的 golden PDU 对照，覆盖正常、ROI、失败 PDU和
stop/drain。不能只分别测试三个 block。

### 4.3 hot-path 表述整改

当前只能证明 RX scratch 固定且不 resize。`publish_burst()` 每帧仍创建 PMT dict、
metadata 和 `s16vector`。报告中的“无热路径分配”改为：

> RX/UHD scratch 在运行期地址与容量不变；每帧 PMT metadata/payload 的必要分配仍
> 存在，并已计入 worker timing。

若要进一步消除分配，另立设计，不得依赖 PMT 未公开内存布局。

## 5. P1：补齐 cpp-pdu 外部功能

### 5.1 scan/manual retune

将 `--echo-backend cpp-pdu` 接入 sweep app。要求：

- retune 请求只在 radio owner worker 的 burst 边界执行；
- Python 控制线程不得直接与在途 UHD I/O 并发调用 tune；
- `freq_hz/freq_offset_hz` 按每个 `pulse_id` 正确记录；
- settling skip/calibration/lock 策略与旧链一致；
- fixed app 行为不变。

QA至少覆盖频率请求顺序、burst 边界应用、失败状态和 metadata；实机覆盖多次升序与
降序 retune。

### 5.2 peak servo

接通 `set_cal_delay_native()`，确保更新在 burst 边界生效。验证：

- skip/cal/lock 帧数；
- 第一个有效峰锁到目标 tap；
- lock 后校准值冻结；
- `calibration_delay_native/work_samples` metadata 与实际生效帧一致；
- retune 与 servo 同时启用时无竞态。

### 5.3 SC16/RX dump

当前 cpp-pdu 会直接禁用 `--dump-rx/--dump-sc16`，不符合外部行为兼容要求。整改可选：

1. 增加异步 PDU dump sink，直接消费 echo 的 SC16 burst PDU；或
2. 将 dump 作为 echo 的第二个消息消费者，使用有界 writer queue。

不得在 radio worker 内写磁盘。验证文件长度、packet offset、metadata 和源 PDU
bit-exact。短窗/ROI dump 与物理全窗 dump 必须用不同模式或字段明确区分。

### 5.4 UDP UCR2

幅度契约修复后进行实测或 loopback：

- 100/200 Hz `udp_sent_ok` 与 CIR ok 数一致；
- 非阻塞发送，EAGAIN 可计数；
- raw/normalized taps 的选择有明确协议定义；
- fixed/scan 的频率字段正确；
- 接收端兼容现有 UCR1/UCR2。

## 6. P1：补齐可信性能基线和埋点

### 6.1 M0 基线补测

在基线 commit `0dbfda5` 上补 Python PDU 500 Hz 压力测试。使用与 cpp-pdu 相同的：

- TX/RX geometry；
- preamble、pulse shape、gain、calibration；
- pulse 数和 PRI；
- `--no-udp`、不 dump；
- CPU governor、系统负载和 UHD/FPGA 版本。

不能用当前修改后的脚本只选择 `--echo-backend python` 代替真正的 baseline commit，
除非先证明 Python 分支从 `0dbfda5` 到当前版本行为和 timing 完全未变。

### 6.2 分位数与 CPU

所有关键阶段至少输出：

```text
count, mean, p50, p95, p99, max
```

包括：

- radio worker（区分等待下一个设备时钟槽与真实 UHD service）；
- PMT build/publish；
- SC16→FC32 convert；
- FIR process+flush；
- resampler handler；
- estimator queue delay/service；
- RX 完成到 CIR 发布的 end-to-end latency。

现有 `worker_us_mean≈PRI` 主要包含等待设备时钟，不能直接当作 UHD I/O 成本。将 timing
拆成：

```text
slot_wait_us
issue_rx_us
tx_send_us
rx_collect_us
pdu_build_us
pdu_publish_us
worker_total_us
```

记录 process CPU、各关键线程 CPU、上下文切换；至少标注 CPU governor 与核心绑定。

### 6.3 队列可观测性

当前 estimator 有 HWM，但 resampler 入口的 GNU Radio message queue 没有长期积压
指标。500 Hz w1 日志中 radio 约 500 Hz，而 CIR 约 315–335 Hz，最后只是等待下游
补算完，不能视为实时吞吐。

至少提供：

- echo schedule FIFO depth/HWM/drop；
- resampler received-emitted backlog 的时间序列；
- estimator depth/HWM/drop；
- writer depth/HWM/drop；
- soak 结束后队列是否回落及运行期间斜率。

如果无法读取 GNU Radio 内部 message queue，使用单调计数差
`echo_published - resampler_completed` 作为 backlog 近似，并定时记录。

## 7. P1：X410 复验矩阵

先关闭 P0，再进行硬件复验。所有用例固定命令、commit、FPGA、UHD、CPU governor，
保存原始 stdout 和机器可读 summary。

| 用例 | 规模 | 通过条件 |
|---|---:|---|
| fixed 100 Hz smoke | 1000 pulses | 0 late/overflow/drop，CIR 连续 |
| fixed 200 Hz smoke | 2000 pulses | 0 late/overflow/drop，backlog 无增长 |
| fixed 200 Hz soak-1 | 60 s | 0 late/overflow/drop，完整写出 |
| fixed 200 Hz soak-2 | 10 min | 0 late/overflow/drop，所有队列无正斜率 |
| Python/C++ A/B | 每项至少 3 轮 | 同配置，报告波动范围而非单次最好值 |
| 500 Hz stress | ≥5 s，多轮 | 定位用途；只有全链长期 ≥500 Hz 才能申请验收 |
| scan/manual | 多次升/降序 retune | metadata、settling、CIR 连续性正确 |
| peak servo | 至少一次完整 lock | 锁定及冻结行为与旧链一致 |
| dump | ≥100 frames | SC16 bit-exact，offset/metadata 正确 |
| UDP | 100/200 Hz | 接收数、序号、频率、taps 和 metric 正确 |

200 Hz 验收不仅看最终 `1000/1000`，还必须满足：

```text
radio deadline 达标
+ downstream sustainable throughput >= 200 PDU/s
+ end-to-end latency p99 有界
+ backlog 无长期正斜率
+ raw/normalized CIR 数值兼容
```

## 8. 证据与版本管理整改

当前 `analysis_outputs/cpp_pdu_ab/*/stdout.log` 在本机存在，但未被 Git 跟踪；提交中
只有 summary JSON。整改后：

- 小型 stdout、命令、环境、summary、统计脚本应可随仓库复核；
- 大型 IQ/CIR 二进制不提交，只记录路径、大小和 SHA-256；
- 每个 case 写 `run.json`，包含完整 argv、return code、wall time、commit、设备和环境；
- summary 必须可由已提交脚本从原始日志重新生成；
- 不手工修改 summary 数字；
- 若 `.gitignore` 排除日志，可改用 `.txt`/压缩摘要，或只提交结构化 event JSONL；
- 不提交工作区内与本整改无关的用户文件。

## 9. 测试执行要求

每个有意义修改后：

```bash
cmake --build gr-uwb/build -j
ctest --test-dir gr-uwb/build --output-on-failure
python3 gr-uwb/apps/test_echo_stream_buffers.py
python3 gr-uwb/apps/test_freq_plan.py
python3 gr-uwb/apps/test_cir_udp_format.py
```

新增定向测试至少包含：

```text
echo_timer_queue_full
sc16_unit_range_matches_python_fc32
cpp_pdu_end_to_end_matches_python_contract
roi_full_vs_auto_predicted
roi_require_sfd_safe_or_fallback
cpp_pdu_udp_payload_scale
cpp_pdu_dump_bitexact
cpp_pdu_retune_boundary
cpp_pdu_peak_servo_lock
```

已知 `uwb_qa_uwb_pdu_rational_resampler.cc` 本机吞吐阈值失败可以继续作为环境项，但
必须记录修改前后实际值；新增功能/数值 QA不得失败。

## 10. 报告修订要求

更新 `docs/phase1/测试报告_UWB_Radar_PDU速率优化.md`：

1. 将 M0、M1、M2、M3 状态按本指南 §1 降级；
2. 删除“payload 契约一致”和“无热路径分配”的错误表述；
3. 不能用“归一化 taps 不受影响”掩盖 raw taps/UDP/writer 量纲变化；
4. 清楚区分 smoke、soak、stress；
5. 清楚区分 schedule wall time 与全链 drain/end-to-end wall time；
6. 报告分位数、CPU、backlog 和多轮波动；
7. 所有未执行项继续列为未验证；
8. 只有真实完成 60 s 和10分钟测试后，才允许写“200 Hz X410 验收通过”。

同时更新 `开发状态.md`，不能把 fixed smoke 扩写成完整 PDU 雷达链验收。

## 11. 建议提交顺序

1. `fix(radar): normalize SC16 PDU input to the Python FC32 contract`
2. `test(radar): compare cpp-pdu and legacy PDU CIR end to end`
3. `fix(radar): validate ROI bounds and require-SFD fallback`
4. `test(radar): cover echo schedule queue overflow and lifecycle`
5. `feat(radar): connect cpp-pdu sweep retune and peak servo`
6. `feat(radar): restore async SC16 dump for cpp-pdu`
7. `test(radar): verify UCR2 values and scale on cpp-pdu`
8. `bench(radar): add percentile CPU and backlog telemetry`
9. `docs(radar): record repeated 200 Hz X410 soak acceptance`

每个提交必须独立构建和测试；不得把算法修正、性能测试数据和无关格式化混在一个
提交中。

## 12. 最终关闭标准

只有以下条件全部满足，整改才可关闭：

- SC16/FC32 同输入端到端 raw/normalized CIR 数值一致；
- full/auto ROI 同输入逐帧一致；require-SFD 有安全上界或 full fallback；
- queue-full、生命周期和端到端 QA通过；
- fixed、scan/manual、servo、dump、UDP 在 cpp-pdu 下均可用；
- 200 Hz 至少完成 60 s 和10分钟 X410 soak；
- 0 late、0 overflow、0 downstream drop、无缺号；
- resampler/estimator/writer backlog 无长期增长；
- end-to-end p99 有界并记录；
- 全量测试除已确认的既有环境阈值外通过；
- 原始证据、复算脚本、summary 和报告一致；
- `开发状态.md` 只陈述实际完成结论。

500 Hz 不属于本轮必须关闭项；如果逐 burst UHD 或全链吞吐仍不能稳定达到 500 Hz，
继续标记为压力测试/后续 M6，不影响 200 Hz 整改验收。
