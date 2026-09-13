# UWB Radar PDU 模式速率优化开发指南

> 面向执行者：OpenCode
>
> 适用分支/系统：X410 CG400，原生 491.52 MS/s，UWB 单站自发自收雷达
>
> 编写日期：2026-09-13

## 1. 任务目标

保留当前雷达链的 **PMT/PDU 异步结构**，优化每脉冲的时间开销和可持续脉冲率。
本任务不再把主链改成 stream/tagged-stream，也不修改通信模式的捕获链。

目标链路：

```text
C++ UwbRealtimeEchoTimer + UhdBurstBackend
  └─ native SC16 RX PDU（有界、异步发布）
       ↓
UwbPduRationalResamplerCcf65_32
  └─ 65/32，持久 worker，可选 radar ROI
       ↓
UwbRadarCirEstimator
       ↓
UwbCirWriter / UDP / peak servo
```

第一验收目标是 **200 Hz 稳定实时且不因下游计算产生 late**。500 Hz 作为压力测试和
瓶颈定位项，不得在逐 burst UHD 路径尚未低于 2 ms 时宣称达标。

## 2. 已知事实与结论

不要重新争论 PDU 与 stream 的抽象优劣，应以现有实测为起点：

| 速率 | 旧 PDU 链 | stream，1 FIR worker | stream，16 FIR workers |
|---|---:|---:|---:|
| 100 Hz | 0 late，实时 | 0 late，实时 | 未测 |
| 200 Hz | 1 late，1.247 s | 196 late，2.253 s | 0 late，1.252 s |
| 500 Hz | 102 late，0.651 s | 871 late，2.303 s | 939 late，2.955 s |

原因判断：

1. PDU 链的队列将 UHD 定时收发与 FIR/CIR 解耦，允许流水重叠。
2. stream 链把逐窗 65/32 FIR 放进 echo 的背压临界路径，单 worker 在 200 Hz
   造成大量 late。
3. PDU 与 stream 使用同一 `RationalResampler65_32Core`；改变载体不会减少 FIR
   运算量。
4. 当前 Python PDU publisher 的 `numpy.tolist() + pmt.init_c32vector()` 每帧约
   1.7–2.9 ms并持有 GIL，是可消除的入口开销。
5. 500 Hz PRI 只有 2 ms，而现有每 burst UHD I/O 路径约 3–7 ms；仅优化 PMT
   封装不能解决这个上限。

完整对比见 `docs/phase1/开发方案_UWB_Radar流式echo与CIR改造.md` §5.1。

## 3. 必须遵守的边界

### 3.1 工程规则

执行前必须阅读：

- `AGENTS.md`
- `开发状态.md`
- `开发需求参考.md`
- `docs/phase1/开发方案_UWB_Radar流式echo与CIR改造.md`
- `docs/phase1/使用说明_X410_CG400自发自收雷达.md`

在新增或改变 GNU Radio block 前：

1. 搜索本地 GNU Radio 源码中相似的 message/PDU block；
2. 先写清 scheduler 语义、线程所有权、队列满和 stop/drain 行为；
3. 本任务应优先复用现有 message-only `gr::block`，不得无理由新建
   `sync_block` 或 `tagged_stream_block`；
4. `work/general_work` 或实时 worker 内不得动态扩容、创建线程或做文件 I/O；
5. 每个有意义的修改都要 build + QA；
6. 不覆盖、不提交工作区内与本任务无关的用户改动和大型测试数据。

### 3.2 功能不变量

- 工作率 CIR 算法仍是 998.4 MS/s，不能降低算法采样率换取表面速度。
- `UwbRadarCirEstimator` 的 predicted timing、CIR tap 定义和坐标系保持不变。
- `pulse_id`、`schedule_index`、`rx_time`、sample-rate、guard、校准延迟等元数据
  必须完整传递。
- fixed、frequency scan/manual retune、peak-align servo、SC16 dump、UDP UCR2 的
  外部行为保持兼容。
- 数值正确性优先于性能；CIR 结果必须与当前 PDU 链逐帧对照。
- 不得把短跑、仿真或 Fake backend 结果写成 X410 soak 验收通过。

## 4. 当前代码入口

优先复用已有实现，不要另写一套 echo scheduler：

- Python 旧链：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py`
  - `TimedUhdEcho`
  - `_recv()` 当前每 fragment 分配 NumPy buffer
  - `_publisher()` 当前执行 `tolist()` 和 `init_c32vector()`
  - `cir_publish_native()` 已计算 CIR 所需的保守发布长度
- 扫频链：`gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py`
  - `SweepTimedUhdEcho`
- 已有 C++ PDU echo：
  - `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`
  - `gr-uwb/lib/uwb_realtime_echo_timer.cc`
  - 它已经是 message-only `gr::block`，使用有界 schedule FIFO 和独立 worker，
    worker 拥有 `EchoGrid` 与 `IRadioBurstBackend`
- 已有 UHD backend：
  - `gr-uwb/include/gnuradio/uwb/uwb_uhd_burst_backend.h`
  - `gr-uwb/lib/uwb_uhd_burst_backend.cc`
- PDU 65/32：
  - `gr-uwb/include/gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_32.h`
  - `gr-uwb/lib/uwb_pdu_rational_resampler_ccf_65_32.cc`
  - `gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h`
- CIR：
  - `gr-uwb/lib/uwb_radar_cir_estimator_block.cc`
- Python bindings：
  - `gr-uwb/python/uwb/bindings/python_bindings.cc`

注意：`UwbRealtimeEchoTimer` 已基本具备所需 PDU 调度语义，不要复制
`UwbEchoTimerStream` 再把输出改回 PDU。应补齐 UHD factory、绑定、radar metadata 和
app 接线。

## 5. 调度与线程模型（实现前必须在代码注释中落实）

正确模型：

```text
Python/control thread
  └─ 启动时提交一次有限或无限 grid schedule

UwbRealtimeEchoTimer dedicated worker（唯一 radio owner）
  ├─ 计算下一未来设备时钟槽；过期槽只跳过，不追赶
  ├─ issue timed RX before timed TX
  ├─ collect RX into fixed SC16 scratch
  ├─ 快速构造并发布 native SC16 PDU
  └─ 立即调度下一 burst，不等待 FIR/CIR/writer

GNU Radio message dispatch / downstream workers
  ├─ PDU 65/32
  ├─ CIR estimator bounded queue
  └─ writer/UDP
```

关键约束：

- 下游积压不能反压或阻塞 radio worker。
- 所有队列必须有界；满时记录明确的 `queue_full/drop`，不得无限增长。
- 实时策略是“宁可明确丢一帧，也不能因为旧帧处理慢而连续错过后续定时槽”。
- radio worker 不做 JSON、磁盘写入、UDP、逐帧 `print(flush=True)` 或 Python
  callback。
- `start/stop/drain` 必须有确定语义；stop 时需要中止在途 UHD I/O 并 join，不能死锁。

## 6. 分阶段实施计划

一次只做一个里程碑。每阶段先留 baseline，再修改、QA、硬件 A/B、记录结果、提交。

### M0：建立可信基线，不改算法

在同一机器、同一 CPU governor、同一 X410 配置下，对当前 Python PDU 链采样：

- 固定参数：CG400、491.52 MS/s、preamble 128、相同 pulse shape、gain 50/60、
  `--no-udp`、不 dump SC16；
- 至少测 100、200、500 Hz；短测统一脉冲数，soak 统一墙钟；
- 至少记录：
  - `late`、burst ok/fail、TX short send、RX timeout/overflow；
  - publisher queue depth/HWM/drop；
  - `get_time/issue/send/recv/enqueue`；
  - `contig/tolist/pmt/pub`；
  - resampler queue/service；
  - estimator queue/HWM/drop/service；
  - process CPU、各线程 CPU、wall time。

将原始日志和汇总 JSON 放进 `/tmp` 或 `analysis_outputs/`；大型抓取不要提交。
任何优化必须使用相同参数做 A/B，不能用不同脉冲数或不同 RX 窗直接比较 wall time。

### M1：用现有 C++ PDU EchoTimer 替换 Python 热路径

#### M1.1 补齐工厂和绑定

为 `UwbRealtimeEchoTimer` 增加/暴露 UHD convenience factory，内部构造
`UhdBurstBackend`。接口风格参考现有 `echo_timer_stream_uhd`，但 block 类型保持
message-only `gr::block`。

要求：

- 仅在 `UWB_HAVE_UHD` 时暴露 UHD factory；UHD-OFF 构建仍通过；
- 保留依赖注入构造方式，QA 继续使用 `FakeBurstBackend`；
- 不把 pybind 的 backend 对象生命周期泄漏到 Python；
- 增加必要的运行时 setter：`set_freq()`、`freq()`、
  `set_cal_delay_native()`，或者明确通过 schedule/control PDU 在 burst 边界更新；
- retune 和 calibration update 必须由 radio owner 线程串行执行，不能与 UHD I/O
  并发调用。

#### M1.2 补齐输出 PDU 契约

`UwbRealtimeEchoTimer::publish_burst()` 当前输出 native SC16 `s16vector`。保持这个
格式，避免先转 fc32；确认 PDU 65/32 已在内部完成 SC16→FC32。

输出 metadata 至少应包含：

```text
status, pulse_id, packet_id, schedule_index
sample_rate=491520000, sample_format=sc16
rx_time_ticks（或兼容 rx_time）
window_start_sample
pre_guard_samples, capture_samples, post_guard_samples
sample_count, rx_capture_samples
sync_repetitions, sfd_mode, code_index
calibration_delay_native_samples, num_delay_samps
tx_packet_samples, source
freq_hz, freq_offset_hz（扫频时）
```

复用 `uwb_radar_pdu_meta` 的白名单/安全整数函数，不要在 echo、resampler、CIR 三处
各写一套坐标映射。

#### M1.3 app 接线

在原 PDU app 中增加显式选择，例如：

```text
--echo-backend python|cpp-pdu
```

开发期默认保持已验证的 `python`，硬件 A/B 通过后再讨论是否将默认值改为
`cpp-pdu`。不要删除 Python fallback，也不要影响 stream app。

新链：

```text
启动时生成 TX native SC16 schedule PDU
  → realtime_echo_timer_uhd.schedule
  → burst
  → pdu_rational_resampler_ccf_65_32.packet
  → radar_cir_estimator.rx
```

fixed、sweep/manual、servo 要分开验收。先完成 fixed；随后复用相同 C++ block 的控制
入口接入 scan 和 peak servo，不在 Python 中重建 UHD 对象。

#### M1.4 M1 验收

- Fake backend：每个有效 index 恰好一个 burst PDU，顺序正确；
- queue full、late skip、partial send/recv、timeout、stop during I/O 均有 QA；
- RX scratch 地址与 capacity 在运行期间不变；
- Python 热路径不再出现 `rx.tolist()` / `pmt.init_c32vector()`；
- 相同 native 输入下，C++ PDU 与旧 Python PDU 的 payload/metadata 对照通过；
- X410 fixed 100/200 Hz A/B，CIR 数量、`peak_tap`、metric 与旧链一致；
- 只有实测证明 `cpp-pdu` 更稳后才可改变默认值。

### M2：缩短“发布/处理窗口”，先不要缩短物理 RX

当前 `cir_publish_native()` 已提供保守长度。先在 C++ echo 输出端实现等价的
`publish_native`/ROI 截断：UHD 仍接收完整 `rx_samples`，但 PDU 只发布 CIR 会读取的
前缀。

必须区分两个指标：

- `rx_samples_requested/received`：物理 UHD 窗；
- `published_samples`：PDU payload 窗。

不要把只减少 PDU 长度写成 UHD I/O 优化。

ROI 上界必须由 CIR 实际访问几何推导，至少覆盖：

```text
predicted origin
+ 最后一个参与 CIR 的 repetition offset
+ SPS
+ cir_pre + cir_post
+ sync refine / SFD search margin（若启用）
+ resampler FIR group-delay/flush guard
+ calibration 最大允许漂移
```

现有 `cir_publish_native()` 只按当前 predicted-timing CIR 读取范围计算。若
`--require-sfd` 或搜索 margin 会访问更后样本，必须单独推导，不能盲目复用默认值。

提供三种模式便于回归：

```text
--publish-native 0     # 发布完整物理 RX 窗，golden/fallback
--publish-native -1    # 自动安全 ROI
--publish-native N     # 显式前缀
```

M2 验收：在现有硬件抓取或固定离线 PDU 上，full 与 auto ROI 的 CIR 状态、peak tap、
复数 taps 逐帧对照；容差沿用当前 estimator QA，不得只比较是否 `status=ok`。

### M3：给 PDU 65/32 增加持久多 worker

stream block 已暴露 `num_workers`，PDU block 也应提供相同能力：

```cpp
make(..., int num_workers = 1)
set_num_workers(int n)
num_workers() const
```

实现要求：

- 复用 `RationalResampler65_32Core::set_num_workers()`；
- worker pool 只在构造或显式配置时创建，绝不能每 PDU 创建线程；
- 默认先保持 1，A/B 后再决定 app 默认 2 或 4；
- 同一个 core 实例不能被多个 message handler 并发调用；若需要包级并行，必须是
  多个独立 core/context，而不是给非线程安全状态加一把大锁；
- 记录 `num_workers`、kernel、taps/profile、input/output samples、service time；
- 1/2/4/8/16 workers 做 sweep，同时观察 UHD worker CPU 和 late，不能只看 FIR
  benchmark。

M3 验收：

- 1 worker 与旧 PDU resampler 位一致或满足已有逐样本容差；
- 1/4/8 worker 输出逐样本一致；
- malformed/oversize PDU、stop/drain、重复 start/stop QA 通过；
- 200 Hz 下选择“达到 0 late 的最小 worker 数”，避免无意义抢占 UHD 核；
- 500 Hz 若仍失败，明确归因并保留数据，不调低正确性门槛。

### M4：消除可证明的 PDU 复制

先用 profiling 画清每个大数组的所有权与复制次数，再改代码。重点检查：

```text
UHD SC16 scratch
  → output s16vector
  → PDU resampler input scratch
  → resampler output scratch
  → output c32vector
  → estimator queue/job scratch
```

优化顺序：

1. 若输入 PMT uniform vector 已连续且生命周期覆盖 handler，直接读取其 const data，
   不复制到 `d_input_scratch_`；
2. 尝试让 resampler 直接写最终消息载荷前，先确认本机 GNU Radio/PMT API 是否支持
   安全、稳定的可写 uniform-vector storage；
3. 若 API 不能保证所有权和写入安全，保留一次明确复制，不得用悬空指针或依赖
   未公开实现；
4. 不要轻易改为自定义 blob 契约；若确有必要，必须同时给所有消费者加版本、长度、
   dtype、endianness 校验和兼容路径。

这一阶段以 profile 证明为准。单帧只节省几十微秒时，不应为了“零拷贝”破坏 PDU
兼容性。

### M5（可选）：融合 radar ROI 重采样与 CIR

只有 M1–M4 后 200 Hz 仍无足够余量，才实现专用 message/PDU block：

```text
native SC16 radar PDU
  → validate metadata
  → crop native ROI
  → 65/32 process+flush
  → predicted-timing CIR
  → 只发布小 CIR PDU
```

目的不是改变算法，而是消除约 222k work samples 的中间 PDU和跨 block 内存往返。
必须复用现有 resampler core、radar CIR core 和 metadata 映射，禁止复制算法。

该 block 类型仍应为 message-only `gr::block`，handler 只校验并进入有界队列，
2–4 个持久 worker 各自拥有独立 scratch/core。结果需要按 `pulse_id` 定义是否保持
有序；若允许乱序，writer、UDP、servo 都必须明确支持，默认建议有序发布。

### M6（单独课题）：物理 RX 缩窗或批量/连续 RX

500 Hz 若被每 burst UHD 3–7 ms 限制，需要优化 radio transaction，而不是继续调整
PMT：

- 先验证能否缩短 `rx_samples`，且仍覆盖 TX burst、目标最大距离、硬件延迟和保护区；
- 评估预排多个 timed TX/RX command；
- 评估连续 RX 后按设备时间/已知 PRI 截窗；
- 必须保留 overflow、时间戳连续性、stop/reacquire 测试。

M6 不得夹在 M1–M5 中顺手修改；它改变硬件调度语义，应另立设计和验收报告。

## 7. 性能埋点规范

所有时间使用单调时钟。汇总至少提供 count、mean、p50、p95、p99、max，不要只给
平均值。

建议每帧时间线：

```text
slot_selected
rx_command_issued
tx_send_begin/end
rx_collect_begin/end
pdu_build_begin/end
pdu_published
resample_start/end
estimator_enqueue/start/end
writer/udp enqueue/end
```

必备计数器：

```text
scheduled_slots, late_slot_skips
bursts_ok, bursts_failed
tx_short_send/reissues
rx_partial/reissues/timeout/overflow
echo_queue_depth/hwm/drop
published_samples_total
resampler_received/emitted/dropped, service_us
estimator_received/done/fail/drop, queue_depth/hwm, service_us
writer_ok/fail, udp_sent/eagain
```

区分三种性能指标：

1. **radio deadline**：是否按设备时钟完成下一次调度；
2. **processing throughput**：下游长期处理速率是否不低于输入脉冲率；
3. **end-to-end latency**：从 RX 完成到 CIR 可用的 p50/p99。

队列暂时吸收积压不等于吞吐达标。soak 结束时队列应回落且无长期正斜率。

## 8. QA 与验证矩阵

### 8.1 无硬件 QA

- FakeBurstBackend 正常、partial、timeout、late、overflow、stop；
- 队列容量边界与 drop 计数；
- scratch 地址不变、无 hot-path resize；
- metadata 类型、必需字段、SC16 IQ pair 长度；
- PDU resampler 1/4/8 worker 数值一致；
- full PDU 与 ROI PDU 的 CIR 逐帧一致；
- fixed/scan/servo 控制并发测试；
- ASan/TSan 可运行时检查 stop/retune/worker 生命周期。

### 8.2 算法对照

- 使用 `testdata/` 中已知 UWB 信号；
- 对照当前 PDU golden 与 `UWB_demodulation/` MATLAB 实现；
- 比较 packet/cir origin、peak tap、每个复数 CIR tap；
- 如果 ROI 改变导致坐标偏移，必须修正 metadata 映射，不得在验收脚本里硬编码补偿。

### 8.3 X410 验收

固定同一套硬件参数，至少执行：

| 用例 | 时长/规模 | 通过条件 |
|---|---:|---|
| 100 Hz smoke | ≥1000 pulses | 0 late，0 estimator drop，CIR 连续 |
| 200 Hz smoke | ≥2000 pulses | 0 late，0持续积压，CIR 无缺号 |
| 200 Hz soak | 先 60 s，后 10 min | 0 overflow；队列无正斜率；结果完整 |
| 500 Hz stress | ≥5 s | 只用于定位；如失败，保留 stage timing 与错误分类 |
| scan/manual | 覆盖多次 retune | 频率 metadata 正确，settling 策略不回退 |
| peak servo | 至少一次 lock | 锁定行为和旧链一致，无线程竞态 |

“0 late”指除明确记录的初始 arm/retune settling 策略外，不因 host processing 跳过
正常调度槽。验收报告必须列出实际 FPGA image、UHD 版本、CPU governor、核心绑定、
命令行和 commit。

## 9. 构建、测试与提交纪律

每个里程碑至少执行：

```bash
cmake --build gr-uwb/build -j
ctest --test-dir gr-uwb/build --output-on-failure
python3 gr-uwb/apps/test_echo_stream_buffers.py
python3 gr-uwb/apps/test_freq_plan.py
python3 gr-uwb/apps/test_cir_udp_format.py
```

实际命令应以当前 build tree 和 `开发状态.md` 为准。若全量 CTest 中已知环境相关
吞吐阈值失败，应同时：

1. 单独运行该测试并保存输出；
2. 证明失败在修改前也存在；
3. 运行所有本任务相关定向 QA；
4. 不得简单写成“全部通过”。

建议提交边界：

1. `test/bench: establish radar PDU throughput baseline`
2. `feat(radar): expose C++ PDU echo timer UHD factory`
3. `perf(radar): publish bounded native ROI PDUs`
4. `perf(radar): add persistent workers to PDU 65/32 resampler`
5. `perf(radar): remove measured PDU copies`（只有数据证明后）
6. `docs(radar): record X410 PDU throughput acceptance`

每个提交必须可独立构建，功能改动与大规模格式化不得混在一起。

## 10. 停止条件与禁止的捷径

出现以下情况应停止该阶段并报告，不要继续堆优化：

- CIR 数值、peak tap 或 metadata 坐标与 golden 不一致；
- radio worker 被下游调用阻塞；
- 队列无界增长；
- 需要依赖 PMT 未公开内存布局才能“零拷贝”；
- worker 数增加后 UHD late/underflow 反而增加；
- 500 Hz 瓶颈已经明确在 UHD transaction，却仍只调整 FIR/PMT。

禁止：

- 用更短 preamble、减少 CIR taps 或关闭必要检查伪造性能提升；
- 用更大无界队列掩盖处理速率不足；
- 只报告平均 service time，不报告 p99、late、drop 和队列趋势；
- 把 stream 测试结果当成 PDU 优化结果；
- 未做 X410 soak 就写“硬件验收通过”；
- 修改/提交与本任务无关的用户工作区文件。

## 11. 期望交付物

OpenCode 最终应交付：

1. 保留 PDU 架构的 C++ UHD EchoTimer app 接线；
2. PDU 65/32 持久 worker 配置与绑定；
3. 安全的 radar ROI 发布模式及 full-window fallback；
4. 新增/更新的 C++ QA、Python QA 和 benchmark 脚本；
5. `analysis_outputs/` 下可复算的 A/B 指标汇总（不含大型 IQ）；
6. 一份 X410 测试报告，逐项区分“已验证”“未验证”“环境阻塞”；
7. 更新 `开发状态.md`，但只有真实完成的里程碑才标为完成。

推荐最终判断标准不是“PDU 构造快了多少”，而是：

```text
在相同信号、相同窗口、相同算法和相同硬件条件下，
200 Hz 长时间运行 0 late / 0 overflow / 0 downstream drop，
CIR 与现有实现数值一致，且队列深度无持续增长。
```
