# 开发规划：X410 C++ 双 TX 高性能 UWB Radar CIR

> 面向执行者：OpenCode
>
> 编写日期：2026-09-19
>
> 目标平台：X410、CG600、737.28 MS/s、UHD 4.6、DPDK
>
> 基线应用：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py`
>
> 目标应用：`gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`

## 1. 最终目标

把 jammer 应用从 Python `TimedUhdEcho` 双通道发送路径迁移到现有 C++
`UwbRealtimeEchoTimer + UhdBurstBackend` 路径，实现：

```text
一次性提交 sense/jammer 波形和扫描计划
  → C++ device-time grid
  → 一条 UHD 多通道 SC16 TX streamer，channels=[sense, jammer]
  → 每个 PRI 同时发送两个通道
  → timed RX
  → native SC16 PDU
  → 65/48 PDU resampler
  → Radar CIR estimator / writer / UDP
```

性能目标不是“C++ 能发出第二路”，而是：

1. 双 TX 与标准单 TX Radar CIR 使用同一种 C++ 调度和 UHD 发送路径；
2. 200 Hz 必须通过 10 分钟 soak；
3. **300 Hz 是本任务硬目标**：在同一次上板会话中，若标准单 TX Radar CIR
   能通过 300 Hz，则双 TX 也必须通过相同规模验收，不得把目标静默降到 50 Hz；
4. steady-state 必须是 0 late、0 TX underflow、0 TX sequence/time error、
   0 RX overflow/timeout、0 resampler/estimator/writer drop；
5. 支持现有 align jammer 的固定/随机时延、频率扫描、dwell、仅 jammer NCO
   retune、settle 后重锚，以及逐脉冲 metadata；
6. 单 TX 旧接口和结果不得回归。

## 2. 实施前必须阅读和核对

执行前完整阅读：

- `AGENTS.md`
- `开发需求参考.md`
- `开发状态.md`
- `docs/phase1/开发方案_X410双TX_UWB通信干扰CFO研究.md`
- `docs/phase1/测试报告_X410双TX_jammer_CFO零点.md`
- `docs/phase1/开发指南_UWB_Radar_PDU模式速率优化.md`
- `docs/phase1/整改指南_UWB_Radar_PDU速率优化验收.md`
- `docs/phase1/测试报告_UWB_Radar_PDU速率优化.md`
- `docs/DPDK_X410_CG600启用.md`

必须先搜索并记录本地参考实现，不得凭印象写 UHD 语义：

- GNU Radio UHD sink：`/home/oi/gnuradio/gr-uhd/lib/usrp_sink_impl.cc`
- gr-radar EchoTimer：
  `/home/oi/workspace/gr-radar-master/lib/usrp_echotimer_cc_impl.cc`
- 本机 UHD 多通道接口：`/usr/local/include/uhd/stream.hpp`
- 当前 C++ echo/backend：
  - `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`
  - `gr-uwb/lib/uwb_realtime_echo_timer.cc`
  - `gr-uwb/include/gnuradio/uwb/uwb_echo_scheduler_core.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_echo_burst_backend.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_uhd_backend_config.h`
  - `gr-uwb/include/gnuradio/uwb/uwb_uhd_burst_backend.h`
  - `gr-uwb/lib/uwb_uhd_burst_backend.cc`

开始前保存以下只读信息到开发记录：当前 commit、`git status --short`、构建配置、
CTest 基线。工作区已有大量用户改动和数据；不得 reset、checkout 或覆盖无关文件。

## 3. 当前瓶颈和必须修正的结构

当前标准 app 默认走 `cpp-pdu`，但 jammer app 明确拒绝该后端并强制 Python。
Python jammer 每拍执行：

```text
issue_stream_cmd
→ Python/pyUHD 双通道 fc32→sc16 send
→ recv 完整窗口
→ 可选 SC16 转换/写盘
→ rx.tolist() + init_c32vector()
```

现有 50 Hz 双 TX 日志已经测到：

- `L=188999` native samples/channel；
- `(2,L)` send 平均约 8.5 ms、最大约 12.5 ms；
- RX 前缀转 PMT 约 4.8–5.2 ms；
- 若启用 `--dump-sc16`，另有约 7.3 ms/拍。

因此把 Python 循环简单搬到另一个线程不算完成。必须消除每拍 Python/GIL、FC32
转换和 composite 重排，并使用 C++ SC16 多通道 streamer。

现有 C++ 路径还有四个单通道假设：

1. `UhdBurstBackendConfig` 只有一个 `tx_channel`；
2. `BurstFragment` 只有一个 `tx_data` 指针；
3. schedule PDU 的 cdr 只接受一个 `s16vector`；
4. `IRadioBurstBackend::tune()` 只能按标准雷达语义联调 TX/RX，不能只调 jammer TX。

这四层必须一起改，不能只改 `tx_args.channels`。

## 4. 冻结的调度语义

Block 类型保持现有 **message-only `gr::block`**。不要新建 `sync_block` 或
`tagged_stream_block`，也不要复制一套 EchoTimer。

```text
Python/control thread
  └─ 启动时只提交一次 schedule PDU：波形、grid、jam plan

UwbRealtimeEchoTimer radio worker（唯一调度所有者）
  ├─ 在整数 device-tick grid 上取得下一个未来 slot
  ├─ dwell 边界仅 retune jammer TX，并在 settle 后重锚当前 index
  ├─ 生成本拍 jammer delay（如启用）
  ├─ 用固定数组规划多通道 TX fragments
  ├─ 先 issue timed RX
  ├─ 再用一条多通道 TX streamer 发送所有通道
  ├─ collect RX 到预分配 SC16 scratch
  ├─ 发布 native SC16 RX PDU
  └─ 立即进入下一 slot，不等待 FIR/CIR/writer

UHD async thread
  └─ 持续收 TX async metadata，按 time/channel 归属 underflow、seq、late
```

约束：

- 过期 slot 只跳过，不追赶；每个有效 schedule index 恰好一个结果 PDU；
- radio worker 中禁止 Python callback、磁盘 I/O、UDP、逐拍打印和动态扩容；
- RX scratch、TX 零区、fragment 数组、UHD buffer-pointer 数组全部预分配；
- `send()` 仍由单一 worker 调用，同一 streamer 不允许并发 send；
- retune 只能发生在 burst 边界，不能与 send/recv 并发；
- 下游队列必须有界，积压不能反压 radio worker；
- stop 必须中止在途 I/O并 join，不得死锁。

## 5. C++ 接口与数据契约

### 5.1 UHD 多 TX 配置

把单个 TX 配置扩展成最多 4 路的固定上限配置。推荐新增：

```cpp
inline constexpr size_t kEchoMaxTxChannels = 4;

struct UhdTxChannelConfig {
    size_t channel;
    std::string antenna;
    double gain_db;
    double center_freq_hz;
};

struct UhdBurstBackendConfig {
    ...
    std::vector<UhdTxChannelConfig> tx_channels;
    size_t rx_channel;
    ...
};
```

配置阶段允许分配；热路径不允许。为兼容旧代码，可在一个过渡版本中保留原
`tx_channel/tx_antenna/tx_gain_db/center_freq_hz` scalar factory，由 factory 转成
长度为 1 的新配置；内部实现只能保留一套真相，禁止长期维护两套发送逻辑。

校验至少包括：1–4 路、物理通道不重复、所有通道 sample-rate readback 严格一致、
频率/增益有限、streamer `get_num_channels()` 等于请求数。

`prepare()` 必须逐 TX 通道设置并回读 rate/frequency/gain/antenna，然后创建：

```cpp
stream_args_t tx_args("sc16", "sc16");
tx_args.channels = {sense_channel, jam_channel};
```

不得退回 Python `fc32` host format。

### 5.2 多通道 TxCommand / fragment

修改 backend 抽象，使每个 TX fragment 同时携带 N 路等长 buffer 指针；RX
fragment 保持单通道。可以拆成 `TxBurstFragment`/`RxBurstFragment`，或使用固定
二维指针，但必须满足：

```text
每次 UHD send 的 buffers.size == configured_tx_channel_count
每个 buffer 本次发送相同 nsamps_per_buff
time_spec + SOB 只在整个 burst 的第一次 send
所有 data send 都不带 EOB
全部通道全部样点接受后，只发一次 zero-length EOB
partial send 从所有通道的同一 sample offset 续发
```

禁止在当前 `issue_tx()` 循环里继续临时构造 `std::vector<const void*>`。使用
`std::array<const void*, kEchoMaxTxChannels>`，再用 UHD `ref_vector(ptr, count)`，
保证每次 send 不分配。

`BurstResult` 和 metadata 必须消除“样点数是单通道还是聚合”的歧义：

- 保留旧 `tx_samples_requested/sent`，定义为 **每通道** 数量以兼容单 TX；
- 新增 `tx_channel_count`、`tx_wire_samples_requested/sent`（通道数×每通道）；
- async 计数新增 channel 字段或 per-channel 计数。

### 5.3 schedule PDU 向后兼容

冻结以下输入形式：

```text
单 TX（旧格式，必须继续支持）
  cons(meta, s16vector)

多 TX（新格式）
  cons(meta, pmt_vector[s16vector_ch0, s16vector_ch1, ...])
```

多 TX payload 保存的是每路“有效波形”，不要求 Python 每拍生成 `(2,L)` 大矩阵。
metadata 至少包含：

```text
tx_channel_count
tx_samples                    # 每通道物理 burst 长度 L
tx_waveform_samples[]         # 每路有效波形长度
tx_base_offsets_native[]      # 每路在 L 内的基准起点
jam_logical_channel           # 默认 1
jam_delay_mode                # fixed / uniform
jam_delay_lo_native
jam_delay_hi_native
jam_delay_seed
jam_freq_offsets_hz[]
jam_dwell
jam_freq_settle_ticks
```

所有浮点 us 参数在 Python/control 层一次性转换成 native integer ticks；热路径不做
浮点几何换算。handler 必须完整验证 vector 数量、SC16 偶数元素、长度、offset、上界、
fragment 上限后才入队。Job 持有 PMT payload 以保证底层指针生命周期覆盖整个 grid。

### 5.4 随机时延必须无整窗重排

不要在每拍对 `(2,L)` 做 `fill(0)+copy`。实现一个纯 C++、无分配的多通道布局
planner：

1. sense 固定放在 `sense_offset=D`；
2. jammer 放在 `D + delay`，其中 `delay∈[-D,+D]`；
3. `L=max(D+sense_len, 2D+jam_len)` 在 arm 时固定；
4. 每拍仅生成公共边界 `{0, sense_begin/end, jam_begin/end, L}`；
5. 对每个区间，每路指针指向自己的波形切片或预分配全零 scratch；
6. 再按 `max_fragment_size` 拆分到固定 fragment 数组。

这样随机时延只改变少量整数和指针，不复制约 1.5 MB composite buffer。必须在 arm
时证明最坏 fragment 数不超过 `kEchoMaxFragmentsPerBurst`，否则拒绝 schedule。

随机数算法必须固定、可复现、无分配。不要依赖跨标准库实现不保证完全一致的
`uniform_int_distribution`；推荐定义 PCG/xorshift 状态和无偏整数映射。每拍输出实际
`jam_delay_native`/`jam_delay_us` metadata。

### 5.5 jammer-only retune 与 dwell 重锚

保留现有标准雷达 `tune(center)` 语义，同时增加明确的按逻辑 TX 通道调谐接口，例如：

```cpp
virtual BurstStatus tune_tx_channel(
    size_t logical_channel, double hz, double& actual_hz, std::string& error);
```

UHD backend 将逻辑通道映射到配置中的物理通道；Fake backend 记录调用顺序和回读值。

在 `pulse_id % jam_dwell == 0` 且目标频率变化时，worker 必须：

1. 完成上一拍并确认没有在途 send/recv；
2. 仅调 jammer TX；sense TX 和 RX 保持 `f0`；
3. 读取 jammer 实际频率；
4. 取当前 device time；
5. 将当前 schedule index 重锚到 `now + settle_ticks`；
6. pulse_id/index 保持连续，不制造虚假丢帧；
7. 将计划/实际 jammer 频率和 retune 序号写入该拍 metadata。

retune settle 是扫描墙钟时间，不得计成 late，也不得通过跳过 dwell 内 pulse 来追赶旧 grid。

### 5.6 TX async 事件必须成为验收数据

现有 C++ backend 已有 async thread，扩展时不得退化。事件至少记录：

- burst ACK；
- underflow / underflow-in-packet；
- sequence error / sequence-error-in-burst；
- time error；
- event channel；
- 能匹配到的 `tx_ticks/schedule_index/pulse_id`；
- 无 time spec 或无法匹配的 startup/unmatched 事件。

新增可查询计数和 summary 字段：

```text
tx_async_underflow
tx_async_seq_error
tx_async_time_error
tx_async_unmatched
tx_async_dropped
```

不能再用“`send()` 返回完整长度”代替“没有 U”。

现有 `collect_result()` 中按 burst 创建的 `std::vector<AsyncEvent>` 也属于需要清理的
热路径分配：改成固定容量 `std::array`、预分配 ring，或由 consumer 直接累计到结果；
超出容量时增加 `tx_async_dropped`，不得静默丢事件。

## 6. 应用层改造

### 6.1 保持一个 jammer app

不要新建第三个硬件 app。修改
`gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`：

- `align` 模式允许 `--echo-backend cpp-pdu`；
- 完成硬件验收后，jam-enabled align 默认选择 `cpp-pdu`；
- 保留 `--echo-backend python` 作为诊断/回归 fallback；
- 删除当前“cpp-pdu cannot carry jammer”的硬拒绝及对应静态测试；
- sense/jammer 只在启动时各生成、重采样、归一化并转换一次 SC16；
- Python 构造一次多 TX schedule PDU，不再运行逐拍 `_one_burst()`；
- 现有 CLI、频率扫描生成、delay seed、summary 字段保持兼容；
- summary 增加 `tx_channels`、每路有效长度、物理 L、C++ delay/retune/async 计数。

`continuous` 独立 jammer streamer 不属于本阶段。cpp-pdu 遇到
`--jam-mode continuous` 必须明确拒绝并说明仍需 Python/后续实现，不能悄悄按 align 发。

### 6.2 RX/CIR 链不得另写

继续复用：

```text
realtime_echo_timer_uhd.burst
→ pdu_rational_resampler_ccf_65_48(UnitRange)
→ radar_cir_estimator
→ cir_writer / UCR2 UDP
```

保持 `publish_native` ROI、CG600 `cal_delay_native=1092.5`、predicted timing、
116 taps、pulse_id、物理 `sample_count` 和已发布 `published_samples` 语义不变。

cpp-pdu 性能验收禁止启用同步 raw IQ 写盘。若后续需要 `--dump-sc16`，应增加消费
RX PDU 的独立有界异步 writer，绝不能在 radio worker 写盘；这不是本任务性能门的
阻断项。

### 6.3 Python bindings / GRC / dry-run

更新：

- `gr-uwb/python/uwb/bindings/python_bindings.cc`
- `gr-uwb/grc/uwb_realtime_echo_timer.block.yml`
- `gr-uwb/lib/uwb_uhd_dry_run.cc`

新 factory 接受 TX channel/gain/antenna/frequency 数组。旧 scalar factory 可以保留为
兼容 wrapper。dry-run 必须打印：通道映射、每通道/聚合字节、active waveform 区间、
最坏 fragment 数、delay 范围、dwell/retune/settle 计划。

## 7. 分阶段实施顺序

每个里程碑独立 build + targeted QA + full CTest；上一阶段不过不得进入下一阶段。

### M0：冻结基线

不改代码，采同一会话基线：

- 标准 cpp-pdu 单 TX：200 Hz、300 Hz，各 3×30 s；
- 当前 Python 双 TX：50/100/200 Hz 短测，仅用于确认旧瓶颈；
- 全部关闭 raw dump，固定频率/固定 delay，`--no-udp`；
- 保存命令、commit、FPGA/UHD/DPDK、CPU governor、stdout/stderr、summary；
- 标准基线必须记录 async error、worker 分段、队列 HWM、CIR 完整性。

若标准单 TX 在本次环境不能通过 300 Hz，应先修复环境或解释与用户既有 300 Hz 结果的
差异；不得因此直接把双 TX 验收目标改成 200 Hz。

### M1：纯 C++ 多通道契约与 Fake backend

修改 config、TxCommand/fragments、Fake backend 和纯 planner，不碰真实 UHD。

必须先有 QA：

- 旧单 TX PDU bit-exact 回归；
- 2/4 通道配置合法性和重复/越界拒绝；
- 两路 fixed payload、不同长度、前后零填充逐样本正确；
- fragment 边界、SOB/time/EOB、partial send 续传正确；
- random delay 最小/零/最大和固定 seed 可重复；
- fragment 上限失败可诊断；
- scratch 地址/capacity 在整个 grid 不变；
- TX layout planner 与 backend `issue_tx()` 热路径 allocation counter 为 0；既有且必要的
  RX PDU/metadata 构造分配允许保留，但必须单独计时，不能混称为“整个 worker 零分配”。

### M2：真实 UHD 多通道 fixed TX

只实现固定双 TX，不接扫描/random delay。更新 binding/dry-run，先跑 UHD-free QA，再做
X410 TX-only/短 RX smoke：

- streamer 回读 2 channels；
- 每拍两路 `tx_samples_sent == L`；
- async underflow/seq/time error 为 0；
- 每个 burst 恰好一个 EOB；若当前 UHD/FPGA 配置会返回 burst ACK，则 ACK 必须被记录并
  正确归属。ACK 缺失本身仅在同会话标准单 TX 稳定产生 ACK 时才判失败；
- 发送已知 marker，验证两路同一 timed burst 且 sample alignment 正确。

如果 fixed 双 TX 仍出现 U，先做最小 TX-only 双通道 DPDK benchmark，区分 backend
fragment/lead 问题与物理传输上限，不要继续叠加 CIR、random delay 和 retune。

### M3：EchoTimer 多 TX schedule + jammer app fixed align

接通新 schedule PDU 与 jam app cpp-pdu fixed 模式。验收：

- `--jam-enable --jam-delay-us 0 --jam-freq-offset 0` 可完整运行；
- jam-disabled 的新 app 与标准单 TX payload/metadata/CIR 一致；
- jam row 全零时，离线向两条处理链输入同一 RX 数据，CIR 逐帧一致到既有容差；硬件
  A/B 只比较包数、坐标和预先声明的统计容差，不要求两次独立 RF 采集逐样本相同；
- jam on 时复现已有 code/CFO 基本现象；
- Python 不再有逐拍 timed loop、`tolist()` 或 composite 重建。

### M4：无拷贝 per-pulse random delay

接入 C++ delay planner/PRNG。验收：

- 每个 delay 在闭区间内且落在 native sample grid；
- metadata 与 Fake backend 实际布局逐拍一致；
- 固定 seed 两次运行得到相同序列；
- 负/零/正 delay 的 sense 坐标保持不动；
- 开启 random delay 后 worker timing 和 300 Hz 完整性不回归。

### M5：jammer-only scan retune

接入 offsets/dwell/settle。Fake QA 和硬件 smoke 必须覆盖：

- 升序、降序、单点、stop 非整步；
- dwell 边界恰好发生在指定 pulse_id；
- 只改变 jammer 物理 TX channel；
- retune 失败有明确状态，不能把旧频率标成新频率；
- settle 重锚后 index/pulse_id 连续，0 late；
- `jam_freq_actual_hz` 回读误差符合 X410 NCO 能力。

### M6：端到端性能和科学结果验收

按第 9 节矩阵执行。只有这一阶段完成后，才能把 jam app 默认切到 cpp-pdu，并更新
`开发状态.md` 为“C++ 双 TX 已验收”。

### M7：文档和清理

- 新增独立测试报告，列出原始日志路径和失败项；
- 更新使用命令和 backend 选择说明；
- 删除过时的“jam 只能 Python”文案；
- 不删除 Python fallback；
- 不提交大型 IQ、构建产物、DPDK/UHD 二进制。

## 8. QA 清单

### 8.1 必须新增/扩展的自动测试

优先扩展而不是另起重复测试框架：

- `gr-uwb/lib/qa_uwb_echo_timer.cc`
- `gr-uwb/lib/qa_uwb_uhd_backend.cc`
- `gr-uwb/lib/qa_uwb_uhd_backend_device.cc`
- `gr-uwb/apps/test_echo_cir_jam_plan.py`
- `gr-uwb/apps/test_jam_app_args.py`
- `gr-uwb/apps/test_echo_tx_channels.py`

至少覆盖：

1. 旧单通道 factory/PDU/API 全部继续工作；
2. 多通道 payload 类型、数量、长度和 offset 的拒绝路径；
3. 两路数据逐样本记录与零区；
4. partial send、0 return/timeout、EOB、stop during I/O；
5. async 事件带 channel/time 的归属和 unmatched 队列上限；
6. random delay PRNG/布局；
7. dwell/retune/re-anchor；
8. queue-full、restart、drain；
9. single EchoTimer→65/48→CIR 端到端 flowgraph；
10. UHD-OFF 构建仍通过。

### 8.2 每个里程碑的命令

```bash
cmake --build gr-uwb/build -j
ctest --test-dir gr-uwb/build -R 'uwb_qa_uwb_(echo_timer|uhd_backend)' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
python3 gr-uwb/apps/test_echo_cir_jam_plan.py
python3 gr-uwb/apps/test_echo_tx_channels.py
python3 gr-uwb/apps/test_jam_app_args.py
```

不要把仓库已有、与本改动无关的环境型吞吐阈值失败写成新增回归；但必须记录修改前后
同一个测试的结果。

## 9. X410 验收矩阵

所有 A/B 使用同一 commit、同一启动会话、相同 RF/波形/ROI/下游参数，关闭 raw dump。
每轮都保存 stdout、stderr、summary、CIR JSONL 和 async 统计。

### 9.1 功能阶梯

| 阶段 | 配置 | 最低规模 | 通过条件 |
|---|---|---:|---|
| A | 单 TX cpp-pdu 标准基线 | 1000 | 与既有结果一致，0 error/drop |
| B | 双 TX，jam row 全零 | 1000 | 离线同输入一致；硬件包数/坐标一致且统计量在预设容差内 |
| C | 双 TX fixed jam/fixed CFO | 1000 | 0 radio/drop，jam 确实到达 RX |
| D | 双 TX random delay | 1000 | delay 逐拍正确，0 radio/drop |
| E | 16–32 点 scan、dwell 20 | ≥320 | retune/metadata 正确，0 late/U |
| F | 1638 点×100 dwell 全扫描 | 163800 | 全量无丢帧、无 steady U、结果可分析 |

### 9.2 性能门

1. **200 Hz mandatory**：120000 pulses / 10 min；
2. **300 Hz mandatory**：先做 3×60 s A/B，再做至少 10 min；
3. 500 Hz 只作压力测试，不得替代 200/300 验收；
4. full scan 在 300 Hz 时，纯 PRI 时间约 546 s，另加约 82 s retune settle，预期
   墙钟约 10.5 min；明显更长必须解释。

每轮硬门槛：

```text
echo_ok == requested pulses
echo_fail == 0
echo_late == 0
tx_async_underflow == 0          # counted steady-state
tx_async_seq_error == 0
tx_async_time_error == 0
rx_overflow == 0
rx_timeout == 0
res_drop == 0
est_drop == 0
wr_ok == requested pulses
missing_count == 0
dup_count == 0
backlog 无长期正斜率
```

DPDK/UHD 初始化阶段无时间戳的事件必须单列为 startup/unmatched，不能混入 steady-state，
也不能直接忽略。报告需同时给出单 TX 与双 TX 的 worker total、TX send、RX collect、PDU
build/publish 均值和最大值；若增加可靠的固定开销 histogram，再给 p50/p95/p99。

### 9.3 数值与科学结果

- jam row 全零：同一 RX 输入下，65/48 输出和 CIR taps 按既有容差逐样本对照；
- jam on：不能要求 CIR 等于单 TX，因为干扰正是实验变量；应复现已有报告中的
  `df=0` floor 上升和 `df≈491.34 kHz` 回到本底趋势；
- `peak_tap` 坐标、cal delay、physical/published sample count 不因双 TX 改造漂移；
- 随机 delay 的 metadata 必须和捕获中相关峰位移一致；
- 最终结果不得只看 `echo_ok`，必须同时检查 async TX error 和 CIR 完整性。

## 10. 失败时的诊断顺序

出现 `U`、late 或吞吐不足时，按以下顺序定位，不允许直接调低 rate：

1. 固定频率、固定 delay、关闭 CIR/UDP/dump，跑 TX-only 双通道；
2. 检查 async event 的 channel、ticks、是否 startup/unmatched；
3. 检查每通道 sample count、partial send、fragment/EOB；
4. 对照 DPDK 单/双通道实际聚合吞吐；
5. 检查 TX lead、`send_timeout`、fragment size 和 CPU affinity；
6. 加回 timed RX；
7. 加回 RX PDU/65-48/CIR；
8. 最后才加 random delay 和 retune。

若 TX-only 双通道可通过 300 Hz，而顺序 `issue_rx→send→collect` 仍因 worker 周期超过
PRI 失败，再单独设计有限深度的 timed-command lookahead。该改造必须先验证 X410/UHD
可排队的 RX command 数，并保持 RX-before-TX、retune barrier 和 stop 语义；不得未经
验证就引入第二个线程并同时操作同一 TX streamer。

若 TX-only 双通道本身不能通过，则优先调查 transport/streamer/fragment，而不是修改
CIR 算法或降低 998.4 MS/s 工作率。

## 11. 明确不在本任务范围内

- full UWB demodulation；
- 修改 MATLAB/CIR 核心算法换性能；
- 降低 998.4 MS/s 工作率；
- jammer `continuous` 独立 streamer 模式；
- 在 radio worker 内做 raw IQ 写盘；
- 用无限队列、超大缓存或丢帧隐藏无法实时的问题；
- 把 Fake backend、dry-run 或短 smoke 写成 X410 soak 验收通过。

## 12. Definition of Done

只有同时满足以下条件才能宣称完成：

1. 单 TX 兼容 QA 和标准 Radar CIR 硬件回归通过；
2. C++ 双 TX fixed/random-delay/scan 功能全部有 Fake QA；
3. X410 双 TX 200 Hz 10 min 全绿；
4. X410 双 TX 300 Hz 与同会话标准单 TX 达到相同的零错误/零丢帧级别，并完成
   3×60 s + 10 min；
5. 至少一次真实多频点 retune soak 通过；最终目标为 1638×100 全扫描通过；
6. TX async underflow/seq/time error 被可靠计数，而不是靠观察控制台 `U`；
7. random delay 和 jammer NCO metadata 与实际发送一致；
8. 全量 CTest、Python QA、UHD-OFF 构建通过，或只剩修改前已存在且有证据的环境失败；
9. 新测试报告、使用说明和 `开发状态.md` 已更新，未夸大未完成项。
