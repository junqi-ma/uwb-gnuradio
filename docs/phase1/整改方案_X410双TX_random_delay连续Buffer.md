# 整改方案：X410 双 TX random-delay 连续 Buffer

> 面向执行者：Grok
> 日期：2026-09-20
> 平台：X410、CG600、737.28 MS/s、UHD 4.6、DPDK
> 状态：方案已完成，代码与硬件整改待执行

## 1. 问题与目标

当前 C++ multi-TX 路径根据 sensing/jammer 的 begin/end 建立共享 fragment
边界，并对每个区间分别调用 UHD `send()`。X410 CG600 高速 TX 路径对非对齐
中间数据包表现出等效的样点补齐，导致 jammer delay 间接改变 TX0 sensing 的
有效样本时序。

短时硬件数据已经证明：即使 jammer TX gain=0，且 jammer 在 SC16 量化后严格
全零，C++ random-delay 仍会使 CIR peak 随 delay 跳变；Python 连续-buffer
后端在相同 random-delay 下保持稳定。因此根因是 C++ 应用层分片，不是残余 RF
功率、CIR 随机噪声或 delay 参数本身。

整改目标：

- TX0 sensing 每帧使用相同 buffer、指针、长度和样本位置；
- random delay 只改变 TX1 jammer 的连续读取窗口指针；
- multi-TX data send 的边界和调用次数不依赖 delay；
- `max_fragment_size >= L` 时，每拍严格只有一个 data fragment；
- 热路径不分配、不填充整窗、不复制 waveform；
- 单 TX、Python backend、CFO scan/retune 和随机序列不回归。

禁止通过 CIR 后处理减去 jammer delay；根因必须在 TX 连续发送布局中消除。

## 2. 冻结现有时间语义

本轮不重构 `t0`、`time_spec` 或 RX pre-guard。保持现有定义：

```text
tx_ticks = 物理 TX buffer 的 sample 0 时刻
sense 实际起点 = tx_ticks + D
jam 实际起点   = tx_ticks + D + delta
```

其中 `delta in [-D,+D]`。现有 jammer app 已把 `D` 加入
`pre_guard_samples`、RX window 和 publish ROI。Python 连续-buffer 对照已经
证明该几何稳定，因此本轮不得再次移动 `time_spec` 或二次调整 pre-guard。

可以新增只读诊断 metadata：

```text
sense_tx_ticks = tx_ticks + D
jam_tx_ticks = tx_ticks + D + delta
sense_offset_native = D
```

已有 `tx_ticks` 的含义不得改变。

## 3. 连续 Buffer 架构

### 3.1 Uniform random delay

沿用现有几何：

```text
Ns = sensing waveform 长度
Nj = jammer waveform 长度
D  = random delay half-span
L  = max(D + Ns, 2D + Nj)
```

arm schedule 时构造一次 sensing 行：

```text
sense_dense: 长度 L，全零
sense_dense[D : D+Ns] = sensing waveform
```

整个 grid 内 `sense_ptr` 和 `sense_count=L` 恒定。

构造 jammer backing：

```text
jam_backing: 长度 L + 2D，全零
jam_backing[2D : 2D+Nj] = jammer waveform
```

每拍 delay 为 `delta` 时：

```text
jam_window_start = D - delta
jam_ptr = jam_backing.data() + 2*jam_window_start
jam_count = L
```

乘 2 是因为一个 SC16 sample 包含 I/Q 两个 `int16_t`。发送窗口内 jammer
起点为：

```text
2D - (D - delta) = D + delta
```

每拍只改变 `jam_ptr`，不改变 TX0，也不复制任何 waveform。

### 3.2 Fixed delay

fixed 模式同样物化连续长度 `L` 的两行：

```text
sense_dense[0 : Ns] = sensing
jam_dense[delay : delay+Nj] = jammer
```

每拍发送相同的两个连续行。修复后 fixed `+4 native` 不得再产生固定 range
bias，应与 fixed zero 的 sensing tap 一致。

### 3.3 Buffer 所有权

推荐新增不可变 `MultiTxWindowBank`，在 schedule/control 路径构造一次，由
`Job` 持有：

```cpp
struct MultiTxWindowBank {
    uint64_t tx_len;
    uint64_t sense_offset;
    uint64_t jam_backing_wave_begin;
    size_t channel_count;
    size_t jam_channel;

    std::array<std::vector<int16_t>,
               echo::kEchoMaxTxChannels> dense_rows;
    std::vector<int16_t> jam_backing;
};
```

约束：

- 在 `handle_schedule()` 完成校验后、入队前构造；
- 使用 checked arithmetic，并受 `max_tx_samples` 限制；
- `Job` 用 `shared_ptr<const MultiTxWindowBank>` 保证生命周期；
- `apply_schedule()` 只冻结所有权和指针；
- `run_one_burst()` 禁止 resize、assign、fill 或 copy；
- schedule queue 有界，因此同时存活的 bank 数量有界。

若必须严格保持 re-arm 也不分配，可在构造函数按 cap 预分配，但必须报告内存
增量。`max_tx_samples=2^21` 时，4 路 dense 加 backing 可能增加约 40--48 MiB。

## 4. 每拍发送路径

替换 `uwb_realtime_echo_timer.cc` 中按 waveform 边界规划 shared fragments 的路径。

uniform 模式热路径应近似为：

```cpp
const int64_t delta =
    d_rng.next_range_inclusive(-D, D);
const uint64_t jam_window =
    static_cast<uint64_t>(D - delta);

TxBurstFragment& f = d_mtxf_[0];
f.offset = 0;
f.count = L;
f.flags = kFlagTimeSpec |
          kFlagStartOfBurst |
          kFlagEndOfBurst;
f.device_ticks = slot.t_tx_whole;

for (size_t c = 0; c < nch; ++c)
    f.tx_data[c] = bank->dense_rows[c].data();

f.tx_data[jam_ch] =
    bank->jam_backing.data() + 2 * jam_window;

tx_cmd.fragment_count = 1;
tx_cmd.tx_multi_fragments = &f;
```

UHD backend 现有多通道 partial-send cursor 可以保留，但首版连续模式要求：

```text
max_fragment_size >= L
```

当前典型值 `L ~= 189003`、`max_fragment_size=262144`，满足要求。由 UHD 对
一次连续 `send()` 自行 packetize。

以后如恢复应用层分片，必须满足：

- 分片只由固定 `L/F` 决定；
- 所有 delay 使用完全相同的 offsets/counts；
- 非末尾片段满足设备 TX 粒度；
- 禁止以 sensing/jammer begin/end 作为边界。

## 5. 分阶段实施

### M0：基线与工作区保护

开始前记录：

```bash
git status --short
git rev-parse HEAD
```

当前 worktree 有大量用户修改和未跟踪数据。禁止 reset、checkout、清理或覆盖
无关文件。先运行 targeted QA 并保存基线。

旧报告中 random-delay `peak_tap=19` 的结论已被新的 digital-zero 隔离实验
推翻，必须标记为待重新验收。

### M1：先写 layout QA

主要文件：

- `gr-uwb/include/gnuradio/uwb/uwb_echo_multitx.h`
- 新增 `gr-uwb/lib/qa_uwb_echo_multitx_layout.cc`
- `gr-uwb/lib/CMakeLists.txt`

新增 checked helper：

```text
jam_backing_length(L,D)
jam_wave_begin(D) = 2D
jam_window_begin(D,delta) = D-delta
```

QA 覆盖：

- `D=0`；
- `delta=-D,0,+D`；
- 穷举小范围全部 delay；
- sensing/jammer 不同长度；
- jammer 全零但长度非零；
- guard 全零、窗口不越界；
- 长度加法/乘法溢出；
- TX1 窗口与逻辑 composite 逐样本一致；
- 所有 delay 下 TX0 逐字节相同。

旧 `multitx_burst_bounds()` 可以暂时保留，但新生产路径不得调用。

### M2：EchoTimer 连续窗口实现

修改：

- `gr-uwb/include/gnuradio/uwb/uwb_realtime_echo_timer.h`
- `gr-uwb/lib/uwb_realtime_echo_timer.cc`
- `gr-uwb/lib/qa_uwb_echo_timer.cc`

要求：

1. `Job` 持有 immutable window bank；
2. schedule 接收时物化 dense rows/backing；
3. `apply_schedule()` 冻结 bank；
4. 删除 delay-dependent bounds；
5. 每拍只生成一个连续 multi-TX fragment；
6. fixed/random 均走连续行；
7. 同 seed 的 delay 序列保持 bit-exact；
8. single-TX 分支完全不改；
9. wire accounting 和 async 归属不变。

Fake backend 至少运行 256 拍并证明：

- TX0 每拍逐样本相同；
- TX1 起点等于 `D+jam_delay_native`；
- 同 seed re-arm 后序列一致；
- 静默 jammer 的 TX1 整行全零；
- `fragment_count==1`；
- time/SOB 只在首次；
- EOB 仍为一次 zero-length send；
- partial send 后两路使用同一个 continuation cursor；
- 热路径 buffer 地址和 capacity 不变。

### M3：metadata 透传

EchoTimer 已发布 delay 字段，但 resampler/CIR metadata whitelist 会丢弃它们。

修改：

- `gr-uwb/include/gnuradio/uwb/uwb_radar_pdu_meta.h`
- 65/48 resampler QA
- CIR estimator block QA

至少透传：

```text
jam_delay_native
jam_delay_us
jam_delay_mode
jam_delay_seed
jam_freq_plan_hz
jam_freq_actual_hz
jam_freq_offset_hz
jam_retune_seq
sense_offset_native
tx_fragment_count
```

`jam_delay_native` 是 native 控制量，经过 65/48 时不得缩放。最终字段必须进入
`cir.jsonl`，供逐 delay 分组验收。

### M4：Python app、参数与文档

修改：

- `gr-uwb/apps/echo_cir_jam_plan.py`
- `gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`
- `gr-uwb/apps/test_echo_cir_jam_plan.py`
- `gr-uwb/apps/test_jam_app_args.py`

保持 schedule PDU 携带有效 waveform，不在 Python 中构造完整 `(2,L)`。

增加：

- dry-run 输出 `contiguous-window`；
- 输出 `L`、`D`、backing 长度、jam window 范围；
- 输出预期 `data_fragments=1`；
- `max_fragment_size<L` 时明确拒绝；
- QA 证明 `jam-scale=1e-8` 转成 SC16 后严格全零。

bindings/factory 签名不需要改变；GRC 只更新说明。

## 6. 构建与软件回归

每次有意义修改后构建并运行相应 targeted QA。完成后至少执行：

```bash
cmake --build gr-uwb/build -j

ctest --test-dir gr-uwb/build \
  -R 'uwb_qa_uwb_(echo_multitx_layout|echo_timer|uhd_backend|pdu_rational_resampler|radar_cir_estimator|radar_e2e)' \
  --output-on-failure

ctest --test-dir gr-uwb/build --output-on-failure

python3 gr-uwb/apps/test_echo_cir_jam_plan.py
python3 gr-uwb/apps/test_echo_tx_channels.py
python3 gr-uwb/apps/test_jam_app_args.py
```

同时保留 UHD-OFF 构建，并建议用 ASan/UBSan 跑 backing 首尾窗口与溢出测试。

## 7. X410 功能验收

统一配置：

```text
X410 CG600，737.28 MS/s
DPDK，taskset -c 2-19
max_fragment_size=262144
no UDP，固定 seed
每组独立输出目录
```

每组 300--500 帧，前 20 帧作为 warm-up。

| 组 | 后端 | delay | jammer |
|---|---|---|---|
| A | C++ | fixed 0 | SC16 全零、TX gain 0 |
| B | C++ | random +/-4 native | SC16 全零、TX gain 0 |
| C | C++ | fixed +4 native | SC16 全零、TX gain 0 |
| D | Python | random +/-4 native | scale 1e-30、TX gain 0 |

功能硬门槛：

```text
A/B/C sensing peak 中位数最大差 <= 1 tap
B 各 delay 分组的 peak 中位数最大差 <= 1 tap
99% CIR 最佳互相关 shift = 0，其余 <= 1 tap
preamble_start 最大差 <= 1 sample
各 delay 组 metric 中位数相对 A <= 0.5 dB
raw/CIR norm 中位数相对 A <= 1%
echo_ok == cir_ok == pulses
echo_fail/res_drop/est_drop == 0
underflow/seq_error/time_error == 0
```

禁止再次出现：

```text
delay -4..+4
-> peak 25/36/36/36/25/34/33/31/30
```

静默 jammer 验证通过后，再用低功率 code10 jammer 验证 TX1 自身确实随 delay
移动；TX0 code9 的相关峰必须保持不动。

## 8. 性能与 soak

功能修复通过后依次执行：

1. 100/200/300/500 Hz，各 10 秒；
2. 200 Hz × 60 秒；
3. 200 Hz × 10 分钟；
4. 300 Hz × 3 次 60 秒；
5. 300 Hz × 10 分钟；
6. 500 Hz × 60 秒能力测试。

正式性能门：

```text
echo_ok == pulses
cir_ok == pulses
echo_fail == 0
underflow/seq_error/time_error == 0
res_drop/est_drop/writer_fail == 0
late_slot_skip == 0
```

当前仓库已有独立的宿主偶发 late 问题，因此结论必须拆开：

- CIR 不再依赖 delay：guard-buffer 功能修复通过；
- soak 仍有 late：生产性能验收未通过，不得写成全部完成。

## 9. Grok 多任务编排

建议按以下依赖顺序协调 agent，避免并发覆盖：

1. Agent A：只读基线、dirty-worktree 审计、失败复现与接口冻结；
2. Agent B：先写 layout/Fake backend 失败 QA，不改生产代码；
3. Agent C：QA 契约冻结后实现 WindowBank 和 EchoTimer 热路径；
4. Agent D：独立完成 metadata passthrough、Python dry-run 和文档；
5. 主 agent：逐阶段合并，每次有意义修改后构建并跑 targeted tests；
6. UHD-free 和全量测试完成后，独占 X410 做硬件测试。

最终报告必须包含：

- commit 与 dirty-worktree 处理说明；
- 改动文件清单；
- 软件测试命令与结果；
- 内存增量；
- 每拍 data send 次数；
- `delay -> peak_tap/metric` 表；
- async/late/drop 计数；
- 同会话 fixed-zero 对照；
- 原始 `summary.json`、`cir.jsonl` 路径；
- 更新后的 `开发状态.md` 与整改测试报告。

## 10. 明确禁止的做法

- 不得按 jammer delay 在 CIR 后处理补偿 sensing；
- 不得把 delay 量化到 8 samples 来掩盖问题；
- 不得继续在 jammer begin/end 处调用独立 `send()`；
- 不得每拍重建、清零或复制完整 `(2,L)` buffer；
- 不得在本轮改变 schedule grid、`tx_ticks` 或 RX pre-guard 语义；
- 不得修改 Python backend 行为作为 C++ 修复替代品；
- 不得把已有宿主 late 问题写成通过；
- 不得 reset、checkout 或覆盖工作区中的用户修改。
