# GNU Radio 软件升采样开发方案：737.28 MS/s → 998.4 MS/s

更新时间：2026-08-11

> 本文是交给 Claude Code 直接执行的任务书。当前阶段固定接受 X410 输入为
> **737.28 MS/s**，不在本阶段重新讨论 X410 内部时钟或其他输入采样率。
> 速率变换使用精确最简有理数 **65/48**。

## 1. 目标与边界

在主机 GNU Radio 中实现并验证以下连续流生产路径：

```text
X410 / RFNoC RxStreamer
SC16 or CF32 @ 737.28 MS/s
        │
        ▼
GNU Radio rational resampler 65/48
        │
        ▼
SC16 or CF32 @ 998.4 MS/s
        │
        ├─ UwbScheduledExtractor（已知 t0/T）
        └─ UwbDetectorSc16 / UwbDetector（未知 packet 时刻）
```

精确速率合同：

```text
737.28e6 × 65 / 48 = 998.4e6
gcd(65, 48) = 1
```

本阶段目标按优先级排序：

1. 输出波形、绝对样本坐标和 MATLAB 65/48 重采样结果一致；
2. 连续流跨任意 GNU Radio `work()` chunk 时结果不变；
3. 不破坏 UHD `rx_time`、overflow/discontinuity 和采样率 metadata 语义；
4. 建立 GNU Radio 自带 resampler 的真实性能基线；
5. 在基线不足时，实现固定 65/48、无热路径分配的优化 C++ block；
6. 判断单通道连续 737.28 → 998.4 MS/s 是否能在目标主机持续实时运行。

本阶段不做：

- RFNoC/FPGA resampler；
- 改写 998.4 MHz MATLAB 解调算法为 737.28 MHz 原生算法；
- GPU/CUDA；
- 多通道；
- 通过降低滤波质量、丢样或静默丢 tag 获得虚假吞吐；
- 把插值描述为恢复 X410 未采集到的模拟带宽。

升采样只把已有信号映射到 998.4 MHz 数字网格，不产生新的射频信息。

---

## 2. 开发前强制调查

Claude Code 在实现任何 block 前必须完成并记录以下调查。

### 2.1 搜索本地 GNU Radio 源码

优先使用 `rg`，查找本机 GNU Radio 中：

```text
rational_resampler_impl
rational_resampler_base
pfb_arb_resampler
set_relative_rate
forecast
general_work
TPP_DONT
rx_time tag propagation
```

已知本机公开接口位于：

```text
/usr/include/gnuradio/filter/rational_resampler.h
```

如果本机没有实现源码，再查当前安装版本对应的 GNU Radio 官方源码。必须在开发
记录中写清：

- GNU Radio 版本；
- 内置 `rational_resampler_ccf` 的 block 类型；
- taps 如何拆成 polyphase arms；
- `forecast/general_work` 如何处理 history、consume 和 phase；
- 默认 tag propagation 行为；
- 内置 kernel 是否已经使用 VOLK/SIMD；
- 是否支持 packed complex int16 输入输出。

不得仅凭 block 名称假设实现性能。

### 2.2 检查本项目相似 block

至少阅读：

```text
gr-uwb/lib/uwb_detector_sc16.cc
gr-uwb/lib/uwb_scheduled_extractor.cc
gr-uwb/include/gnuradio/uwb/uwb_detector_sc16.h
gr-uwb/include/gnuradio/uwb/uwb_scheduled_extractor.h
gr-uwb/apps/benchmark_detector_sc16.cc
gr-uwb/lib/CMakeLists.txt
gr-uwb/apps/CMakeLists.txt
```

复用项目现有约定：

- SC16 stream item 为 `std::complex<int16_t>`；
- 大 chunk、预分配、无 `work/general_work` 内存分配；
- QA 使用 Boost.Test 并注册到 CTest；
- benchmark 输出编译器、CPU、GNU Radio 版本、格式和速率；
- 不修改或破坏现有 detector/demod QA。

### 2.3 开发前记录基线

在方案实施记录中保存：

```bash
git status -sb
cmake --build gr-uwb/build -j
ctest --test-dir gr-uwb/build --output-on-failure
```

先确认已有测试通过；如果已有失败，必须明确区分基线失败和本任务回归。

---

## 3. 分阶段架构

不要直接从零编写 SIMD resampler。按下面三个层级逐步推进，每一级都要先 QA、
benchmark、记录结论，再决定是否进入下一级。

### Level A：GNU Radio 内置 CF32 基线

路径：

```text
CF32 @ 737.28M
→ gr::filter::rational_resampler_ccf(65, 48, taps)
→ CF32 @ 998.4M
→ null sink / existing downstream block
```

用途：

- 建立权威 GNU Radio 行为基线；
- 与 MATLAB 对照滤波器设计、群时延、样本数和 phase；
- 测量通用 block 的真实持续吞吐；
- 为自定义优化 block 提供逐样本 golden。

Level A 可以先用独立 benchmark/QA flowgraph，不需要新增 OOT block。禁止仅用
`Throttle` 或文件 I/O 测量 resampler 性能。

### Level B：固定 65/48 CF32 优化 block

仅当 Level A 未达到最低实时门槛或 profile 明确显示 resampler 是瓶颈时新增：

```text
gr::uwb::UwbRationalResamplerCcf65_48
```

固定合同：

- input：1 路 `gr_complex` stream，737.28 MS/s；
- output：1 路 `gr_complex` stream，998.4 MS/s；
- interpolation：65；
- decimation：48；
- taps：构造期加载，运行期只允许在停流/显式 reset 后替换；
- status message：可选，用于 discontinuity、reset、invalid tag；
- 无 PDU 端口。

### Level C：SC16 生产热路径

Level B 正确但 CF32 输入带宽/转换成为瓶颈时，增加以下候选之一，并用 benchmark
决定保留哪个：

```text
方案 C1：SC16 input → CF32 output，融合 Q15→float 与 polyphase FIR
方案 C2：SC16 input → SC16 output，Q15/定点 polyphase FIR
```

命名建议：

```text
UwbRationalResamplerSc16Ccf65_48   # C1
UwbRationalResamplerSc16Sc16_65_48 # C2
```

优先顺序：

1. 先实现 C1，直接兼容现有 CF32 `UwbScheduledExtractor`；
2. 只有输出 CF32 的约 7.99 GB/s 写带宽被证明是瓶颈时才实现 C2；
3. C2 接入 scheduled 路径前，需要另行增加或复用 SC16 scheduled extractor，不能
   在 flowgraph 中立即把整个 998.4 MS/s 连续流转换回 CF32；
4. 未经 profile 不同时维护三套生产 kernel。

---

## 4. GNU Radio block 类型与 scheduler 语义

### 4.1 正确 block 类型

65/48 是变速流处理，必须使用：

```text
gr::block + forecast() + general_work()
```

不能使用：

- `sync_block`：输入输出不是 1:1；
- `decim_block`：不是纯整数抽取；
- `interp_block`：不是纯整数插值；
- `tagged_stream_block`：输入是无限连续 IQ，不以 packet length tag 分帧；
- message/PDU block：本任务主目标是连续流升采样。

### 4.2 固定宏块合同

稳态下以最简周期处理：

```text
48 input samples → 65 output samples
```

优化 block 应优先：

- `set_relative_rate(65, 48)`，使用当前 GNU Radio 版本支持的整数重载；
- `set_output_multiple(65)`，让 scheduler 尽量提供整数宏块；
- 每次处理 `B` 个宏块：消费 `48B`，产生 `65B`；
- 对 EOS 尾部使用明确、已测试的 drain 规则；
- 使用整数 phase/count，不用浮点累计决定 consume/produce。

如果 GNU Radio history 语义适合 polyphase FIR，可使用 `set_history()`；否则在对象
内部维护固定大小对齐 delay line。二者必须先与本地 GNU Radio 相似 block 对照，
不能同时维护两份重叠历史。

### 4.3 `forecast()`

`forecast(noutput_items)` 必须依据整数比例和 FIR 历史精确计算最少输入：

```text
macroblocks = ceil(noutput_items / 65)
new_input   = macroblocks × 48
```

history/delay-line 所需样本按最终实现方式增加。不得用 `double` 比例向下取整导致
scheduler 活锁，也不得永远额外请求一个完整大 chunk 导致有限流无法结束。

### 4.4 `general_work()`

必须满足：

- 仅处理完整可证明安全的输入/输出范围；
- 调用 `consume_each(actual_consumed)`；
- 返回 `actual_produced`；
- phase、delay line、绝对输入/输出计数跨调用保持；
- chunk 大小变化不改变输出；
- `noutput_items < 65`、EOS 尾部和 reset 均有 QA；
- 热路径不构造 `std::vector`、PMT、string 或临时 taps；
- 构造期完成 taps 分相、反序、对齐和 scratch 分配。

### 4.5 建议内部状态

```cpp
struct ResamplerState {
    uint64_t input_items;
    uint64_t output_items;
    uint32_t phase;
    bool locked;
    bool pending_discontinuity;
};
```

所有公开 sample index 使用 `uint64_t`。运行统计至少提供：

```text
input_items
output_items
resets
discontinuities
tag_errors
```

---

## 5. 多相滤波器设计

### 5.1 算法

数学定义为：

```text
y = downsample(FIR(upsample(x, 65)), 48)
```

实现必须使用 65 相 polyphase FIR，禁止真的插入 64 个零。65 和 48 已互质，不能
进一步约分。

每个输出样本只执行对应 phase arm 的有效 taps。固定 48→65 周期允许预计算：

- 65 个输出的 phase 序列；
- 每个输出相对当前输入 cursor 的 advance；
- 每相 taps 起点、长度和对齐 padding；
- 一个宏块总消费量必须严格等于 48。

### 5.2 滤波器规格

不要直接接受 GNU Radio 自动 taps 作为最终生产滤波器。新增 MATLAB 设计/导出脚本：

```text
testdata/design_resampler_737p28_to_998p4.m
```

脚本至少输出：

```text
interpolation = 65
decimation = 48
input_rate = 737.28e6
output_rate = 998.4e6
taps
passband edge
stopband edge
passband ripple
stopband attenuation
group delay（input/output/virtual-grid samples）
```

第一版建议从以下规格开始，再用真实 UWB 对照调整：

```text
实系数、线性相位 FIR
passband ripple ≤ 0.1 dB
stopband attenuation ≥ 70 dB
passband/transition 必须限制在 737.28 MHz 输入的 Nyquist 范围内
```

最终 passband edge 不能拍脑袋确定。必须同时检查：

1. `testdata/` 已知 UWB 信号的频谱；
2. MATLAB decoder 的相关峰、CFO、CIR、soft chips 和 FCS；
3. tap 数与实时吞吐。

开发记录至少比较两组候选：

- Quality：较长 taps，作为正确性 golden；
- Realtime：满足解调/FCS指标的最短 taps。

若缩短 taps，只能在 QA、频响和真实 golden 全部通过后保留。

### 5.3 增益、舍入与饱和

必须明确插值滤波器 DC 增益。MATLAB、GNU Radio 内置 block 和自定义 block 应使用
同一 taps/scale，避免把幅度差误认为波形误差。

SC16 路径必须定义：

- accumulator 位宽；
- coefficient Q format；
- rounding（不是隐式截断）；
- 对 `[-32768, 32767]` 的饱和；
- full-scale 正弦和接近饱和 UWB 的行为；
- 输出 scaling metadata 或固定 scale 合同。

---

## 6. 时间戳、stream tag 与不连续处理

本任务不能只保证 IQ 数值正确，还必须保持 X410 时间语义。

### 6.1 tag propagation

优化 block 建议设置：

```text
TPP_DONT
```

然后手动传播受支持的 tags，避免 GNU Radio 默认按浮点 relative rate 映射造成
边界歧义。至少处理：

```text
rx_time
rx_rate / sample_rate
rx_freq（值不变）
overflow / discontinuity
用户 schedule/marker tag（若存在）
```

### 6.2 绝对 offset 映射

tag offset 必须使用同一整数 phase 状态映射，不能简单地对每个 offset 单独执行：

```text
round(input_offset × 65 / 48)
```

必须定义 tag 对应的“第一个受影响输入样本”映射到哪个输出样本，并通过 impulse/tag
QA 固定语义。群时延是另一项 metadata，不能通过偷偷移动所有 tag 来掩盖。

### 6.3 `rx_time`

`rx_time` 的物理时间值不因升采样改变；它在输出流上的 tag offset 需要按 resampler
phase 映射。输出 sample period 为：

```text
1 / 998.4e6 s
```

如果输入发生 overflow、时间跳变或 UHD 重启：

1. 不允许继续使用旧 FIR history；
2. 清空 delay line；
3. 重置有理 phase；
4. 增加 discontinuity/reset 计数；
5. 在输出发出明确 tag/status；
6. 下游 `UwbScheduledExtractor` 必须重新 seed 或重映射 schedule。

不能跨缺失数据做看似连续的插值。

### 6.4 坐标 metadata

下游 PDU/写盘至少能追溯：

```text
input_sample_rate = 737.28e6
output_sample_rate = 998.4e6
resample_interp = 65
resample_decim = 48
resample_filter_delay
input_absolute_sample（或对应 rx_time）
output_absolute_sample
discontinuity_epoch
```

---

## 7. 文件与接口建议

Level A 只新增 benchmark/QA 支撑。进入 Level B 后建议新增：

```text
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_ccf_65_48.h
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h
gr-uwb/lib/uwb_rational_resampler_ccf_65_48.cc
gr-uwb/lib/qa_uwb_rational_resampler_core.cc
gr-uwb/lib/qa_uwb_rational_resampler.cc
gr-uwb/python/uwb/bindings/uwb_rational_resampler_ccf_65_48_python.cc
gr-uwb/grc/uwb_rational_resampler_ccf_65_48.block.yml
gr-uwb/apps/benchmark_resampler_65_48.cc
testdata/design_resampler_737p28_to_998p4.m
testdata/export_resampler_65_48_golden.m
docs/performance/测试报告_GNURadio软件升采样_65_48.md
```

如果当前 pybind 采用集中式注册方式，按现有结构调整文件名，不能机械照抄。

核心与 GNU Radio adapter 分离：

```text
uwb_rational_resampler_core.h
    只负责 polyphase、phase、delay line、consume/produce
    不依赖 PMT、GNU Radio scheduler 或文件 I/O

uwb_rational_resampler_*.cc
    只负责 forecast/general_work、tag、统计和 block API
```

GRC 参数第一版只暴露：

```text
taps_file / quality_profile
tag_propagation_enable
reset_on_discontinuity
```

65、48、737.28e6、998.4e6 是当前 fixed-profile 合同，不做伪通用运行时参数。

---

## 8. QA 测试矩阵

实现不能只用“能跑 flowgraph”验收。

### 8.1 core 数值 QA

必须覆盖：

1. **零输入**：输出全零，无 NaN/未初始化数据；
2. **单位冲激**：与 MATLAB `upfirdn(x,taps,65,48)` 对齐；
3. **复冲激**：I/Q 符号、共轭方向正确；
4. **DC**：稳态增益符合定义；
5. **复单音**：至少覆盖低频、通带边缘、阻带；
6. **随机复数**：与 MATLAB golden 比较；
7. **真实 UWB 窗口**：与 MATLAB 重采样逐样本比较；
8. **SC16 极值**：`-32768/32767`、rounding、饱和；
9. **很短输入**：0、1、47、48、49、95、96 个输入；
10. **长输入计数**：输出/消费数量和 phase 无漂移。

浮点误差标准应在 golden 导出后依据 taps/scale 明确写入测试，不能使用过宽的
“大概相同”容差。建议同时报告：

```text
max_abs_error
relative_L2_error
normalized correlation
gain error
phase error
```

### 8.2 chunk invariance

同一输入至少用以下切分方式调用 core/block：

```text
一次性整块
固定 48 samples
固定 4096 samples
1/2/47/48/49/随机 chunk 混合
GNU Radio scheduler 自动 chunk
```

去除统一定义的尾部 drain 后，输出长度、IQ、phase 必须一致。

### 8.3 scheduler QA

覆盖：

- `forecast()` 不活锁；
- `set_output_multiple(65)` 下有限 source 正常 EOS；
- 小 output buffer；
- 多次 `general_work()`；
- `consume_each()` 与返回 produced 精确；
- flowgraph stop/restart；
- taps/reset API；
- 析构不死锁；
- work/general_work 热路径无动态扩容。

### 8.4 tag/rx_time QA

输入 tag 放在：

```text
offset 0、1、47、48、49
跨 work chunk 边界
FIR warm-up 前后
两个连续 rx_time epoch
模拟 overflow/discontinuity 前后
```

验证输出 offset、value、reset 和 epoch。不得只验证 tag 数量。

### 8.5 UWB 算法回归

使用 `testdata/` 已知 UWB 信号，并与 `UWB_demodulation/` MATLAB 实现对照：

```text
packet start
SYNC correlation peak/metric
CFO
SFD position
CIR / soft chips
PHR
PSDU
FCS
```

至少保留一个无噪声 golden 和一个真实/带噪 golden。升采样后的 packet start 必须
明确到 998.4 MHz output sample；同时保留到 737.28 MHz input sample 的反向映射。

---

## 9. Benchmark 设计

### 9.1 禁止的测量方式

不得：

- 从慢磁盘读大文件作为 resampler benchmark；
- 在 flowgraph 中放 `Throttle`；
- 把生成测试信号的时间算入 kernel 吞吐；
- 只跑几十毫秒；
- 只报告输出 MS/s，不报告输入 MS/s；
- 用处理短数组的 cache-hot 数字宣称持续实时；
- 因下游 null sink 没有读数据而误判优化器消除了实际工作。

### 9.2 两层 benchmark

新增 `benchmark_resampler_65_48`，至少支持：

```text
kernel：直接调用 core，预生成并复用大环形输入
flowgraph：GNU Radio source → resampler → checksum/null sink
```

必须比较：

```text
A：GNU Radio rational_resampler_ccf
B：固定 65/48 CF32 block（若实现）
C1/C2：SC16候选（若实现）
```

运行信息：

```text
GNU Radio version
compiler/version/flags
CPU model、ISA、core count
VOLK profile/machine
input/output format
tap count、每相 taps 分布
chunk size/output multiple
input samples、output samples
wall time、CPU time
input MS/s、output MS/s
real-time ratio
estimated payload read/write GB/s
checksum
```

每种模式：

- warm-up 后至少 7 轮；
- 报告 median、min、max；
- 每轮至少处理足以运行 2 秒的数据，但不需要分配数秒全流数组；
- 再做 30 秒 sustained run；
- 观察 RSS 是否增长、输出计数是否漂移。

### 9.3 性能门槛

最低可行门槛：

```text
input throughput  ≥ 737.28 MS/s
output throughput ≥ 998.4 MS/s
30 秒无 backlog、无样本丢失、无计数漂移
```

生产目标留 20% 余量：

```text
input throughput  ≥ 884.736 MS/s
output throughput ≥ 1,198.08 MS/s
```

单通道理论 payload：

```text
SC16 input  @ 737.28M = 2.94912 GB/s
CF32 input  @ 737.28M = 5.89824 GB/s
SC16 output @ 998.4M  = 3.99360 GB/s
CF32 output @ 998.4M  = 7.98720 GB/s
```

报告必须区分 kernel 算力与内存/flowgraph 带宽。如果 kernel 达标但 flowgraph 不
达标，要继续定位 source、buffer copy、format conversion、scheduler 和 sink，不能
继续盲目优化 FIR。

### 9.4 profile 决策

只有 profile 证明瓶颈后才采取对应优化：

| 证据 | 后续动作 |
|---|---|
| 通用 phase/control 占比高 | 固定 48→65 schedule，移除通用分支 |
| FIR dot product 占比高 | VOLK/AVX2，优先实 taps × complex input |
| SC16→CF32 转换占比高 | 融合转换与 FIR |
| 输出写带宽饱和 | 保持 SC16 输出，并适配下游 SC16 |
| work 调用过密 | 增大 buffer、output multiple、max_noutput_items |
| 单 block 单核算力不足 | 先评估 phase/output 并行；最后才考虑有 halo 的有序多 worker |
| 实机 overflow 但合成 benchmark 正常 | 检查 NIC/UHD buffer、CPU affinity、调度和 rx_time |

禁止在一个提交中同时改 taps、SIMD、数据格式和 scheduler；否则无法归因。

---

## 10. 优化实现要求

### 10.1 构造期

允许：

- 校验并归一化 taps；
- 拆成 65 个 polyphase arms；
- 为 SIMD 对齐 padding；
- 预计算 65-output phase/input-advance schedule；
- 分配 delay line 和固定 scratch；
- 选择 scalar/VOLK/AVX2 kernel；
- intern 稳定 PMT tag key。

### 10.2 `general_work()` 热路径

禁止：

- `new/delete`；
- `std::vector::resize/push_back` 导致扩容；
- taps 重排；
- PMT 字典构造；
- 日志逐 chunk 输出；
- mutex 竞争；
- 文件 I/O；
- 每样本求模或浮点 rate 计算。

允许使用预计算表将一个宏块展开为固定 65 个 phase 操作。先保留 scalar reference
kernel，任何 SIMD kernel 都必须逐样本对照 scalar。

### 10.3 SIMD 顺序

1. 编译器自动向量化 + 对齐数据；
2. 检查 GNU Radio FIR/VOLK 是否可直接复用；
3. VOLK；
4. AVX2 fixed-profile kernel；
5. 仅在目标机支持且有独立 fallback 时考虑 AVX-512。

不得删除 scalar fallback。运行时输出实际选择的 kernel。

### 10.4 多线程限制

连续有状态 FIR 很难直接按 chunk 并行。第一版保持单 block、单有序 state。只有单核
kernel 明确不足时，才设计：

- 每块输入 halo；
- 由绝对输入 offset 推导起始 phase；
- 有序输出 reorder；
- discontinuity 统一 reset；
- 固定内存池和有界队列。

多线程版本必须证明输出与单线程 bitwise/容差一致，不能因 chunk 边界改变样本。

---

## 11. Flowgraph 集成

### 11.1 新软件路径示例

不要覆盖现有 RFNoC 示例。新增独立文件，例如：

```text
gr-uwb/examples/x410_software_resampler_uwb_scheduled.grc
gr-uwb/examples/x410_software_resampler_uwb_detector_sc16.grc
```

基线 scheduled flowgraph：

```text
RFNoC RxStreamer CF32 @ 737.28M
→ rational_resampler_ccf 65/48
→ UwbScheduledExtractor CF32 @ 998.4M
→ UwbPacketWriter
```

优化 flowgraph 依据最终胜出的数据格式连接。所有 sample-domain 参数必须使用输出
率 998.4 MHz；UHD/rx_time 输入合同仍记录 737.28 MHz。

### 11.2 buffer 与调度

实测调优：

- source/resampler/downstream `min_output_buffer`；
- resampler `max_noutput_items`；
- 65 的 output multiple；
- GNU Radio buffer 是否造成额外大规模 copy；
- CPU affinity 与实时调度仅作为单独 A/B 项；
- 不允许通过无限增大 buffer 隐藏 backlog。

### 11.3 启动检查

生产 app 启动时必须打印并检查：

```text
actual input rate = 737.28e6
interp/decim = 65/48
declared output rate = 998.4e6
input/output format
tap profile/count
filter delay
selected kernel
UHD rx_time present
```

若实际输入率不等于 737.28e6，不得仍把输出 metadata 标记为 998.4e6。

---

## 12. 执行阶段与停止条件

### Phase 0：调查与基线

- 完成 §2 源码调查；
- 运行现有 CTest；
- 新增内置 CF32 resampler benchmark；
- 记录 Level A 数值和吞吐。

完成标准：报告可复现，明确内置 block 是否达最低门槛。

### Phase 1：滤波器与 MATLAB golden

- 增加 MATLAB taps 设计/导出脚本；
- 导出 impulse、tone、random、真实 UWB golden；
- GNU Radio 内置 block 与 MATLAB 对照；
- 明确 group delay、scale 和 output length。

完成标准：golden 自动测试通过，不靠人工看图。

### Phase 2：固定 65/48 core/block

仅在 Level A 性能不足时执行：

- 先写 core QA；
- 实现 scalar fixed schedule；
- 实现 GNU Radio `block` adapter；
- 完成 chunk、EOS、tag、rx_time QA；
- 构建并运行全部 CTest。

完成标准：与 MATLAB和内置 block 一致，无现有 QA 回归。

### Phase 3：逐项性能优化

- profile；
- 每次只改一个瓶颈；
- scalar/VOLK/AVX2 或 SC16 融合逐项 A/B；
- 每项修改后运行相关 QA、全 CTest 和 benchmark 多轮中位数。

完成标准：达到最低门槛，或以 profile 证明目标主机连续流不可行。

### Phase 4：UWB 与 flowgraph 集成

- 新增软件升采样 GRC 示例；
- testdata 逐 packet/MATLAB 对照；
- scheduled `t0/T` 坐标检查；
- detector packet start 检查；
- 30 秒 sustained flowgraph。

完成标准：packet start、截取范围、FCS/CIR满足 §8，0 drop/backlog。

### Phase 5：X410 实机验收

- 先短时检查 actual rate、rx_time、scale 和频谱；
- 再做 30 秒、10 分钟持续流；
- 统计 UHD overflow、resampler reset、downstream drop、CPU、RSS、queue HWM；
- 人为制造一次 stop/restart 或 discontinuity，验证不会沿用旧 history/schedule。

完成标准：见 §13。

### 停止/转向条件

满足以下任一条件时，不再继续无证据微调，并在报告中建议转向“原生率 scheduled
截窗后只升采样 PDU”或 RFNoC：

1. 已完成 profile 和两种独立 kernel 优化，flowgraph 仍低于实时 15%以上；
2. 输出 CF32 内存带宽已饱和，且 SC16 端到端改造超出本阶段范围；
3. 为达到实时必须降低滤波质量到 MATLAB/FCS/CIR验收失败；
4. 目标主机/NIC无法稳定接收输入流；
5. 连续路径没有至少 10% 可复现余量，长测持续 backlog/overflow。

达到停止条件不是“优化完成”。必须保留正确基线、profile 和可复现报告，明确下一
架构选择。

---

## 13. 最终验收标准

### 13.1 正确性

- 65/48 速率合同精确；
- impulse/random/真实 UWB 与 MATLAB golden 通过；
- 任意 chunk 切分输出一致；
- output count、phase 长时间无漂移；
- packet start 和 IQ 截取范围与 MATLAB逐 packet 对照；
- 已知 golden 的 PHR/PSDU/FCS 不回归；
- `rx_time` 和 discontinuity 行为通过自动 QA。

### 13.2 软件质量

- block 类型和 scheduler 语义在头文件/方案记录中说明；
- core 与 GNU Radio adapter 分离；
- `general_work()` 无动态分配和文件 I/O；
- scalar reference 保留；
- QA 在实现前或随实现同步增加；
- 全部 CTest 通过；
- `git diff --check` 通过；
- 不覆盖用户无关修改。

### 13.3 性能

最低：

```text
≥737.28 MS/s input
≥998.4 MS/s output
30 秒 0 drop、0 count drift、无 backlog
```

生产推荐：

```text
≥1.2× realtime 合成 benchmark
10 分钟实机 0 UHD overflow、0 downstream drop
RSS 无持续增长
```

如果只达到最低门槛而没有余量，结论必须写为“功能可行、生产风险高”，不能写成
“生产实时完成”。

---

## 14. Claude Code 最终交付清单

Claude Code 完成每个 Phase 后更新：

```text
docs/performance/测试报告_GNURadio软件升采样_65_48.md
开发状态.md
```

最终回复和报告必须包含：

1. 修改文件清单；
2. GNU Radio scheduler 语义说明；
3. taps/频响/群时延；
4. MATLAB golden 误差；
5. chunk/tag/rx_time QA；
6. 内置、固定 CF32、SC16候选的公平 benchmark 表；
7. 全部构建与测试命令及结果；
8. 30 秒与实机长测结果；
9. 是否达到最低/生产门槛；
10. 未完成项和明确下一步。

执行纪律：

- 每次 meaningful modification 后构建并运行相关测试；
- 每个优化先给 profile 证据，再实施，再做 A/B；
- 不用单轮峰值下结论，使用多轮中位数；
- 不提交大测试文件或实机 IQ；
- 未通过 MATLAB、QA、持续吞吐三类验收前，不声明完成。

