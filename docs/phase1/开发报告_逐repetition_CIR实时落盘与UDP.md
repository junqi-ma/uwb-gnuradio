# 开发报告：逐 repetition CIR 实时落盘与 UDP

> 日期：2026-09-20
> 状态：代码与离线 QA 完成；X410 吞吐/soak 未验收

## 1. 目标与调度语义

原链路对选中的 SYNC repetitions（旧平均模式跳过前 10 个）先相干平均，每个 pulse
只输出一个 CIR。本改动让普通 X410 CIR 与 jamming app 默认输出每个选中
repetition 的独立 CIR。

`UwbRadarCirEstimator` 和 `UwbCirWriter` 都是零 stream 端口的 message/PDU
`gr::block`：输入 handler 只校验/入有界队列，单 worker 保序计算或写盘，不参与
GNU Radio stream scheduler 的 `consume/produce`。一个输入 RX PDU 在 repetition 模式
下展开为 `min(cir_repetitions, sync_repetitions-cir_skip_initial)` 个 CIR PDU。

## 2. 实现

- `estimate_radar_cir_repetition()`：复用 prepare 阶段分配的 code/tap scratch，热路径
  不分配；利用 sampled HRP code 的稀疏性跳过零 chip。
- estimator 新参数 `emit_individual_repetitions`（默认 `false`，调用端源码兼容追加；
  C++ 符号已变化，系统安装必须同步更新）；
  每条成功输出增加：
  - `cir_output="repetition"`
  - `repetition_index`：SYNC 中的绝对 0-based index
  - `repetition_ordinal`：本次选择集合中的 0-based 序号
  - `repetition_count`：该 pulse 应输出的 repetition 数
  - `valid_repetitions=1`
- 普通 `x410_cg400_hrp_echo_cir.py` 与 jamming app 默认
  `--cir-output repetitions`；可用 `--cir-output average` 回到原相干平均模式。
  repetition 模式从 index 0 开始输出全部 SYNC；average 回退模式保留旧 skip10。
  sweep/stream 的 peak/frequency servo 仍固定使用 average，避免把一个 pulse 的
  多个 repetition 误当成多次 servo 观测；这两个入口显式请求 repetitions 会拒绝。
- writer 仍输出 `cir.cf32`、`cir_norm.cf32`、`cir.jsonl`。二进制按
  `(pulse_id, repetition_ordinal)` 顺序连续排列，JSONL 提供逐记录 offset 和上述索引。
  writer 改为每 256 条批量 flush，app 队列至少 256 条。
- UDP 协议升级为 UCR3（48-byte header + 116 complex64 taps），在 UCR2 基础上增加
  `repetition_index u16` 与 `repetition_count u16`；`cir_udp_recv.py` 继续兼容
  UCR2/UCR1/raw。
- MATLAB 可用 `read_uwb_cir(..., repetitionIndex)` 读取指定 repetition，或用
  `read_uwb_cir_repetitions()` 一次得到 `tap_count × repetition_count` 矩阵。

128-SYNC 默认 repetition 模式 `skip=0, cir_repetitions=auto`，所以每个 pulse
输出 128 条 CIR（index 0..127）。`--cir-output average` 保留旧配置：skip10，
用 index 10..127 的 118 次相干平均生成一条 CIR。

## 3. 验证

```bash
cmake --build gr-uwb/build -j 4
ctest --test-dir gr-uwb/build --output-on-failure \
  -R 'uwb_qa_uwb_(radar_cir_estimator|cir_writer)'
python3 gr-uwb/apps/test_cir_udp_format.py
python3 gr-uwb/apps/test_jam_app_args.py
python3 gr-uwb/apps/test_echo_cir_jam_plan.py
python3 testdata/uwb_radar/verify_cir_individual.py
```

结果：targeted CTest 3/3、UDP 6/6、jam args 15/15、jam plan 75/75。
新增数值硬门证明 `mean(individual CIR)` 与原 averaged CIR 的 relative L2 `<2e-6`；
block QA 证明 64-SYNC/skip10 恰好输出 54 条，index 10..63 且顺序稳定。
全量 CTest 42/43；唯一失败仍为既有、环境相关的 PDU resampler 吞吐阈值
（本轮约 217 PDU/s，门限 >400），与 CIR 改动无关。

## 4. 未完成与性能边界

- 未安装到系统 GNU Radio；运行硬件前仍需按项目既有流程 install。
- 未做 X410 repetition 模式 smoke/200 Hz/300 Hz soak，不得声称硬件验收通过。
- 128-SYNC 时 CIR record、PMT 和 UDP datagram 数放大 128 倍；每条 datagram 为
  976 bytes（48 + 116×8），200 Hz 为 25.6 kdatagram/s、约 25 MB/s（不含链路开销）。
- 当前 UDP 是非阻塞 best-effort，`udp_eagain` 是真实丢包计数。需要无丢失网络传输时，
  后续应设计带 sequence/chunk 的批量协议或可靠传输，而不是隐藏 drop。
