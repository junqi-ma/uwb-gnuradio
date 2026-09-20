# 开发报告：逐 repetition CIR 实时落盘与 UDP

> 日期：2026-09-20
> 状态：FC32 落盘 + UCR4 block-floating SC16 UDP 代码/离线 QA 完成；X410 吞吐/soak 未验收

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
  `(pulse_id, repetition_ordinal)` 顺序连续排列；JSONL 改为每个 UWB pulse
  一行，公共时间/标定字段只写一次，`repetitions` 对象用等长列数组保存
  `repetition_index/status/tap_count/file_offset/peak/metric/norm/estimator_us`。
  `repetition_records` 和 `repetition_complete` 使 queue drop 或停止时不完整的
  pulse 仍可观测。writer 每 256 条 CIR record 批量 flush，app 队列至少 256 条。
  128-SYNC、116 tap 的典型聚合 JSON 行约 9 kB，即 200 pulse/s 约 1.8 MB/s，
  相比逐 repetition JSONL 的约 20 MB/s 显著下降；二进制 CIR 仍是 FC32。
- UDP 协议升级为 UCR4：每个 datagram 仍只承载一个 repetition CIR。52-byte
  header 增加 `repetition_index/count` 和 `cir_scale f32`，payload 为 116 个交错
  little-endian SC16 tap（464 bytes），总长 516 bytes。量化采用每 CIR
  block-floating scale：`FC32 = SC16 × cir_scale`，因此保留 raw CIR 绝对幅度。
  `cir_udp_recv.py` 解码后仍向调用者提供重建的 complex64 `taps`，并继续兼容
  UCR3/UCR2/UCR1/raw。磁盘 `cir.cf32/cir_norm.cf32` 不量化、保持 FC32。
- MATLAB reader 在内存中把列数组展开为兼容的逐 repetition metadata；仍可用
  `read_uwb_cir(..., repetitionIndex)` 读取指定 repetition，或用
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

结果：targeted CTest 4/4、UDP 8/8、jam args 15/15、jam plan 75/75。
新增数值硬门证明 `mean(individual CIR)` 与原 averaged CIR 的 relative L2 `<2e-6`；
block QA 证明 64-SYNC/skip10 恰好输出 54 条，index 10..63 且顺序稳定。
实际 `CirUdpSink → localhost UDP → cir_udp_recv` loopback 为 516 bytes，重建
relative L2 `1.52e-5`（门限 `<1e-4`）。
全量 CTest 42/43；唯一失败仍为既有、环境相关的 PDU resampler 吞吐阈值
（本轮约 217 PDU/s，门限 >400），与 CIR 改动无关。

## 4. 未完成与性能边界

- 未安装到系统 GNU Radio；运行硬件前仍需按项目既有流程 install。
- 未做 X410 repetition 模式 smoke/200 Hz/300 Hz soak，不得声称硬件验收通过。
- 128-SYNC 时 CIR record、PMT 和 UDP datagram 数仍为 128/pulse；每条 UCR4
  datagram 为 516 bytes（52 + 116×4），200 Hz 为 25.6 kdatagram/s、约
  13.2 MB/s（约 106 Mbit/s，不含链路开销）。每包小于普通 MTU，不触发 IP 分片。
- 当前 UDP 是非阻塞 best-effort，`udp_eagain` 是真实丢包计数。需要无丢失网络传输时，
  后续应设计带 sequence/chunk 的批量协议或可靠传输，而不是隐藏 drop。
