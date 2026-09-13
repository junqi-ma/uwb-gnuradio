# 开发方案：UWB Radar 流式 echo / CIR 改造（gr-radar 风格）

> 状态：**进行中（M1 完成，M2/M3 未开始）**
> 分支：`feature/uwb-monostatic-radar`
> 日期：2026-09-13
> 范围：只改造 **UWB radar echo-CIR 链**（`x410_cg400_hrp_echo_cir*.py` 对应的链路），
> 不动通信链（`x410_auto_scheduled_capture.py`）和盲捕获链。

---

## 1. 背景

当前雷达 echo-CIR 链的数据通路完全是 **PMT/PDU（GNU Radio message）**：

```
TimedUhdEcho (Python basic_block, UHD rx_stream.recv → numpy)
  -- msg "rx": PMT (meta dict, c32vector rx_len) -->
UwbPduRationalResamplerCcf65_32 (gr::block, PDU)
  -- msg "packet" -->
UwbRadarCirEstimator (gr::block, PDU)
  -- msg "cir" --> UwbCirWriter + CirUdpSink (+ PeakAlignSink)
```

问题：

- 发布线程每脉冲要把整个 RX 窗（preamble 128 时 141300 native）转成 PMT
  c32vector（`rx.tolist()` + `pmt.init_c32vector`，实测约 4–7 ms，全程持 GIL），
  100 Hz 下挤占定时 TX 循环 → UHD 报 TX underflow（`U`），retune 处更集中。
- 已用 `--publish-native`（默认 `-1` 自动，只发 CIR 需要的段）把 `U` 从 142 → 5，
  但每包仍有 Python/GIL 转换，属于治标。

对照 **gr-radar**（`kit-cel/gr-radar`）的处理方式：

- 回波处理全程用 **stream / tagged stream**（`usrp_echotimer_cc` 是
  `tagged_stream_block`，1 入 TX tagged stream，1 出 RX tagged stream），
  块与块之间靠 buffer + tag 传递，**不逐包造 PMT**；
- 只在**峰值/目标属性**这一层才转 PMT 消息。

## 2. 需求与目标

1. **echo 用 C++** 实现，提升性能，去掉 Python/GIL 热点。
2. **UHD 输出采用 stream / tagged stream**：每个 RX 窗作为一个 tag 定界的数据块，
   meta 以 stream tag 承载。
3. **Resample 与 CIR 分成两步**：流式 65/32 重采样块 + CIR 块；
   **算出 CIR 后再发布 PMT/PDU**（writer / UDP / 首峰 servo 不变）。
4. **TX 改成和 gr-radar 一致**：TX 波形以 tagged stream 送入 echo。
5. **只改造 UWB radar echo 链**。
6. **给 UWB radar 模式每个步骤设置充足 buffer**，降低 U/O（TX underflow /
   RX overflow）风险。

## 3. 目标架构

```
[TX 波形 tagged stream]  (如 vector_source + stream_to_tagged_stream)
        │
        ▼
UwbEchoTimerStream            (C++；owns USRP；一次 work() 一个定时 burst)
   输出: RX window tagged stream (fc32), tags:
     packet_len=rx_len, pulse_id, schedule_index, sample_rate(491.52e6),
     window_start_sample=0, pre_guard_samples(native), capture/post_guard_samples,
     calibration_delay_native_samples, sync_repetitions, sfd_mode, code_index,
     rx_time
        │  stream
        ▼
UwbRationalResamplerCcf65_32  (C++, tagged_stream_block, 491.52→998.4)
   每窗独立 reset+process+flush；把 radar tag 按同一 map 规律映射到 work 域
        │  stream (998.4, tag 已映射)
        ▼
blocks.tagged_stream_to_pdu(complex_t, "packet_len")   (GR 自带)
   由 tag 组成 meta dict，构造 cons(meta, c32vector)
        │  PDU
        ▼
UwbRadarCirEstimator (现有，不改) → UwbCirWriter / CirUdpSink / PeakAlignSink
        │ msg
        └── servo 回写 → echo.set_cal_delay_native()
```

要点：`blocks.tagged_stream_to_pdu` 的语义正是「第一元素是所有 tag 组成的字典，
第二元素是数据向量」，因此只要 echo + resampler 把现有 PDU 路径那套 meta 作为
stream tag 发出，现有 `UwbRadarCirEstimator` 无需改动即可继续发 `cir` PDU。

复用仓库已有的 C++ 资产：`UwbRealtimeEchoTimer`、`UhdBurstBackend`
（`IRadioBurstBackend`）、`EchoGrid`（`uwb_echo_scheduler_core.h`）。

## 4. 分阶段计划

### M1（完成）流式 65/32 重采样块

- 新增 `UwbRationalResamplerCcf65_32`：`gr::tagged_stream_block`，1 入 1 出
  fc32，`packet_len` 定界，每窗 `reset + process + flush`（与一次性 PDU
  重采样数值一致，且与 PRI 无关）。
- radar tag 映射：复用 `radar_meta::apply_radar_whitelist` 与 core 的
  `map_input_offset_to_output`，输出 `window_start_sample`、`pre_guard_samples`、
  `capture/post_guard_samples`、`sample_rate=998.4e6`、
  `calibration_delay_work_samples`，并透传 `pulse_id/schedule_index/
  sync_repetitions/sfd_mode/code_index`。
- 输出 buffer 至少容纳一个满窗（`set_min_output_buffer(0, 1<<20)` items）。

**产物（commit `02882da`）**

| 文件 | 说明 |
|---|---|
| `gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_ccf_65_32.h` | 块声明 |
| `gr-uwb/lib/uwb_rational_resampler_ccf_65_32.cc` | 实现 |
| `gr-uwb/lib/qa_uwb_rational_resampler_65_32_stream.cc` | CTest |
| `gr-uwb/python/uwb/bindings/python_bindings.cc` | `uwb.rational_resampler_ccf_65_32` 绑定 |
| `gr-uwb/grc/uwb_rational_resampler_ccf_65_32.block.yml` | GRC |
| `gr-uwb/lib/CMakeLists.txt`、`gr-uwb/grc/CMakeLists.txt` | 构建注册 |

**验证（已完成）**

- 与 PDU 重采样器在同一 native 窗上逐样本对比：`max|diff| = 7e-7`（浮点舍入）。
- tag：`window_start=map(0)=42`、`pre_guard=map(983)-map(0)=1997`、
  `cal_native 334 → cal_work 678.438`、`sample_rate=998.4e6`、`packet_len=8403`。
- `ctest -R uwb_qa_uwb_rational_resampler_65_32_stream` **Passed**。

### M2（未开始）C++ 流式 echo

新增 `UwbEchoTimerStream`（`gr::block` 或 `tagged_stream_block`）：

- **输入**：TX native tagged stream（每包一个 burst）。
- **输出**：RX tagged stream（SC16→fc32），带 M1 需要的 native tag（见第 3 节）。
- `work()` 内驱动 `EchoGrid` + `IRadioBurstBackend`：`issue_rx()` →
  `issue_tx()` → `collect_result()`，每调用一个 burst（阻塞到该 burst 完成，
  与 gr-radar `usrp_echotimer_cc::work()` 一致）。
- **sweep 相关接口**：需要 `set_freq(hz)`（运行时 retune）与
  `set_cal_delay_native(v)`（首峰 servo 回写）；都要做成线程安全（原子/短锁），
  由 Python app 在 burst 边界调用。
- 复用现有 `UwbRealtimeEchoTimer` 的调度/转发语义（`apply_schedule`/
  `run_one_burst`）与 `UwbBurstToStream`（若选择先做桥接方案）。

> 决策点：是否先做 **桥接方案** `UwbRealtimeEchoTimer`（消息、现有）→
> `UwbBurstToStream`（PDU→tagged stream，新）→ M1 → `tagged_stream_to_pdu` →
> 现有 estimator。桥接改动小、复用现成 UHD 引擎，但 TX 仍走 schedule PDU，
> 不完全满足「TX 如 gr-radar」。全流式 `UwbEchoTimerStream` 更贴合需求但改动大。

### M3（未开始）接线 + buffer

- app flowgraph：
  `vector_source(TX) → UwbEchoTimerStream → UwbRationalResamplerCcf65_32 →
   blocks.tagged_stream_to_pdu → UwbRadarCirEstimator → writer/UDP/servo`。
- 去掉 Python 发布线程与 `_pub_q`（不再有 `tolist()/init_c32vector`）。
- buffer（`set_min_output_buffer(port, items)`，单位是 **items**）：

| 位置 | 建议 | preamble 128 示例 |
|---|---|---|
| echo TX 输入 | `2·tx_native` | 253k items ≈ 2 MB (sc16) |
| echo RX 输出 | `4·rx_len` | 565k items ≈ 4.5 MB |
| resampler 输入 | `2·rx_len` | 283k items |
| resampler 输出 | `2·map(rx_len)` | ~574k items ≈ 4.6 MB |
| `tagged_stream_to_pdu` 输入 | `1·map(rx_len)` | ~287k items |
| estimator job 队列 | 保持 64 | — |

  必要时 `set_max_output_buffer`；若使用 vmcircbuf，按 gr-radar 文档调整
  `kernel.shmmax` / `net.core.rmem_max`。

## 5. 验收与风险

**验收**

- M1：与 PDU 重采样逐样本一致（已通过）+ CTest。
- M2/M3：同一段抓下来的 native 窗，分别走「旧 PDU 链」与「新 stream 链」，
  CIR（`peak_tap` 与 taps）逐样本一致；100 Hz / 干扰 soak 统计 `U`/`O`/`late`/
  `est_drop` 明显下降。

**风险**

- C++ echo 需要链接 UHD（`ENABLE_UHD_BACKEND`）；本机有 DPDK/AVX-512 的坑，
  可用 `/tmp/uhd_eal_noret` 绕过。现有 `UhdBurstBackend` 已经把 timed
  burst、partial send/recv、error 映射做好，优先复用。
- 流式重采样若连续（不逐窗 reset）会带来 FIR 相位/窗间串扰，故 M1 选择逐窗
  独立；代价是每窗多一次 reset/flush（可接受）。
- retune 后前 ~4 拍群时延 settling（见首峰对齐文档）在 M2 仍要处理。

## 6. 相关文档

- 使用手册（频率扫描 + 首峰对齐）：
  [`使用手册_X410_CG400频率扫描_echo_cir_sweep.md`](使用手册_X410_CG400频率扫描_echo_cir_sweep.md)
- 基础链路说明：
  [`使用说明_X410_CG400自发自收雷达.md`](使用说明_X410_CG400自发自收雷达.md)
- 需求（原始）：
  [`开发需求_UWB自发自收Radar.md`](开发需求_UWB自发自收Radar.md)
