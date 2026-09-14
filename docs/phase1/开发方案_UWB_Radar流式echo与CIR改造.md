# 开发方案：UWB Radar 流式 echo / CIR 改造（gr-radar 风格）

> 状态：**M1/M2/M3 完成，已上板验证**
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

### M2（完成）C++ 流式 echo

新增 `gr-uwb/include/gnuradio/uwb/uwb_echo_timer_stream.h` +
`gr-uwb/lib/uwb_echo_timer_stream.cc`：

- `UwbEchoTimerStream`：`gr::tagged_stream_block`，1 入 TX complex64 tagged
  stream（每包一个 burst）/ 1 出 RX complex64 tagged stream（每窗一个）。
- `work()` 每调用一个 burst，阻塞到 `collect_result()`；**复用**
  `EchoGrid`/`plan_fragments`（`uwb_echo_scheduler_core.h`）与
  `IRadioBurstBackend`（`uwb_echo_burst_backend.h`），逻辑对齐
  `UwbRealtimeEchoTimer::apply_schedule/run_one_burst`（RX 命令先于 TX）。
- TX complex64→SC16（×32768 截断）固定 scratch；RX SC16→complex64（÷32768）；
  固定 fragment 数组，work 热路径无分配。
- 输出 tag：`pulse_id`、`schedule_index`、`sample_rate`、
  `window_start_sample=0`、`pre_guard_samples`、`capture/post_guard_samples`、
  `sample_count`、`calibration_delay_native_samples`、`sync_repetitions`、
  `sfd_mode`、`code_index`、成功时 `rx_time`（`packet_len` 由基类加）；
  失败窗补零并发 `burst_status` tag，流不中断。
- 线程安全 `set_freq(hz)` / `set_cal_delay_native(v)`（原子），供 sweep 与
  首峰 servo；`bursts_ok/failed`、`late_slot_skips`、`last_error()` 等计数。
- `make_uhd(...)` 便捷工厂（`#ifdef UWB_HAVE_UHD`）内部构造
  `UhdBurstBackend`；Python 绑定 `uwb.echo_timer_stream_uhd(...)`。
- `IRadioBurstBackend` 新增纯虚 `tune(freq, err)`，`FakeBurstBackend` 与
  `UhdBurstBackend`（`set_tx_freq`/`set_rx_freq` + 回读）实现。
- QA：`gr-uwb/lib/qa_uwb_echo_timer_stream.cc`（`FakeBurstBackend` 流图，
  校验输出长度/tag/计数），CTest 通过。

### M3（完成）接线 + buffer

新增 `gr-uwb/apps/x410_cg400_hrp_echo_cir_stream.py`（+ 纯 Python 单测
`test_echo_stream_buffers.py`）：

```
vector_source_c(TX, repeat) → stream_to_tagged_stream("packet_len")
  → uwb.echo_timer_stream_uhd → uwb.rational_resampler_ccf_65_32
  → blocks.tagged_stream_to_pdu(complex, "packet_len")
  → uwb.radar_cir_estimator → cir_writer / CirUdpSink / PeakAlignSink
```

- 没有 Python 发布线程/`_pub_q`，不再 `tolist()/init_c32vector`。
- `--freq-mode fixed|scan|manual`：后台线程按 dwell 调 `echo.set_freq(hz)`；
  首峰 servo 订阅 `est.cir`，回写 `echo.set_cal_delay_native(...)`。
- buffer（`set_min_output_buffer(port, items)`，单位 items）：

| 位置 | 取值 | tx=94492, rx=109288 |
|---|---|---|
| `stream_to_tagged_stream` 输出 | `2·tx` | 188984 |
| echo 输出 | `4·rx` | 437152 |
| resampler 输出 | `2·ceil(rx·65/32)` | 443984 |
| `tagged_stream_to_pdu` 输入 | 随 resampler 输出（同一连接取大） | 443984 |

- **关键修正 1（UHD）**：`UhdBurstBackend` 的 streamer CPU 格式写成了
  `"s16"`，UHD 实际 token 是 **`"sc16"`**（否则 prepare 报
  `Cannot find a conversion routine`）。已改为 `("sc16","sc16")`。
- **关键修正 2（收尾丢帧）**：echo `work()` 返回 `WORK_DONE` 后 GNU Radio
  会拆掉消息端口，最后一帧 CIR 在投递前丢失。因此 echo 跑
  `target_frames + 8` 个 burst（保持流图存活），app 在 `target_frames` 处
  退出并把多余的 slack 帧从 `cir.jsonl`/`cir.cf32` 截掉。

## 5. 实测结果（2026-09-13，X410 CG400，491.52 MS/s）

命令示例：

```bash
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_stream.py \
  --args addr=192.168.10.2 --freq-mode fixed --pulses 8 --pri-s 0.05 \
  --gain-tx 50 --gain-rx 60 --no-udp --output /tmp/x410_stream
```

| 项目 | 流式链 | 旧 PMT/PDU 链（同参数） |
|---|---|---|
| 突发 | 8/8 ok，late=0 | 12/12 ok，late=0 |
| resampler tag_errors | 0 | — |
| CIR | 8/8 ok | 12/12 ok |
| `peak_tap` | 45–46 | 45–46 |
| `cir_peak_metric` mean | 0.0727 | 0.0716 |
| 估计器 service | ~380 µs | ~354 µs |

- **CIR 与旧链一致**（peak_tap 同、metric 同量级），说明流式重采样 + tag 映射
  数值正确。
- 首峰 servo（`--peak-target-tap 30`）在流式链上工作：`align_locked=true`，
  `last_first_peak=30`，`cal_delay_native 334 → 342.37`。
- scan（`--freq-start 6489.0e6 --freq-stop 6489.8e6 --freq-step 0.4e6
  --freq-dwell 10`）：`retune=2`，每频点 10 帧，`cir_ok=30`。
- CTest：除已知的 `uwb_qa_uwb_pdu_rational_resampler.cc` 吞吐阈值（本机慢）
  外，**37/37 全过**；`test_echo_stream_buffers` 8/8、`test_freq_plan` 35/35、
  `test_cir_udp_format` 5/5。

### 5.1 处理速度：stream vs 旧 PDU 链

同一配置（preamble 128、minphase、gain 50/60、`--no-udp`），看 `late`（错过
调度槽数）与 `schedule_wall_s`（墙钟，接近 `帧数×PRI` 为达标）：

| 速率 | 旧 PDU 链 | 流式链（res-workers=1） | 流式链（res-workers=16） |
|---|---|---|---|
| 100 Hz | 0 late，实时 | 0 late，实时 | — |
| 200 Hz | **1** late，1.247 s | **196** late，2.253 s | **0** late，1.252 s |
| 500 Hz | 102 late，0.651 s | 871 late，2.303 s | 939 late，2.955 s |

结论：

- **低速（≤100 Hz）**：两条链都能实时，CPU 相当；流式链去掉了 Python
  `tolist()+init_c32vector`（GIL 热点），但 C++ SC16↔fc32 与 tag 机制抵消了。
- **中速（200 Hz）**：流式链默认 1 个 FIR worker 时被 **65/32 重采样 FIR**
  卡住，回压到 echo 的调度 → 大量 `late`；把重采样器配置成多 worker
  （`--res-workers 4…16`）后 **与 PDU 链持平（late=0，1.252 s）**。
  单 worker 是本链真正的吞吐瓶颈：每窗 109288→222k 点的多相 FIR。
- **高速（500 Hz）**：两条链都跟不上（每 burst UHD I/O ~3–7 ms）；
  流式链的 C++ `UhdBurstBackend` 每 burst 开销更大，暂不如 PDU 链。

**所以：目前流式版并没有提升处理速度**——它解决了原 PDU 链的 Python/GIL
`U` underflow 隐患并更接近 gr-radar 结构，但把瓶颈暴露成「每窗 FIR」与
「每 burst C++ UHD 后端开销」。已加 `--res-workers`（默认 4）让中速达标；
要进一步提升需要：优化 65/32 FIR（更小 `realtime_minorder` taps / 更多
worker / AVX）、以及精简 `UhdBurstBackend` 的每 burst 路径。

## 6. 验收与风险

**验收**

- M1：与 PDU 重采样逐样本一致（已通过）+ CTest。
- M2/M3：同一段抓下来的 native 窗，分别走「旧 PDU 链」与「新 stream 链」，
  CIR（`peak_tap` 与 taps）逐样本一致；100 Hz / 干扰 soak 统计 `U`/`O`/`late`/
  `est_drop` 明显下降。

**风险**

- C++ echo 需要链接 UHD（`ENABLE_UHD_BACKEND`）；本机曾用
  `/tmp/uhd_eal_noret` 绕过 DPDK/AVX-512（现已废弃，DPDK 库与 libuhd 已按
  AVX2 重编，见 `docs/DPDK_X410_CG600启用.md`）。现有 `UhdBurstBackend`
  已经把 timed burst、partial send/recv、error 映射做好，优先复用。
- 流式重采样若连续（不逐窗 reset）会带来 FIR 相位/窗间串扰，故 M1 选择逐窗
  独立；代价是每窗多一次 reset/flush（可接受）。
- retune 后前 ~4 拍群时延 settling（见首峰对齐文档）在 M2 仍要处理。

## 7. 相关文档

- 使用手册（频率扫描 + 首峰对齐）：
  [`使用手册_X410_CG400频率扫描_echo_cir_sweep.md`](使用手册_X410_CG400频率扫描_echo_cir_sweep.md)
- 基础链路说明：
  [`使用说明_X410_CG400自发自收雷达.md`](使用说明_X410_CG400自发自收雷达.md)
- 需求（原始）：
  [`开发需求_UWB自发自收Radar.md`](开发需求_UWB自发自收Radar.md)
