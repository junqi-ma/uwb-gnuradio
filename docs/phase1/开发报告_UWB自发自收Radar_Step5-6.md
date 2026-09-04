# 开发报告：UWB 自发自收 Radar Step 5–6

> 日期：2026-09-05
> 分支：`feature/uwb-monostatic-radar`
> 对照评审：[Review_UWB自发自收Radar_Step5-6.md](Review_UWB自发自收Radar_Step5-6.md)
> 状态：**R1–R4 已按 Codex 整改落地，申请复验 Step 5–6。不是 Phase A / EchoTimer / UHD / X410 完成。**
> 未改 [`../../开发状态.md`](../../开发状态.md)。未开始 Step 7。

---

## 1. 复验结论

| 项目 | 结论 |
|---|---|
| R1 Loopback profile | `handle_tx` 在任何 IQ/scratch 处理前校验 rate/dtype/SYNC/SFD/code/长度；失败只发 status，无 RX PDU |
| R2 固定 scratch | `make()` 按 `max_input_samples` 一次分配 input/output scratch；handler 无 `resize`/`reserve` |
| R3 whitelist + 极值 | 白名单字段表驱动断言；坐标走 `radar_i64_*` + `try_map`；溢出/`uint64>INT64_MAX` → `invalid_metadata` |
| R4 PacketSource | native 必须 sidecar/descriptor；SC16 有独立 golden；过短/错配 `make()` 失败 |
| 目标 QA | **3/3 Passed** |
| 全量 CTest | **23/23 Passed** |
| `git diff --check` | 干净 |

未宣称 Step 7、EchoTimer、UHD、X410 或 Phase A。Loopback 仍是纯软件信道。

---

## 2. 块类型与调度语义

对照本机 GNU Radio：`blocks/message_strobe.h` 是 host timer 消息源，**不用**它产生 PRI。

| 块 | 类型 | 端口 | 热路径 |
|---|---|---|---|
| `UwbPduRationalResamplerCcf65_48` | `gr::block` | `packet`→`packet`/`status` | 固定 scratch 上 process+flush；白名单映射 |
| `UwbRadarPacketSource` | `gr::block` | `emit`→`tx`/`status` | `make()` 加载完整包；handler 只改 pulse_id |
| `UwbLoopbackEcho` | `gr::block` | `tx`→`rx`/`status` | `make()` 分配 max RX/TX scratch；超限不扩容 |

---

## 3. R1 — Loopback profile validator

第一版 Loopback 只接受 998.4 MHz CIR 工作域。`validate_loopback_profile()` 在 IQ 处理之前执行：

| 条件 | status | 行为 |
|---|---|---|
| 缺/`sample_rate` ≠ 998.4 MHz | `bad_input_rate` | 不发 RX |
| payload 与 `sample_format` 不一致；SYNC 非 32/64/128；未知 SFD；code 非 9–12；长度字段与 payload 不符；负 guard | `invalid_profile` | 不发 RX |
| `n_tx > max_tx` 或 RX 窗超 `max_rx` | `invalid_window` | 不发 RX（原契约） |
| 非 PDU / 非法向量 | `invalid_input` | 不发 RX（原契约） |

缺 `sync_samples`/`sfd_samples` 时按已验证 profile 补全（`reps*1016`、`|SFD|*1016`）。**不会**把非法输入改写成默认 64/4z2/code9。

表驱动 QA：缺/错 rate、format 不一致、SYNC=0/16、SFD=`nope`、code 8/13、长度不符、负 guard。每例 `pdus_emitted==0`、`pdus_dropped==1`、恰一条稳定 status。既有整数/分数/多径/噪声 QA 保留。

---

## 4. R2 — PDU resampler 固定 scratch

`make()` / `make_from_taps()` 增加公开配置 `max_input_samples`，默认 **262144**（覆盖现有 scheduled e2e 窗 252000 与 Radar 75 µs 窗）。

构造时：

```text
d_input_scratch_.assign(max_input_samples, 0)
d_scratch_.assign(expected_output_length(max_input, T), 0)
```

`max_output_samples` 是一次 process+flush 的 `expected_output_length` 上界，不是经验 +256。handler：

- `n_in > max_input_samples` → `invalid_window`，0 输出
- 输出容量不足 → `internal_error`（内部契约；正常窗口不应触发）
- 无 `resize` / `reserve` / `ensure_scratch`

QA：SC16/FC32 多次打满允许窗，`data()`/`capacity()`/`size()` 不变；`max+1` CF32 与 SC16 只 status、无扩容。

约 75 µs native 窗（55650 @737.28）本机，**warmup 一次后**稳态：

```text
warmup_us=1037  steady_mean_handler_us=1166  p95_us=1477  p99_us=1700  n=32
```

首次建容不计入稳态。

---

## 5. R3 — metadata 完整性与坐标极值

白名单表驱动断言覆盖：`pulse_id`、`schedule_index`、TX/RX time、`num_delay_samps`、`calibration_id`、`code_index`、`uhd_error`、`sfd_samples`、`range_guard_samples`、`tx_packet_samples`、`rx_capture_samples`、三个 index 键及全部 `*_native`。

- `calibration_delay_native_samples` 允许小数；work = `native * 65/48`，门限 `1e-9`
- 32/64/128 的 Radar metadata 映射均有独立用例
- 坐标加减与 `map` 参数走 `uwb_radar_checked_math.h`；`try_to_i64` 拒绝 `uint64 > INT64_MAX`
- 负 guard、`sample_count < pre+capture`、`INT64_MAX`/`INT64_MIN` window、溢出 predicted SFD → `invalid_metadata`，0 输出
- 旧 scheduled 字段语义不变。`detected_start_sample < 0` 仍是“未检测”哨兵，跳过映射，不丢 PDU

---

## 6. R4 — PacketSource native / SC16

- native 737.28 MHz：**必须** `metadata.json` sidecar 或显式 `descriptor_path`。sidecar 校验 `rate_*`、`sync_repetitions`、`sfd_mode`、`code_index`、`tx_length_*` 与文件样点数。非空短文件不能当完整包。
- work 998.4 MHz：仍要求 `SYNC+SFD` 最小长度；若 sidecar 存在则同样绑定。
- `sync_repetitions` 仅 32/64/128。
- SC16 golden：`testdata/uwb_radar/tx_998p4.sc16` / `tx_737p28.sc16`，LE 交织 int16，`round(clip(x*32767))`；`tx_998p4_sc16_head16.i16` 核对字节序。PDU 类型为 `s16vector`。
- 描述符现在强制绑定 `sample_format`、`dtype`、`byte_order`、`layout` 和
  `bytes_per_complex`：FC32 是 `fc32/complex64/8`，SC16 是 `sc16/int16/4`，
  均为 LE `interleaved_iq`。同目录存在 `metadata.<format>.json` 时优先使用；
  因而 FC32 `metadata.json` 不会再被 SC16 文件误用。
- 错误 dtype、字节序、layout、格式描述符、奇数字节、过短 native、profile 与
  sidecar 不符：`make()` 抛异常。QA 构造两份都恰为 4 complex samples 的 native
  文件，验证即使长度相等，FC32↔SC16 descriptor 交叉也必定失败。
- PacketSource **不**做 65/48；native TX 仍由 Step 5 PDU 块处理。

---

## 7. 稳定 status 一览

| event | 块 | 含义 |
|---|---|---|
| `invalid_input` | 三者 | 非 PDU / 空或非法向量 |
| `bad_input_rate` | resampler / Loopback | 缺或错误 sample_rate |
| `invalid_profile` | Loopback | dtype/SYNC/SFD/code/长度/负 guard |
| `invalid_window` | resampler / Loopback | 超过 `max_*` |
| `invalid_metadata` | resampler | 坐标溢出或不可映射 |
| `internal_error` | resampler | 输出 scratch 不足（内部契约） |
| `short_guard` | resampler | 既有 FIR 群时延警告，仍输出 |
| `invalid_emit` / `invalid_waveform` | PacketSource | 既有 emit 失败 |

---

## 8. 测试

```bash
ctest --test-dir gr-uwb/build -R 'qa_uwb_pdu_rational_resampler|qa_uwb_radar_packet_source|qa_uwb_loopback_echo' --output-on-failure
ctest --test-dir gr-uwb/build --output-on-failure
git diff --check
```

全量 CTest：**23/23 Passed**。`git diff --check` 干净。

| QA | 结果 |
|---|---|
| `uwb_qa_uwb_pdu_rational_resampler.cc` | Passed |
| `uwb_qa_uwb_radar_packet_source.cc` | Passed |
| `uwb_qa_uwb_loopback_echo.cc` | Passed |

既有 MATLAB integer/fractional golden、32/64/128 完整 packet、scheduled dump e2e 与非 Radar 测试继续通过。

---

## 9. 未做

- Step 7 `UwbRadarCirEstimator` message block
- CirWriter、EchoTimer、UHD、Phase A、X410
- 未提交、未 push
- 未改 `开发状态.md`
- `sfd_search_margin=64` 仍不是硬件冻结值
