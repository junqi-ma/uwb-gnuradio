# 开发需求：UWB 自发自收 Radar（Echo Timer + CIR 落盘）

> 分支：`feature/uwb-monostatic-radar`（从 `experiment/sync-polarity-validation` / `41aa698` 拉出）
> 日期：2026-09-04
> 状态：**需求稿，待审批**（实现前基线；未开始写代码）
> 主线入口：实现合并前同步到 [`../../开发状态.md`](../../开发状态.md)
> 对照实现：`/home/junqima/workspace/gr-radar` 的 `usrp_echotimer_cc`
> 算法对照：[`../../UWB_demodulation/+uwbdecoder/estimateCir.m`](../../UWB_demodulation/+uwbdecoder/estimateCir.m)
> 审批说明：本文是实现前需求，不改 [`../../开发状态.md`](../../开发状态.md)。审批意见请直接改本文或另开评审记录。

---

## 1. 目标

在 GNU Radio OOT `gr-uwb` 中实现 **UWB 单基地自发自收雷达最小闭环**：

```text
周期产生一个已知的正常 UWB packet（SYNC 长度可配）
  → 按 gr-radar echo_timer 语义同时发射并接收
  → 在已知 TX 时刻附近搜索 SFD，由 SFD 回推 SYNC 起点
  → 用 SFD 前的 SYNC 做 UWB CIR 估计（复数，MATLAB estimateCir 对齐）
  → 将 CIR（及元数据）落盘
```

本阶段交付的是 **雷达观测**（每脉冲一条复数 CIR），不是完整 UWB 解调，也不是 QM35 旁路捕获。旁路捕获（`UwbAutoScheduledExtractorSc16`）继续服务“听别人的雷达”；本需求服务“自己当雷达”。

验收口径（离线必须先绿，硬件后补）：

1. 软件 loopback：在已知 TX 种子附近找到 SFD，回推的 SYNC origin
   误差 ≤ 1 个工作采样点；已知时延 `τ` 的 CIR 主峰误差也 ≤ 1 点。
2. normalized CIR 与 MATLAB `estimateCir` 在同一合成包上逐 tap
   对照（复数 L2 相对误差 < 1e-5，或与现有 demod golden 同级）；
   raw CIR 另验证幅度线性，不用归一化结果代替。
3. 落盘可被 MATLAB 读回：`cir.cf32` + `cir.jsonl` 条数、offset、tap 数一致；开启时 `cir_norm.cf32` 也一致。
4. 本机 QA 不依赖 USRP；X410 实机是后续硬件验收，不得用 loopback 冒充。

---

## 2. 非目标

本需求 **明确不做**：

- 距离/速度跟踪、CFAR、RCS、多目标关联（gr-radar 的 `estimator_*` / `tracking_singletarget`）。
- 连续 host 65/48 实时流（已判死）。
- 把能量门 / Detector 接到自发自收主路径（TX 时刻已知）。
- 实时 SIC、SYNC 极性编码硬件开关。
- CFO 估计、CFO/常相位补偿及相位斜率有效性判定。
- RF 动态范围优化、ADC clipping/直泄漏抑制和 TX/RX gain 自动选择。
- 修改 `UwbRealtimeDemodulator` 默认行为去“顺便”当雷达。
- 宣称 X410 soak / overflow 通过（硬件阶段单独写测试报告）。

可选、默认关闭：接收窗 IQ 调试落盘。TX 可以是包含
SFD/PHR/PSDU/FCS 的正常 UWB packet，但 RX 算法在找到 SFD 后只回溯
SYNC 估计 CIR，**不解码 PHR/payload/FCS**。

---

## 3. 为什么仿 echo_timer，以及必须改什么

### 3.1 gr-radar `usrp_echotimer_cc` 在做什么

实现：`gr-radar/lib/usrp_echotimer_cc_impl.cc`。

| 点 | 行为 |
|---|---|
| 块类型 | `gr::tagged_stream_block`，`packet_len` tag |
| 每个 work() | 读一个 TX 脉冲包；`get_time_now()`；并行线程 TX `send` + RX `recv`；join 后输出 RX |
| TX | `tx_metadata.has_time_spec = true`，`time_spec = now + wait_tx`，SOB 后跟空 EOB |
| RX | `STREAM_MODE_NUM_SAMPS_AND_DONE`，`stream_now = false`，`time_spec = now + wait_rx` |
| 长度 | RX 样点数 = TX 包长 |
| 延迟 | `num_delay_samps`：输出左移该点数，尾部补零（电缆/链路上电延迟） |
| 设备 | 两台 USRP：TX 为 clock/time master，RX 默认 MIMO slave |
| 处理 | 下游 `estimator_sync_pulse_c` 对 TX/RX **取模后滑动点积** 找峰 |

这是 **突发相参收发**：PRI 内只传一个有限长脉冲，RX 用 timed burst 取同样本数，而不是 1 GS/s 连续收。

### 3.2 必须保留的语义

1. 同一时间基上 **预先调度** TX 与 RX（`has_time_spec` / `stream_now=false`）。
2. RX 用 `NUM_SAMPS_AND_DONE`，脉冲之间射频可以静默。
3. 用整数 `num_delay_samps` 补偿已知 TX/RX 链延迟（泄漏峰校准）。
4. 每个雷达脉冲对应一次 TX 波形 + 一次 RX 窗，一对一。

### 3.3 不能照搬的部分

| gr-radar | 本项目必须改 |
|---|---|
| RX 长度 = TX 长度 | 默认 RX 窗 = 预保护 + SYNC + SFD + 距离/滤波保护；不必接收不解码的 payload |
| 两台 USRP + MIMO | 默认 **一台 X410**，TX/RX 分通道、共享内部 clock/time |
| host CF32 连续/等长包 | 原生 **SC16 @737.28 MS/s** 突发；CIR 工作率 **998.4 MS/s** |
| `abs(x)` 后滑动相关 | 必须用 IEEE 802.15.4a/z **SYNC 相干 CIR**（`estimateCir`） |
| tagged stream 穿过全速 PRI 填零 | PRI=5 ms @737.28 约 3.69e6 样点/脉冲；host 不能把静默段当流。TX 只含有效脉冲，RX 只含截窗 |
| `gr-radar` 链 UHD | 现有 `libgnuradio-uwb` **不链 UHD**。Echo timer 对 UHD 可选编译；loopback 无 UHD 必须能测 |

GNU Radio 树中同类块：`uhd.usrp_sink` / `uhd.usrp_source`（SOB/EOB + `tx_time`/`rx_time` tag）、`blocks.message_strobe`、`digital.corr_est_cc`。没有 UWB CIR 突发雷达块。echo_timer 是收发时序参考；CIR 参考本仓库 MATLAB/C++。

---

## 4. 与现有代码的关系

| 现有能力 | 本需求用法 |
|---|---|
| `UwbAutoScheduledExtractorSc16` / scheduled dump | **不**接主路径。那是听外部 QM35。自发自收的 t0 就是自己的 TX time_spec |
| `UwbRealtimeDemodulator` + SFD/CIR stages | **复用算法，不复用整个解调块**。抽出 radar SFD + CIR core；保留 **复数 CIR**；不跑 PHR/payload |
| `CirResult.cir_values` 现为 `vector<float>` 实部 | 解调诊断维持现状。雷达 CIR 用独立 `vector<complex<float>>`，禁止再写成实部 |
| `UwbPduRationalResamplerCcf65_48` | RX 突发 SC16→CF32 后升到 998.4，再做 SFD/CIR；需补齐 radar metadata 传递并重测尾延迟 |
| `UwbPacketWriter`（IQ `capture.iq`+`jsonl`） | 契约风格对齐；**新写 `UwbCirWriter`**，主产物是 CIR 不是 IQ |
| `testdata/reference_preamble_code9_737p28.cf32`（751 点） | 仅用于 native 检测/诊断模板，禁止直接重复 64 次当 TX |
| MATLAB `estimateCir` / `generate_uwb_tx_from_decode` | CIR golden 与完整正常 UWB packet TX 波形 |
| `x410_auto_scheduled_capture.py` | 频率 6489.6 MHz、间隔 5 ms、SC16 入口脚本风格可抄；本需求另做 `x410_uwb_echo_radar.py` |
| SIC worktree TX reconstructor | 不在本分支合并 SIC。完整 packet 波形优先 MATLAB/已验证 testdata |

CIR 窗提醒：解调默认 `cir_pre=8`、`cir_post=30`（38 tap @998.4 约覆盖 5.7 m 往返距离栅格）。雷达使用独立 `radar_max_range_m`和 `2R/c` 换算，不复用解调 `cir_max_path_m`。

---

## 5. 系统架构

### 5.1 生产主路径（X410 突发，host-only）

```text
UwbRadarPacketSource          # 启动时加载/生成完整 TX packet，不掌握实时 PRI 节拍
    │  PDU: c32vector + meta(sync_reps, sfd_mode, tx_len, fs_tx)
    ▼
UwbEchoTimer                  # 唯一调度者：timed TX + NUM_SAMPS_AND_DONE RX
    │  原生 SC16 @737.28；RX 窗 = pre + SYNC/SFD + range + filter guard
    │  PDU: s16vector + meta(pulse_id, tx_time, rx_time, delay_samps, overflow)
    ▼
UwbPduWindowCrop?             # 可选：去掉 TX 前保护中不参与 CIR 的段
    ▼
SC16 → CF32（窗内）
    ▼
UwbPduRationalResamplerCcf65_48     # 737.28 → 998.4，仅突发
    ▼
UwbRadarCirEstimator          # 已知 TX 窄窗搜 SFD → 回推 SYNC → estimateCir；无 CFO stage
    │  PDU: c32vector(CIR taps) + meta
    ▼
UwbCirWriter                  # cir.cf32 + cir.jsonl
```

默认 **不** 走 RFNoC 65/48 连续流，也 **不** 连续 `usrp_source`。

### 5.2 离线 / QA 路径（无 USRP，必须先通）

```text
UwbRadarPacketSource
    ▼
UwbLoopbackEcho               # 已知时延、衰减、AWGN、可选多径；无 UHD
    ▼
（可选 PDU 65/48，若 TX 已是 998.4 则跳过）
    ▼
UwbRadarCirEstimator
    ▼
UwbCirWriter 或 message_debug
```

`UwbLoopbackEcho` 对应 gr-radar 的 `static_target_simulator_cc`：把 echo_timer 的时序契约变成纯软件，使 QA 不碰射频。

### 5.3 数据率（说明为何必须突发）

PRI = 5 ms → 200 pulse/s。

| 方案 | host 负担 | 结论 |
|---|---|---|
| 连续 SC16 @737.28 | 2.95 GB/s | 现有旁路捕获才用；自发自收不需要 |
| echo_timer 把整个 PRI 当 tagged packet | 每脉冲 3.69e6 样点 | 禁止 |
| 突发：64-SYNC + 4z2 SFD，15 m RX 分析窗约 75.4 µs SC16 | RX 约 44.5 MB/s；TX 另按完整 packet 长度计（不含传输开销） | **采用** |

---

## 6. 块类型与调度语义（实现前约束）

AGENTS.md：先定块类型，再写代码；`work`/`general_work` 内不分配。

### 6.1 `UwbRadarPacketSource`

- 类型：`gr::block`，**0 流端口**，输出消息口 `tx`。
- 启动时生成或加载一条完整的正常 UWB packet，发给 EchoTimer
  作为可重用波形；不用 host `message_strobe` 生成 5 ms 硬实时节拍。
- 不用 `sync_block` 输出填零的 PRI 流。
- TX 波形在 `make()` 预计算，热路径只引用，不每脉冲重建。
- PRI、`t0`、`schedule_index` 由 EchoTimer 依据 USRP device time 统一管理。

### 6.2 `UwbEchoTimer`

- 类型：`gr::block`，0 流端口；输入 `tx`，输出 `rx` / `status`。
- **不**在 message handler 里做 UHD I/O（会卡住 scheduler）。
- Handler 只更新波形/控制状态；**专用调度 worker** 根据 device time
  产生 PRI 栅格，调 UHD `send`/`recv`。
- 语义对齐 echo_timer 的 timed burst、SOB/EOB、`NUM_SAMPS_AND_DONE`和
  `num_delay_samps`；时间几何使用 `t_rx=t_tx-pre_guard`，不照搬独立 wait 参数。
- 不依赖“每 PRI 来一条 TX PDU”。若调度时刻已过，记录
  `late_drops`，跳到下一个未来栅格，不追赶发送。
- UHD：`find_package(UHD)` 可选。无 UHD 时该块不编进 lib，或编成 stub；QA 走 loopback。
- 单 X410：优先 `multi_usrp` 同一 device 上 TX streamer + RX streamer（两通道）；也允许 `args_tx`/`args_rx` 分设备（回退 echo_timer 双机）。
- UHD 热路径禁止假设一次 `send()`/`recv()` 完成整个 burst；必须循环处理
  partial send/recv，在首个 TX fragment 上带 time spec/SOB，在最后一个
  fragment 正确结束 EOB，并处理 TX async error、RX timeout/overflow/late/
  broken-chain。RX timed command 必须在 TX 前留足提交余量。

不选 tagged_stream 的原因：本仓库解调/写盘是 PDU；PRI 静默不应进 GNU Radio 流；UHD 突发 I/O 必须离开 scheduler 线程。

### 6.3 `UwbLoopbackEcho`

- 类型：`gr::block`，0 流端口；`tx` → `rx`。
- Handler 内对预分配 scratch 做延迟/衰减/噪声（最大 TX+range 在 `make()` 分配）。
- 元数据与 EchoTimer 相同（假 `tx_time`/`rx_time`），CIR 块无法区分来源。

### 6.4 `UwbRadarCirEstimator`

- 类型：`gr::block`，0 流端口；与 `UwbRealtimeDemodulator` 相同：有界队列 + worker + per-worker scratch。
- 热路径调用 header-only `radar_cir_one(...)`，不 `new`。
- 输入：998.4 CF32 PDU（升采样后）。若输入仍是 737.28，拒绝并 `invalid_inputs++`（第一期不在本块内重采样）。
- 现有 PDU 65/48 adapter 必须传递 `pulse_id`、device time、SYNC/SFD
  配置、校准和 RX 窗几何；QA 逐字段检查，不可依赖默认 PMT
  metadata 透传。

### 6.5 `UwbCirWriter`

- 类型：`gr::block`，0 流端口；与 `UwbPacketWriter` 相同：handler 入队，worker 写盘。
- 不在 handler 里写文件。

---

## 7. TX 波形

### 7.1 默认（第一期必须）

IEEE 802.15.4z BPRF 正常 packet：

| 参数 | 默认 | 说明 |
|---|---|---|
| code | 9 | `HRPCodes(9)`，已在 `uwb_phy_profile.h` |
| SYNC 重复 | 64 | **可配**；TX 生成器、SFD 回推和 CIR 可用 repetition 必须使用同一值 |
| SFD | 必须有，默认 4z2 | RX 找到 SFD 即回溯 SYNC 估计 CIR |
| PHR/PSDU/FCS | 固定合法内容 | 使 TX 保持正常 UWB packet；RX 不解码 |
| 工作率生成 | 998.4 MS/s 下生成完整 packet | 64-SYNC 段为 64×1016 = 65024 样点（~65.1 µs） |
| 上射频 | 对**完整 packet**一次性 48/65 到 737.28 | 启动前预生成 native SC16/CF32 golden，热路径不重采样 |
| PRI | 5 ms | `defaults::kQm35PacketIntervalS` |
| 幅度 | 可配峰值 | 文件波形先做峰值归一化；X410 实发必须显式给出 TX amplitude/gain，不提供高功率隐式默认 |

TX 是包含 SYNC + SFD + PHR + PSDU/FCS 的正常 pulse-shaped packet，
不是稀疏 `sampled_code`。`sampled_code` 只给 CIR 相关器。

第一期允许：

1. 启动时从文件加载一条由 MATLAB `lrwpanWaveformGenerator`
   或已验证工具生成的完整 737.28 packet golden。
2. 或在 998.4 生成完整 packet 后，启动时一次性 48/65 到 native。

禁止将 751 点 native 单 SYNC 模板直接拼接重复：真实 native
SYNC 周期是 `1016*48/65 = 750.276923...` 样点，必须对完整 packet
连续重采样以保留分数相位。

不在第一期用 C++ 重写完整 PHR/PSDU 成形器。

### 7.2 明确禁止

- 把 5 ms PRI 填零后当一条 TX 突发发出去。
- 第一期每脉冲随机 payload（虽然不影响 SFD 前的 CIR，但会降低 TX golden
  与硬件复现性）。
- 默认 350 µs PRI（`run_decode_uwb_radar.m` 旧配置）。默认 5 ms；350 µs 仅作可配项。

---

## 8. Echo timer / UHD 契约

### 8.1 时序

对脉冲 `k`：

```text
t_tx[k] = t0 + k * PRI
t_rx[k] = t_tx[k] - pre_guard_s
```

- `arm_delay` 默认 0.05 s，仅用于首脉冲前端稳定和预排程。
- `t0 = get_time_now() + arm_delay`；之后 TX 严格按 device-time PRI 栅格排程。
- RX 必须早于 TX 一个 `pre_guard_s` 开始；禁止用两个独立
  `wait_tx/wait_rx` 接口表达这一几何关系。
- 若 `t_tx` 已过（调度落后）：丢该脉冲，`late_drops++`，从下一未来栅格恢复，禁止追赶发送。

### 8.2 RX 窗几何（原生 737.28）

```text
rx_len = pre_guard + sync_samples + sfd_samples + range_guard +
         resampler_tail_guard
```

默认（737.28 MS/s）：

| 段 | 时间 | 样点 |
|---|---|---|
| pre_guard | 2 µs | `llround(2e-6 * 737.28e6) = 1475` |
| SYNC + SFD | 由 `sync_repetitions` 和 `sfd_mode` 决定 | 64-SYNC 约 65.1 µs；4z2 SFD 再加约 8.1 µs |
| range_guard | 由 `radar_max_range_m` 决定，默认 15 m 往返 | `ceil(2*15 / c * 737.28e6) = 74` |
| resampler_tail_guard | 由 65/48 FIR 契约决定 | 必须覆盖 PDU 尾部群时延，不得裁掉 CIR 后 tap |

15 m 往返 ≈ 100 ns ≈ 73.8 native 样点；在 998.4 MS/s 工作域为
约 100 tap。默认 RX 只需覆盖 pre-guard + SYNC + SFD + CIR 后置窗；
PHR/payload 既不解码也不必默认回传 host。调试模式可显式收完整 packet。
完整 TX packet 的空中时长必须小于 PRI 并留出调度余量。

`num_delay_samps`：RX 缓冲左移后再输出（与 echo_timer 相同），把 TX 泄漏对准 CIR 窗的参考零时延。该值由校准测，默认 0。

### 8.3 元数据（RX PDU 必带）

```text
pulse_id, schedule_index
tx_time_full, tx_time_frac
rx_time_full, rx_time_frac
num_delay_samps
calibration_delay_native_samples   # double，包含分数延时
calibration_id
sample_rate                 # 737280000
sample_format               # "sc16"
pre_guard_samples, sync_samples, sfd_samples, range_guard_samples
tx_packet_samples, rx_capture_samples
sync_repetitions, sfd_mode
uhd_error                   # none / overflow / timeout / late
```

overflow：该脉冲作废，`status` 上报，不送 CIR。不在本需求做 overflow 后自动重捕获（与旁路硬件验收分开）。

### 8.4 射频默认（X410）

与现有捕获脚本对齐，可改：

- device args：`addr=192.168.10.2`
- 频率：6489.6 MHz
- radio rate：737.28 MS/s，禁止 UHD coercion
- otw/cpu：SC16
- TX antenna / channel：可配；RX 默认另一通道（如 `RX2` 或 Radio 1）
- clock/time：`internal`（单机）；双机时 TX master、RX 外参考

---

## 9. CIR 估计

### 9.1 算法（必须与 MATLAB 一致）

移植 `estimateCir.m` + 已有 `stage_cir_softchips` 的 CIR 段：

1. **SFD timing**：用已知 TX time、`pre_guard`和校准链路延时预测
   SFD 位置，只在有界 `sfd_search_margin` 内做相关搜索。不做全 RX 窗
   的长 SFD 滑动相关，不使用能量门。
2. **SYNC 回推**：SFD 成功后，按 `sync_repetitions`、SFD 模式和每符号
   周期回推 preamble origin；在小 margin 内校正对齐。SFD 失败则本脉冲
   `status=sfd_failed`，不估计 CIR。
3. **无 CFO stage**：第一期不估计 CFO/相位斜率，也不对 RX/CIR
   施加 CFO 或常相位旋转。QA 的 MATLAB 对照使用无 CFO 输入；带 CFO
   的适用性留待后续需求。
4. **CIR**：跳过前 `cir_skip_initial_repetitions`（默认 10），对剩余
   SYNC 对齐相干平均，再与 **未成形** `sampled_code` 做 forward
   相关（MATLAB：`conv(average, flipud(conj(code)), 'valid') / code_energy`）。
5. 同时输出 **未归一化复数 CIR** 和用于 MATLAB 对照的 L2-normalized CIR。
   生产雷达观测以 raw CIR 为主，禁止只落单位范数 CIR。

第一期 **不** 做 soft-chip FIR、PHR/payload/FCS 解码，也不做任何
CFO 估计/补偿。
任一阶段失败都必须写一条无 CIR 的 JSONL：SFD 失败为
`status=sfd_failed`；SYNC 回推/对齐失败为 `status=timing_failed`；
CIR 计算失败为 `status=cir_failed`。

### 9.2 窗长

```text
pre_samples  = cir_pre_samples          # 默认 8
post_samples = radar_cir_post_samples
           或 ceil(2 * radar_max_range_m / c * fs)
```

**往返距离约定（必须写进元数据，避免和 MATLAB 解调窗混用）：**

- 解调 `cir_max_path_m` 保持旧语义，雷达块禁止复用该参数。
- 雷达 `radar_max_range_m` 是目标单程距离，tap 窗按往返延时 `2R/c`
  换算。
- 雷达落盘增加 `range_m_per_tap = c / (2 * fs)`（往返）。
- @998.4 MS/s：`range_m_per_tap ≈ 0.150136 m`。
- 默认雷达：`cir_pre_samples=16`（泄漏/早径），
  `radar_max_range_m=15`，对应约 100 个 post taps；QA 对照 MATLAB 时可显式
  切到 `8/30`。

CIR 在 **998.4 MS/s** 上估计，以便逐 tap 对 `estimateCir.m`。禁止第一期在 737.28 上另写一套 CIR 还声称 MATLAB 对齐。

### 9.3 与 demod CIR 的隔离

- 新文件：`include/gnuradio/uwb/uwb_radar_cir_core.h`（header-only）。
- 可调用 demod 的 SFD/timing 辅助函数；不调用 CFO 估计或补偿函数。
  结果结构独立：`RadarCirResult { vector<complex<float>> raw_taps;
  vector<complex<float>> normalized_taps; ... }`。
- 不修改 `CirResult.cir_values` 的实部诊断语义（避免破坏现有 QA golden）。

### 9.4 737.28 域能否直接估计 CIR

可以。737.28 MS/s 复数 IQ 已保留信号带宽，升到 998.4 MS/s
不会增加新的射频信息。但 native 域每 chip 为 `96/65` 样点、
每 SYNC 为 `750.276923...` 样点，不能原样复用 998.4 域的
`sampled_code` 和 `estimateCir`。

Phase-1 保留 PDU 65/48，原因是可直接对齐 MATLAB 且现有吞吐高于
200 pulse/s，而不是因为物理上必须插值。native 直接 CIR 作为后续
性能候选，必须实现 65-phase/已知 TX 波形估计、新 MATLAB golden
和统一 delay-axis 对照。详见
[评估_UWB_Radar_CIR重采样必要性.md](评估_UWB_Radar_CIR重采样必要性.md)。

---

## 10. CIR 落盘契约

目录由 `--output` 指定，默认文件名：

```text
<out>/cir.cf32          # raw complex CIR：除以 code_energy，不做 L2 归一化
<out>/cir_norm.cf32     # 可选：L2-normalized CIR，用于 MATLAB 算法对照
<out>/cir.jsonl         # 每脉冲一行
<out>/run.json          # 一次运行的静态配置（PHY、PRI、频率、窗）
```

可选 `--write-rx-iq` 才写 `capture.iq`+`capture.jsonl`（复用 PacketWriter 契约）。默认关。

### 10.1 `cir.jsonl` 每行

```json
{
  "pulse_id": 0,
  "schedule_index": 0,
  "status": "ok",
  "sample_rate": 998400000.0,
  "tap_count": 120,
  "pre_samples": 16,
  "post_samples": 104,
  "file_offset_taps": 0,
  "zero_delay_tap": 16,
  "peak_tap": 16,
  "cir_peak_metric": 0.91,
  "peak_delay_from_calibration_ns": 0.0,
  "range_m_per_tap": 0.150136,
  "raw_l2_norm": 0.013,
  "preamble_start_sample": 1475,
  "sfd_start_sample": 66500,
  "tx_time_full": 0,
  "tx_time_frac": 0.05,
  "num_delay_samps": 0,
  "sfd_ok": true,
  "timing_ok": true
}
```

`file_offset_taps` 是 `cir.cf32` 里的复样点偏移（每 tap 8 字节）。
目标距离按 `(tap-zero_delay_tap)*range_m_per_tap` 计算。`peak_tap`
在实机上可能是 TX→RX 直泄漏，不得无校准地称为目标首径。

SFD 未检测成功时仍写一行 JSONL：`status="sfd_failed"`、
`sfd_ok=false`、`tap_count=0`；不向 `cir.cf32`/`cir_norm.cf32` 写 tap，
`file_offset_taps` 保持当前末尾偏移。禁止回退到仅用 TX 种子的未验证 CIR。

### 10.2 MATLAB

新增 `UWB_demodulation/read_uwb_cir.m`：

```matlab
[cir, meta] = read_uwb_cir(outDir, pulseId);
% cir: tap_count x 1 complex64
```

QA 用该函数对照 loopback 主峰。

---

## 11. 参数与默认值

写入 `uwb_defaults.h` 新命名空间或明确 radar 段，避免和 scheduled dump 的 300/100 µs 干扰窗混淆。

| 名字 | 默认 | 备注 |
|---|---|---|
| `kRadarPriS` | 0.005 | 与 QM35 5 ms 相同量级，可配 |
| `kRadarCodeIndex` | 9 | |
| `kRadarPreambleReps` | 64 | |
| `kRadarNativeRateHz` | 737.28e6 | TX/RX 射频 |
| `kRadarWorkRateHz` | 998.4e6 | CIR |
| `kRadarCenterHz` | 6489.6e6 | |
| `kRadarPreGuardS` | 2e-6 | RX |
| `kRadarMaxRangeM` | 15 | 目标单程距离；内部按 `2R/c` 换算 |
| `kRadarCirSkip` | 10 | |
| `kRadarArmDelayS` | 0.05 | 仅启动 |
| `kRadarSfdMode` | `4z2` | TX 与 RX 搜索必须一致 |
| `kRadarSfdSearchMargin` | 待校准 | 已知 TX 时刻周围的有界搜索窗 |

---

## 12. 验证与 QA

实现完成的定义 = QA 绿，不是“块能编译”。

### 12.1 必做（无硬件）

1. `qa_uwb_radar_cir_core.cc`：合成可配 SYNC + SFD + 已知分数/整数
   时延；先断言 SFD 位置和回推的 SYNC origin，再断言主峰 tap 与 MATLAB
   `estimateCir` 差 ≤ 1；normalized CIR 复数相对 L2 < 1e-5。
   另注入错误/缺失 SFD，断言 `sfd_failed`、空 CIR，且不调用 CIR core。
2. `qa_uwb_loopback_echo.cc`：时延 N 样点（明确 native/work 坐标域）
   → writer 的 `(peak_tap-zero_delay_tap)` 对应 N，raw CIR 幅度与注入衰减一致。
3. `qa_uwb_cir_writer.cc`：10 帧中混合成功 CIR 与 `sfd_failed`；JSONL
   行数=10，只有成功帧写 tap，失败帧不推进 `file_offset_taps`。
4. `testdata/run_uwb_monostatic_loopback.py`：source → loopback → resample（若需要）→ CIR → 落盘；退出码非 0 表示峰位或条数失败。
5. MATLAB：`read_uwb_cir` + 与 `estimateCir` 比同一 `testdata` 包。

### 12.2 硬件（本需求不作为“完成”门槛）

脚本：`gr-uwb/apps/x410_uwb_echo_radar.py`

- `--dry-run`：打印 rate 契约、TX/RX 通道、窗长，退出 0。
- 实机：电缆/泄漏校准 `num_delay_samps`；静室或近距离金属板应出现稳定主峰。
- 单独测试报告，不写进“需求已完成”。

### 12.3 回归

现有 CTest 必须全绿。禁止为雷达 CIR 改 demod 实部 golden。

---

## 13. 分阶段交付

详细的逐步开发、QA 和退出条件见
[开发计划_UWB自发自收Radar.md](开发计划_UWB自发自收Radar.md)。

### Phase A — 算法与落盘（无 UHD）

- `uwb_radar_cir_core.h` + QA vs MATLAB
- `UwbRadarPacketSource`（完整 packet 文件/预生成波形）
- `UwbLoopbackEcho`
- `UwbRadarCirEstimator`
- `UwbCirWriter` + `read_uwb_cir.m`
- loopback e2e 脚本

**Phase A 通过 = CIR 算法与文件契约完成**，不代表 timed-burst
收发或雷达最小闭环完成。

### Phase B — Echo timer 块

- `UwbEchoTimer`（可选 UHD）
- GRC YAML
- 文档：与 gr-radar 时序对照表
- 无设备时 ctest 跳过或只测队列/超时桩

### Phase C — X410 应用

- `x410_uwb_echo_radar.py`
- dry-run + 人工实机
- 校准 `num_delay_samps` 记录进 `run.json`

### 明确不做的后续（另开需求）

- 更长距离窗 / 逐 SYNC CIR
- 动态 payload/实时重建 TX packet
- CFO 估计/补偿与带 CFO 情形的有效性门限
- RF 动态范围、clipping 和直泄漏抑制优化
- RFNoC TX
- 实时显示 CIR 瀑布图

---

## 14. Key Decisions

1. **突发 echo_timer，不要连续 RX。** 自发自收控制 t0，连续 737 MS/s 没有收益，只会 overflow。
2. **PDU 消息块，不要 tagged PRI 流。** 与现有 demod/writer 一致；UHD I/O 在 worker 线程。
3. **Phase-1 CIR 在 998.4，射频在 737.28。** 保留 PDU 65/48 是为了
   复用 MATLAB `estimateCir` 和已验证整数码片网格；native CIR 物理上可行，
   但需新算法/golden，不在 Phase-1 生产路径。
4. **独立雷达 CIR 结构（复数），不动 demod 的实部 `cir_values`。**
5. **第一期发送正常 UWB packet，SYNC 长度可配。** RX 只搜 SFD
   并回溯 SYNC 估计 CIR，不解码 PHR/payload/FCS。
6. **UHD 可选链接。** 无射频机器必须能编过、测过 Phase A。
7. **主产物是 CIR 文件，不是 IQ。** IQ 仅调试开关。
8. **单 X410 双通道为默认硬件；双 USRP MIMO 仅兼容。**
9. **loopback 峰位 QA 只是算法线；EchoTimer 和 X410 分别是 timed-I/O 与硬件线。**
10. **SFD 是 CIR 的硬门槛。** SFD 失败的帧只写 `sfd_failed`
    metadata，不估计 CIR，不做仅依赖 TX 种子的回退。
11. **第一期无 CFO stage。** 不估计、不补偿、不作 CFO 有效性验收。
12. **RF 动态范围不在本需求验收内。** TX/RX gain、clipping、
    直泄漏抑制和天线/衰减器优化后续单独处理。

---

## 15. Open Questions

实现按上一节默认进行。若要改默认，在开写 Phase A 前定：

1. **SYNC 长度变体**：第一期 CI 必须覆盖哪些 repetition？建议至少
   32/64/128，并验证 SFD 回推与 CIR skip/count 不越界。
2. **TX packet golden**：选定固定 PHR/PSDU/FCS 和完整 packet 生成器；
   生产 TX 必须是整包一次性 48/65 后的 native 波形，不再提供
   751 点单符号 repeat 选项。
3. **X410 TX/RX 通道**：同一 Radio 的 TX/RX0+RX2，还是 Radio0 TX + Radio1 RX？需 `uhd_usrp_probe` 后写进应用默认，需求层保持可配。
4. **PRI**：默认 5 ms 是否过慢（相干积累/多普勒）？第一期保 5 ms 与现有雷达周期一致；更快 PRI 只改参数，不改架构。
5. **SFD 搜索窗**：需由时延校准得到固定 TX/RX + 48/65 + 65/48
   群时延和抖动，再固化 `sfd_search_margin`；不允许直接放大到全窗暴力搜索。
6. **雷达零距离校准**：整数 `num_delay_samps` 不足以表达分数群时延。
   需定义 `zero_delay_tap`、double 校准延时、`calibration_id` 及泄漏峰与
   目标峰的区别。

---

## 16. PR Plan

每个 PR 可独立审查、带 QA；按序合并到 `feature/uwb-monostatic-radar`。
本节保留粗粒度里程碑；实际逐步开发和提交边界以
[开发计划_UWB自发自收Radar.md](开发计划_UWB自发自收Radar.md)
的 Step 0–12 / C1–C12 为准。

### PR1 — 需求文档入库

- 文件：本文、[`../README.md`](../README.md) 索引
- 依赖：无
- 需求原文入库，不写代码

### PR2 — Radar CIR core + MATLAB 对照

- 文件：`uwb_radar_cir_core.h`、`qa_uwb_radar_cir_core.cc`、testdata golden、可选 `read_uwb_cir.m`
- 依赖：PR1
- header-only；合成时延；不动 demod `CirResult`

### PR3 — Packet source + loopback echo

- 文件：`uwb_radar_packet_source.{h,cc}`、`uwb_loopback_echo.{h,cc}`、GRC YAML、QA
- 依赖：PR2
- 无 UHD；已知时延 PDU 契约

### PR4 — CIR estimator 块 + CirWriter

- 文件：`uwb_radar_cir_estimator.{h,cc}`、`uwb_cir_writer.{h,cc}`、`testdata/run_uwb_monostatic_loopback.py`、QA
- 依赖：PR3；复用 PDU 65/48
- Phase A e2e：source → loopback → resample → CIR → 落盘

### PR5 — EchoTimer（可选 UHD）

- 文件：`uwb_echo_timer.{h,cc}`、CMake `find_package(UHD)`、QA 桩（超时、late drop、delay shift）
- 依赖：PR3 的 PDU 契约
- 无设备 ctest 不打开真射频

### PR6 — X410 应用与 dry-run

- 文件：`gr-uwb/apps/x410_uwb_echo_radar.py`、examples GRC、简短 README
- 依赖：PR4+PR5
- `--dry-run` 进 CI；实机不进 CI

---

## 17. 实现时禁止事项

- 在 `work`/message handler 里 `malloc`/扩 `vector`。
- 连续 `usrp_source` + 能量门当自发自收。
- 把 `UwbAutoScheduledExtractorSc16` 并联到自己的 TX（t0 已知）。
- CIR 只存幅度或实部还称为雷达 CIR。
- 未跑 QA 就写“CIR 已对齐 MATLAB”。
- 把 loopback 截图当成 X410 验收。
