---
tags:
  - cfo
  - sfd
  - pulse-shaping
  - gnuradio
  - matlab
  - review
date: 2026-08-20
aliases:
  - CFO shaping delay vs SFD code grid
  - decoder 2-sample CFO offset
---

# Decoder CFO：SFD 码栅格 vs 成形波形群时延

给审查 **GNU Radio / C++ `UwbRealtimeDemodulator`（及任何 HRP UWB 解调）** 的代理用。MATLAB 已在 `+uwbdecoder/compensateCarrierOffset.m` 修过；本文说明问题、证据、修法，以及 C++ 里如何判定有没有同一缺陷。

相关代码（MATLAB，仓库 `F:\USRP数据解调`，分支 `gnuradio-scheduled-dump`）：

| 文件 | 角色 |
|---|---|
| `+uwbdecoder/detectRepeatedPreamble.m` | seeded 检测，模板 = **成形** `preamble_waveform` |
| `+uwbdecoder/refineTimingWithNsSfd.m` | SFD 精定，模板 = **未成形** `kron(sfd, sampled_code)` |
| `+uwbdecoder/estimateCir.m` | CIR，模板 = **未成形** `sampled_code` |
| `+uwbdecoder/compensateCarrierOffset.m` | CFO；`directCfoAnchors` 曾把成形波形放到 SFD 码栅格上 |
| `+uwbdecoder/buildUwbReference.m` | `lrwpanWaveformGenerator`，`SamplesPerPulse=2` |
| `tests/testCompensateCarrierOffset.m` | 合成时延 1/2/3、code 9/10/11、无 SFD 路径 |

C++ 对照入口（仓库外）：`uwb-gnuradio` 的 `UwbRealtimeDemodulator`，dump 解码脚本 `uwb-gnuradio/testdata/decode_scheduled_sc16_dump.py`。手册见 [[CODEX_scheduled_dump_MATLAB对照手册]]。

---

## 1. 结论（先看这个）

**不是 SFD 没对准。** SFD 把起点从成形波形对齐校正到扩频码栅格，这一步是对的。

缺陷是：**CFO 相关仍使用带脉冲成形群时延的 SYNC 波形，却放在 SFD 精定后的码栅格原点上。** 相关峰偏了约 **2 sample**（998.4 MHz 下约 2 ns），幅度掉约 **28 dB**，相位被旁瓣和 overlap 干扰拉弯，线性拟合把弯当成频偏。

Packet 47 DW1000 上，decoder CFO 从 **−2271 Hz** 变成 **−1739 Hz**（与 cancel 拟合 −1747 Hz 只差 8 Hz）。

C++ 若「SFD 用冲激/扩频码，CFO 用成形脉冲且钉在 SFD 起点、不做局部搜峰」，就有同一问题。若 CFO 用的是匹配滤波 **峰值**（argmax），或 SFD 与 CFO **同一套模板、同一原点**，则没有这个问题。

---

## 2. 现象（MATLAB SIC，packet 47）

Dump：`F:\UWB基带数据\qm35_gain1_scheduled_sc16_dump_20260817`，packet_id **47**，DW profile `dw1000_code10_n256`，full SIC。

生成 DW 信号 vs after-QM35 的相位呈整包斜坡（约 +25° → −24° / 256 SYNC），等效残余频偏 **~−520 Hz**。QM35 相位没有这条斜坡。

拆开后有两层，审查 C++ 解调时 **第一层才是 decoder 的问题**：

| 层 | 内容 | 是否 decoder 缺陷 |
|---|---|---|
| A | `directCfoAnchors` 在码栅格上相关成形波形，CFO 多估 ~520 Hz | **是** |
| B | SIC `estimate_uwb_cir_slow_phase` 把 decoder 补偿后 CIR 里剩下的线性项当 second-stage CFO，叠到已经拟合过的 replica 上 | SIC 二次使用，不是解调本身 |

本文只管 **A**。B 只在 C++ 也做 regenerate/SIC 时才需要对照。

---

## 3. 根因：两套模板、两套原点

HRP 工作率 998.4 MHz，`SamplesPerPulse=2`，一个 SYNC = 1016 sample。

### 3.1 三条链路用的模板不同

```
detectRepeatedPreamble   模板 = preamble_waveform     （MATLAB Butterworth 成形 SYNC）
refineTimingWithNsSfd    模板 = kron(SFD, sampled_code)（扩频冲激串，无成形）
estimateCir              模板 = sampled_code
compensateCarrierOffset  模板 = preamble_waveform     （direct 路径，SFD 之后）
        └── 旧代码：起点 = SFD 精定后的 start_sample，不再搜峰
```

`buildUwbReference`：

- `sampled_code`：第一个冲激在 **sample 1**
- `preamble_waveform`：同一 chip 的成形峰值在 **sample 3**
- 群时延 = **2 sample**（因果脉冲成形，不是某一包数据的偶然值）

code 9（QM35）、10 / 11（DW1000）同一套 `lrwpanWaveformGenerator`，时延相同。

### 3.2 Packet 47 上三套起点（after-QM35，SYNC 25）

DW SFD 精定 `start_sample = 275417`。前导检测（成形波形）的 overlap 起点是 **275415**（早 2 sample）。

| 模板 | 时间基准 | 相关峰相对该起点的 lag | lag=0 相对峰 |
|---|---|---|---|
| `sampled_code` | SFD `275417` | **+0** | 0 dB（就是峰） |
| SFD = `kron(DW-8, sampled_code)` | 预期 SFD = `275417+256×1016` | **+0** | 0 dB |
| CIR（同样 `sampled_code`） | `275417` | 峰值 **delay 0.000 ns** | — |
| `preamble_waveform` | SFD `275417` | **−2** | **−28 dB** |

`sampled_code` 抛物线亚采样约 **−0.48 sample**，CIR 在 −1 ns 与 0 ns 几乎一样强。那是模拟脉冲半个样点的宽度，**不是 2 sample 的帧定时错误**。

若 SFD 整体晚了 2 sample，`sampled_code` 和 SFD 模板峰也会在 lag −2，CIR 首径会在 −2 ns。这三项都在 0 → **SFD 对齐是对的。**

### 3.3 为什么 2 sample 会变成数百 Hz 的 CFO 误差

UWB ternary 前导脉冲很稀疏。成形波形相对码栅格再晚 2 sample，相关落到 **chip 间隙**：

- 对齐后 |corr| ~ 1.3e5；错位后 ~ 5.3e3（−28 dB）
- Packet 47 overlap 里 QM35 抵消残差相关 ~ 1.9e3，和错位 DW 相关只差约 9 dB；对齐后是 37 dB
- 错位相位在 overlap（SYNC 27–186）被拉弯；16 个 CFO 锚点里有 9 个落在 overlap
- 线性拟合把弯当成斜率 → decoder **−2271 Hz**
- 同一段 IQ、CIR 成形模板或把成形波形提前 2 sample：CFO **−1739 Hz**，与 cancel `fitReplicaCfo`（−1747 Hz）一致

这 **不是** 16 锚点太稀（全 SYNC 25–256 用错位模板仍是 −2350 Hz；CIR 匹配后 16 锚点也能给出 −1742 Hz），也 **不是** 前 10 个 SYNC 的 PLL，也 **不是** SFO（峰位置相对 1016 栅格全程卡在 −2 sample）。

Decoder 按 −2271 Hz 去旋之后，CIR 第 11–74 个 SYNC 上还剩 **+520 Hz**。那是过补偿的直接后果，不是第二个物理频偏。

### 3.4 哪条路径会踩坑

MATLAB `decode_uwb` 在 seeded + SFD 成功时：

1. `detectRepeatedPreamble(..., seed)` → `direct_sfd_timing=true`，起点按 **成形波形**
2. `refineTimingWithNsSfd` → 起点改到 **码栅格**（packet 47：275415 → 275417）
3. `compensateCarrierOffset` → `directCfoAnchors` 把成形波形放到新起点上（旧代码无平移）

非 direct 路径用 matched-filter **峰值**（`preamble.matched` 在 `peaks` 处取样），峰落在能量上，**没有这个问题**。

`estimate_uwb_preamble_cir`（preamble-only SIC）**不做 SFD 精定**，起点仍是成形波形对齐的，**不能**再提前群时延，否则会修过头。

---

## 4. MATLAB 修法（已落地）

`+uwbdecoder/compensateCarrierOffset.m`：

1. 仅当 `direct_sfd_timing` **并且** 已做 NS-SFD 精定（`sfd_waveform_correlation >= 0.10`）时，计算成形时延。
2. 时延 = `sampled_code` 第一个非零冲激，到 `preamble_waveform` 在该 chip 窗内的幅度峰值，单位 sample。这是 **reference 的属性**，不是 capture 的属性，禁止写死 `2`。
3. CFO 锚点起点和常相位对齐窗口都减去该时延，使成形脉冲峰值落回码栅格。
4. 无 SFD 精定：`shapingDelay = 0`。

验收：

- `tests/testCompensateCarrierOffset.m`：时延 1/2/3、无成形、code 9/10/11、无 SFD 标记
- Packet 47 DW：新 decoder CFO −1739 Hz vs cancel −1747 Hz（残差 8 Hz）；去掉 SFD 标记则仍为 −2271 Hz

---

## 5. 给 GNU Radio C++ 的审查清单

C++ 不需要和 MATLAB 函数同名。按下表 **搜概念和组合**，不要只搜 `CFO` 字符串。

### 5.1 先画两张表

**表 A — 模板**

| 阶段 | C++ 符号 / 文件 | 模板是什么 |
|---|---|---|
| 前导检测 / 相关峰 | ? | 成形脉冲？RRC？冲激扩频码？Ipatov 序列？ |
| SFD | ? | `kron(sfd, spreading_code)` 类冲激串，还是已成形波形？ |
| CIR / despread | ? | 与 SFD 相同？ |
| CFO（相位 vs 时间） | ? | 与 SFD 相同？与检测相同？ |

**表 B — 原点**

| 阶段 | 时间原点 | 是否在该原点做 ±N sample 搜峰 |
|---|---|---|
| 检测 | | |
| SFD 精定后的 `frame_start` / `sfd_start` | | |
| CFO 第 k 个 SYNC 窗口 | `start + k*period` 还是 `peak[k]` | |

### 5.2 判定规则

| 组合 | 结论 |
|---|---|
| SFD 与 CFO **同一模板、同一原点** | 无此 bug |
| CFO 相位取自匹配滤波 **峰值**（每个 SYNC 先 argmax 再取复数） | 无此 bug（峰在能量上） |
| SFD / CIR 用 **冲激扩频码**，CFO 用 **成形脉冲**，窗口钉在 SFD 起点、**不做搜峰** | **有此 bug** |
| CFO 用成形脉冲，但先用 `sampled_code` 或本地 argmax 对齐再取相位 | 已规避 |
| 前导检测用成形、SFD 把 start 改到码栅格、CFO 仍用检测那套成形模板 + 新 start | **有此 bug**（MATLAB 旧路径） |

典型群时延：MATLAB `SamplesPerPulse=2` 的 Butterworth 是 **2 sample @998.4 MHz**。C++ 若用更长 RRC/根升余弦，delay 可能是 4–16 sample，幅度损失和相位弯曲会更重，不要只搜 `±2`。

### 5.3 建议在 C++ 里 grep 的关键词

```text
CFO / cfo / carrier offset / frequency_offset / phase slope / unwrap
SFD / sfd / SHR
preamble / SYNC / Ipatov / ternary / spreading code / delta / pulse shape
Butterworth / RRC / rrc / root raised cosine / FIR pulse
matched filter / correlate / lag / group delay
start_sample / sfd_start / preamble_start + k * period
```

重点读「对已知起点做 `rx[start + k*T + 0 : L]` 与模板内积、再 unwrap 相位、再直线拟合」的函数。若模板是成形的而 `start` 来自 SFD 码相关，就是候选。

### 5.4 不修代码时的实测判定（推荐）

对 **已经 FCS 通过** 的一包（mixed dump 上 DW 或 QM35 均可）：

1. 记下 C++ 的 `detected_start` / SFD 起点 `S`。
2. 在 998.4 MHz IQ 上，用 **CFO 实际使用的那份模板** `h`，算  
   `|⟨h, rx[S+Δ : S+Δ+|h|]⟩|`，`Δ = -8…+8`。
3. **判定：**
   - 峰值在 `Δ=0`，且 `|corr|(0)` 相对峰掉不到 3 dB → 原点与模板一致，**无此 bug**。
   - 峰值稳定在 `Δ=±d`（d≥2），且 `Δ=0` 比峰低 **>10 dB** → **有此 bug**；`d` 就是该 C++ 脉冲的群时延。
4. 再对 **SFD 实际使用的模板** 做同样扫描。若 SFD 峰在 0、CFO 模板峰在 ±d，即可确认是模板不一致，不是 SFD 没锁上。
5. 对比：CFO 拟合值 vs 用「峰位置对齐后的同一模板」或 CIR 成形 replica 再拟合。差额若有几百 Hz，且 overlap 时更大，与 MATLAB packet 47 同类。

Packet 47 参考点（998.4，窗内 1-based）：

```text
DW SFD-refined start     275417
DW 前导检测 overlap start 275415    (= 275417 - 2)
QM35 start               301613
SYNC period              1016 sample
```

### 5.5 C++ 若确认有问题，修法（与 MATLAB 相同原则）

**不要写死 2。**

1. 从 **本机脉冲成形滤波器 / 合成 SHR 波形** 算群时延：第一个扩频冲激 → 该 chip 成形峰值。
2. 仅当 CFO 原点是 **SFD/码栅格** 且模板是 **成形波形** 时，把 CFO 窗口起点减去该时延。
3. 若 CFO 原点来自成形波形检测、从未做 SFD 码栅格精定，**不要减**。
4. 更稳的写法：CFO 与 SFD/CIR 用同一 `sampled_code`（或同一 Ipatov 冲激串）；或每个锚点先在 ±半个脉冲内 argmax 再取复数相位（所有锚点共用 **同一个** lag，避免逐 SYNC 跳径）。

### 5.6 不要误判成这些问题

| 看起来像 | 实际 |
|---|---|
| SFD 没对准 2 sample | SFD/CIR 在码栅格上是准的；偏的是 CFO 成形模板 |
| C++ `qm35_det_minus_pred` 的 +70…+300 sample | 那是检测相对预测的锁定残差（0.07–0.3 µs），比 2 sample（2 ns）大两个数量级 |
| 16 个稀疏锚点拟合不稳 | 错位模板即使用满 256 SYNC 仍偏 ~500 Hz |
| PLL / 接收机开机相位 | 只影响最前面几个 SYNC；MATLAB PLL 模板在 SYNC 9 只有约 2° |
| SFO | packet 47 上理想模板峰全程 lag=−2，斜率为 0 ppm |
| cancel 拟合 CFO 与 decoder 不一致 | 先检查 decoder 是否过补偿；cancel 对 IQ 重拟合往往更接近真值 |

### 5.7 若 C++ 还做 SIC / 再生抵消

即使 decoder CFO 修对了，也不要把「decoder 去旋之后 CIR 逐 SYNC 的线性相位」再加到 **已经对原始 IQ 拟合过 CFO** 的 replica 上。MATLAB 层 B 就是这样把 +520 Hz 又写回去的。C++ SIC 若有 `slow CIR phase` / `second stage CFO`，对照 `cancel_uwb_packet_in_iq.m` 里 `enable_cir_slow_phase` 与 `applyCirSecondStageCfo`。

---

## 6. 给审查回复用的最短结论模板

```text
CFO 模板: <成形 / 冲激码 / 与 SFD 相同>
SFD 模板: <...>
CFO 原点: <SFD start + k*period / MF peak>
每个锚点是否搜峰: <是/否>
|corr| 相对 SFD 起点的最佳 lag: <Δ=...>
lag=0 相对峰的 dB: <...>
判定: <有此 bug / 无此 bug / 已用搜峰规避>
群时延（若有）: <N sample @ 998.4 MHz>
```

无此 bug 时不必改 C++。有此 bug 时按 §5.5 修，并用 §5.4 在 mixed dump 的 DW overlap 包上复测（仅看 clean QM35 可能幅度损失不明显，相位弯曲会轻得多）。
