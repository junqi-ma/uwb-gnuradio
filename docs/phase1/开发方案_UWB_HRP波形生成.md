# 开发方案：UWB HRP 波形 C++ 生成

> 日期：2026-09-09
> 分支：`feature/uwb-monostatic-radar`
> 状态：**P1、P2、P3 已验收**。读文件 `UwbRadarPacketSource` 不替换。

## 1. 目标

C++ 在 998.4 MS/s 组 IEEE 802.15.4a HRP BPRF 完整包，现有 `demodulate_one` 能 FCS pass，并与 MATLAB `lrwpanWaveformGenerator` golden 逐样本对照。

P1 冻结：code 9，64 SYNC，IEEE SFD，无 STS，6.81 Mb/s，SamplesPerPulse=2，PSDU=127 B（含 FCS `0x584b`），包长 **249280**。

## 2. 块类型

`UwbHrpPacketSource`：`gr::block`，0 流端口，message PDU。无 `forecast`/`general_work`。`emit` 发布已生成 IQ；不在 host 上产生 PRI。热路径不扩容。

同类：`UwbRadarPacketSource`。上游 GNU Radio 无 HRP UWB 调制器。

## 3. 算法（与 MATLAB / 现有 demod 互逆）

1. PSDU 字节 → LSB-first 比特；已含 FCS 则不再追加。
2. RS(63,55) 系统编码，GF(2^6) poly 0x61，根 α¹..α⁸，部分块前导 0 填到 330 bit。
3. PHR 13 系统比特（速率 6.81=`10`b，长度 7 bit MSB，ranging=0，预留=0，preamble 64=`01`b）+ SECDED → 19 bit。
4. 卷积 `poly2trellis(3,[2 5])` 编码 `[PHR19 | RS | 00]`。前 21 符号走 PHR 场（0.85 Mb/s），其余走 payload 场（6.81 Mb/s）。
5. BPM-BPSK：hop = 该符号 spreading 首 chip 为 −1 则为 1，否则 0（scrambler 比特 `s[offset + n·cpb]`）。
6. SYNC/SFD：`kron(seq, sampled_code)`，508 chip/符号。
7. 芯片偶数样本放置，与 48 点脉冲核因果卷积（由 `reference_preamble.bin` LS 拟合），整包 `max(abs)` 缩放到 0.8。P3：SYNC/SFD/STS 脉冲成型进 `HrpPrefixCache`；PHR/payload 按非零 chip 稀疏放置同一 48 点核；VOLK `s32f_multiply` + `x2_interleave` 缩放并打成 FC32。

脉冲核：`testdata/uwb_hrp_tx/pulse_48.f32`。禁止用 751 点 native SYNC 铺包。

## 4. 验收

- 生成 IQ 与 `window.cfile[9984:9984+249280]` L2 相对误差 < 1e-6。
- `demodulate_one`（`sfd_mode=ieee`）FCS pass，字节等于输入 PSDU。
- 不改 `testdata/uwb_radar/tx_998p4.cf32`。
- P3：`apps/benchmark_hrp_mod` 上 64-SYNC、PSDU≤127 B 均值 ≤1000 µs/包（相同 PSDU 与每包不同 PSDU）。2048-SYNC 不承诺 1000 pps。不宣称 X410。
