# 验收：UWB HRP 波形生成 P2

> 日期：2026-09-09
> 结果：**通过**
> 下一阶段：P3 VOLK / 1000 pkt/s（64-SYNC、PSDU≤127 B）

## Compact 交接（P3 只需要这些）

- 组包仍是 `modulate_one` + `UwbHrpPacketSource`。读文件块不要动。
- 允许 SYNC `{32,64,128,256,512,1024,2048}`，SFD `ieee|decawave|4z1..4z4`，code 9–12。
- PSDU：构造期字节向量；`append_fcs` 给数据字节加 IEEE CRC；`emit` 的 u8vector 是精确 PSDU（不再加 FCS）。0 字节合法。
- `insert_sts` 仅 4z：SP1，SFD 后 / PHR 前。BPRF STS = gap512 + 64×512 + gap512 chip。DRBG 表 `kStsDrbgBprf`（MATLAB `allDRBG_STS.mat` 前 32 行）。
- ieee/decawave + STS → 失败。SYNC 非法 / code 非法 / PSDU>127 → 失败。
- TX 上限 `kMaxHrpTxSamples=1<<22`（2048-SYNC+STS+127B）。不要改雷达 RX `kMaxRadarBurstSamples`。
- 有 STS 的包：**不**走现有 demod FCS。验长度 + 雷达 `search_sfd`。4z2 无 STS、ieee 有 payload：demod FCS。
- 0 字节包以 PHR 结尾，不要要求 payload FCS。
- 热路径仍是标量 FIR 48 点。P3：剖析 `benchmark_hrp_mod`，预计算 SYNC/SFD/STS，VOLK FIR/缩放；变 PSDU 也要 ≥1000 pps。2048-SYNC 不承诺 1000 pps。
- 不改 `testdata/uwb_radar/tx_998p4.cf32`。不宣称 X410。

## 门

| 门 | 结果 |
|---|---|
| P1 回归（ieee 127 B 逐样本 + FCS） | pass |
| 4z2 无 STS 随机 8 B+FCS demod FCS | pass |
| 4z2+STS 22 B vs 雷达 `tx_998p4.cf32`（corr>0.999，SFD=65024） | pass |
| SYNC 32…2048、PSDU=0 长度 | pass |
| code 10 ieee FCS | pass |
| 块：STS 需 4z、append_fcs、emit 覆盖 PSDU | pass |
| demod_core / radar_packet_source 回归 | pass |
