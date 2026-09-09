# 验收：UWB HRP 波形生成 P1

> 日期：2026-09-09
> 结果：**通过**
> 下一阶段：P2 参数化（SYNC / SFD / PSDU / STS）

## Compact 交接（P2 只需要这些）

- 工作网格 998.4 MS/s，`SamplesPerPulse=2`，脉冲核 `kPulse48`（`testdata/uwb_hrp_tx/pulse_48.f32`）。
- 组包：`gr::uwb::mod::modulate_one`（`uwb_hrp_mod_core.h`）。
- 流图块：`UwbHrpPacketSource`，message PDU；`UwbRadarPacketSource` 仍是读文件，不要合并。
- P1 冻结包：code 9，64 SYNC，**ieee** SFD，**无 STS**，PSDU 127 B（已含 FCS `0x584b`），长度 **249280**。
- Golden：`testdata/realtime_demod_golden/window.cfile[9984:9984+249280]`。
- 卷积输入是 `[PHR19 | RS | 00]`，前 21 符号走 PHR 场。
- hop = spreading 首 chip 为 −1 则为 1。
- `insert_sts` 目前直接 `return false`。P2 打开。
- 解调闭环必须 `sfd_mode=ieee`。现有 demod **不解 STS**（PHR 假定紧接 SFD）。
- 不改 `testdata/uwb_radar/tx_998p4.cf32`。
- 不宣称 1000 pps / X410。

## 门

| 门 | 结果 |
|---|---|
| `uwb_qa_uwb_hrp_mod_core` | pass（RS 对拍、IQ L2&lt;1e-6、`demodulate_one` FCS） |
| `uwb_qa_uwb_hrp_packet_source` | pass（249280 PDU） |
| `uwb_qa_uwb_demod_core` | pass（回归） |
| `uwb_qa_uwb_radar_packet_source` | pass（读文件路径未改） |

## P2 要做

SYNC 32…2048；SFD ieee/decawave/4z*；code 9–12；PSDU 0 / 随机 N+FCS / hex / emit 覆盖；`insert_sts` 仅 4z（SP1：SFD 后、PHR 前，BPRF 33792 chip = gap+64×512+gap）。STS 用 MATLAB `allDRBG_STS.mat` 前 32 行。有 STS 的包只验字段 + 雷达 SFD，不要求 demod FCS。
