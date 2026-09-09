# 验收：UWB HRP 波形生成 P3

> 日期：2026-09-09
> 结果：**通过**
> 主机：12th Gen Intel Core i7-12700（20 线程），`CMAKE_BUILD_TYPE=Release`

## Compact 交接

- 组包仍是 `modulate_one` + `UwbHrpPacketSource`。读文件块不要动。
- P1 门未放宽：ieee 127 B 对 `window.cfile[9984:9984+249280]` L2&lt;1e-6，`demodulate_one` FCS。
- 热路径：`HrpPrefixCache` 预计算 SYNC/SFD/STS 脉冲成型；PHR/payload 按非零 chip 稀疏加 48 点核；`volk_32f_s32f_multiply_32f` 缩放，`volk_32f_x2_interleave_32fc` 打成 FC32。
- 变 PSDU 复用同一 prefix（code / SYNC / SFD / STS 不变）。
- 发送上限仍是 `kMaxHrpTxSamples=1<<22`。不要改雷达 RX `kMaxRadarBurstSamples`。
- `apps/benchmark_hrp_mod [warmup] [rounds]`，ctest `uwb_benchmark_hrp_mod`。
- 64-SYNC PSDU≤127 B **host-only** ≥1000 pkt/s。2048-SYNC 不承诺。不宣称 X410。
- 不改 `testdata/uwb_radar/tx_998p4.cf32`。

## 门

| 门 | 结果 |
|---|---|
| P1 回归（ieee 127 B 逐样本 + FCS） | pass |
| prefix 缓存后再次生成 P1 仍 L2&lt;1e-6 | pass |
| P2 回归（4z2 / STS / SYNC 长度 / code10 / 块） | pass（`uwb_qa_uwb_hrp_mod_core`） |
| `uwb_qa_uwb_hrp_packet_source` | pass |
| demod_core / radar_packet_source 回归 | pass |
| 64-SYNC 127 B 相同 PSDU mean≤1000 µs | **542.9 µs，1842 pkt/s** |
| 64-SYNC 127 B 每包不同 PSDU mean≤1000 µs | **528.4 µs，1893 pkt/s** |

`benchmark_hrp_mod 8 128`（Release）：

| 剖面 | mean | p50 | p95 | pkt/s | 门 |
|---|---|---|---|---|---|
| 64-SYNC 127 B cached | 542.9 µs | 521.1 | 668.1 | 1842 | 是 |
| 64-SYNC 127 B unique PSDU | 528.4 µs | 516.2 | 583.3 | 1893 | 是 |
| 64-SYNC 0 B | 149.3 µs | 144.9 | 180.8 | 6697 | 否 |
| 2048-SYNC 0 B | 4655.6 µs | 4504.5 | 5549.9 | 215 | 否（不承诺） |
| 64-SYNC 4z2+STS 22 B | 327.7 µs | 322.2 | 355.0 | 3051 | 否 |

## 复现

```bash
cmake --build gr-uwb/build --target benchmark_hrp_mod uwb_qa_uwb_hrp_mod_core.cc -j
cd gr-uwb/build
ctest -R 'uwb_qa_uwb_hrp_|uwb_benchmark_hrp_mod|uwb_qa_uwb_demod_core|uwb_qa_uwb_radar_packet_source'
./apps/benchmark_hrp_mod 8 128
```
