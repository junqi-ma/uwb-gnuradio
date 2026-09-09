# 测试报告：X410 CG400 实时 HRP 定时发送/接收与 CIR 连续写入

日期：2026-09-09
设备：NI X410 `ni-x4xx-2607179`，FPGA `CG_400`，`addr=192.168.20.2`
脚本：`gr-uwb/apps/x410_cg400_hrp_echo_cir.py`

## 1. 射频与速率

| 项 | 值 |
|---|---|
| 采样率 | 491.52 MS/s（CG400 原生，不是 UC200 737.28） |
| 工作率 | 998.4 MS/s（PDU 65/32，L=65 M=32） |
| TX | 前方面板 ch1 = UHD ch0 / `TX/RX0` |
| RX | 前方面板 ch4 = UHD ch3 / `RX1` |
| 载波 | 6489.6 MHz |
| 增益 | TX 40 dB，RX 50 dB |
| 波形 | C++ `UwbHrpPacketSource`，code 9，64 SYNC，4z2，STS on |

不要对已知 `t0` 的雷达路径并联能量门 Detector + ScheduledExtractor。
CIR 必须留在 998.4 MS/s。

## 2. 本机约束

- Host UHD 4.6 + 系统 DPDK AVX-512 在 i7-12700 上 `rte_srand` SIGILL。
  本轮 `ENABLE_UHD_BACKEND=OFF` 重建 `gnuradio-uwb`，Python 绑定从
  `gr-uwb/build/python/uwb/bindings/uwb_python*.so` 直接加载；UHD 只用
  PyUHD，`LD_LIBRARY_PATH` 指向去 AVX-512 ctor 的本地 `rte` 副本。
- 设备在 QSFP `sfp1` / `192.168.20.2`，不是默认 `192.168.10.2`。
- 官方 `x410_uwb_radar_validate.py` 内置 adapter 仍是旧 PyUHD 方言，本机未改。
- C++ `UhdBurstBackend` / `EchoTimer` 没有 Python 绑定，本轮未接入 GNU Radio。

## 3. 100 Hz × 10 s

```bash
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir.py \
  --args addr=192.168.20.2 \
  --rate-hz 100 --duration-s 10 \
  --gain-tx 40 --gain-rx 50 \
  --cal-delay-native 334 \
  --sfd-search-margin 128 --sfd-threshold 0.12 \
  --sync-refine-margin 32 --sync-refine-threshold 0.02 \
  --tail-guard-us 24 \
  --output <dir>
```

| 计数 | 结果 |
|---|---|
| 定时突发 | 1000/1000 ok，late=0 |
| TX 间隔 | min=max=mean=0.010 s，跨度 9.99 s |
| PDU 65/32 | 1000 in / 1000 out，drop=0 |
| CIR 估计 | 1000 完成，fail=0，drop=0 |
| CIR 写入 | 1000 帧，`pulse_id` 0…999 连续 |
| `cir.cf32` | 928000 字节 = 1000 × 116 tap × 8 |
| 峰值 | `peak_tap=22`，`cir_origin=2717`，`sfd=67746` vs pred `67741` |
| metric | 约 0.0172（弱耦合/泄漏，不是环回金样 SNR） |

同一射频路径、`sfd-search-margin=8192` 时估计器约 181 ms/帧，队列深 64，
只写出 100/1000，其余 `queue_full`。校准已锁 ±2 样本后必须把 margin 收到约 128。

## 4. 未声称

- 不是 200 pps soak。
- 不是 C++ `UhdBurstBackend` 生产验收。
- 不是 X410 盲捕获 / overflow 重捕获验收。
- 未与 MATLAB CIR 金样逐 tap 对照。
