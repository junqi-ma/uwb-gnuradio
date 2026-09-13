# C++ PDU EchoTimer A/B 实测（M1/M2/M3）

设备与参数：X410 CG400，`addr=192.168.10.2`，原生 491.52 MS/s，TX/RX0 ch0、
RX1 ch3，`--preamble-length 128 --pulse-shape minphase --gain-tx 50
--gain-rx 60 --cal-delay-native 334 --no-udp`。同一时刻单进程占用 X410。

后端：
- `python` = `TimedUhdEcho`（每脉冲 Python PMT publisher）。
- `cpp-pdu` = `UwbRealtimeEchoTimer` + `UhdBurstBackend`（message-only 网格，
  无 Python 定时路径）。

## 结果

| case | backend | res-workers | pulses | wall_s | echo_ok | late | res_drop | est_drop | wr_ok | worker_us_mean |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 Hz | python (M0) | – | 1000 | 10.242 | 1000 | 0 | 0 | 0 | 1000 | – |
| 100 Hz | cpp-pdu | 1 | 1000 | **10.260** | 1000 | 0 | 0 | 0 | 1000 | 10105 |
| 200 Hz | python (M0) | – | 1000 | 5.248 | 999 | **1** | 0 | 0 | 999 | – |
| 200 Hz | cpp-pdu | 1 | 1000 | **5.266** | 1000 | **0** | 0 | 0 | 1000 | 5126 |
| 200 Hz | cpp-pdu | 4 | 1000 | 5.246 | 1000 | 0 | 0 | 0 | 1000 | 5129 |
| 500 Hz* | cpp-pdu | 1 | 2500 | 5.267 | 2500 | 0 | 0 | 0 | 2500 | 1981 |
| 500 Hz* | cpp-pdu | 8 | 2500 | 5.349 | 2499 | 50 | 1 | 0 | 2499 | 1994 |

\* 500 Hz 只作瓶颈定位，**不是稳定验收**（见下）。

## 结论

- **200 Hz 达标（M1 验收）**：`cpp-pdu` 1000/1000、0 late、0 res/est drop、
  CIR 1000 条，`schedule_wall_s≈5.27 s`（实时）。旧 Python 链 200 Hz 有 1 late。
- **100 Hz 两链都实时**，`cpp-pdu` 与 Python 基线 wall 相差 <0.02 s。
- **去掉了 Python 定时热路径**：`cpp-pdu` 没有 `tolist/init_c32vector`
  （M0 基线每脉冲 ~1.08 ms + ~0.80 ms，持 GIL）。radio worker 的
  `worker_us_mean` ≈ PRI（100 Hz 10.1 ms，200 Hz 5.13 ms），即 worker 与设备
  时钟同步。
- **M3 worker 数**：200 Hz 下 res-workers=1 已 0 late / 0 est_drop
  （每窗 FIR 在下游消息线程，不反压 radio worker），4 无额外收益。
- **500 Hz 边界**：res-workers=1 本次 2500/2500 通过，但 `worker_us_mean
  =1981 us ≈ PRI 2000 us`，已贴每 burst UHD 上限（spec §2.5 要求逐 burst
  UHD 路径 <2 ms 才算达标）；同日 res-workers=8 反而出现 50 late / 1 fail
  （多 worker 抢占 UHD 核，spec §10 的停止条件之一）。**不写成 500 Hz 验收通过。**

## 数值说明（重要）

- CIR 坐标与几何（`cir_origin_sample=2717`、`calibration_delay_work_samples
  =678.4375`、`predicted_sfd_start_sample=132765`）与 Python 链**逐字段一致**。
- `cpp-pdu` 发布 native SC16，resampler 内部转 FC32（spec §M1.2 要求保留 SC16），
  因此 `cir_peak_metric`/`raw_l2_norm` 比 Python 链大 ~32768×（int16 满量程）。
  CIR taps 是归一化的（`emit_normalized=True` → `cir_norm.cf32`），
  **SFD/同步相关判决是功率归一化**（`m = norm(acc)/(pwr+eps)`，
  `uwb_radar_sfd_core.h:170`），所以阈值与 CIR 形状不受该量纲影响。
- **跨进程 CIR 不能逐帧直接比对**：同参数两次 Python 硬件抓取的
  `cir_norm` 余弦只有 ~0.27（第一峰随抓取时刻亚样本漂移）；数值一致性应离线
  用同一输入 PDU 对照（已由 `qa_uwb_pdu_rational_resampler_65_32_workers.cc`
  与 PDU resampler QA 覆盖）。

## 文件

每个 case 一个目录：`summary.json`（app 汇总）、`stdout.log`。
`../m0_baseline/` 是 Python 基线（SA4）。
