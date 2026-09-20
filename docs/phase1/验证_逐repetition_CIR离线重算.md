# 验证报告：逐 repetition CIR 离线重算（per-repetition individual values）

> 状态：**离线参考实现与验证完成（canonical golden 硬门通过 + 真实采集对照通过）**
> 日期：2026-09-20
> 脚本：`testdata/uwb_radar/verify_cir_individual.py`（numpy-only，golden 模式无需 scipy）
> 参考实现：`UWB_demodulation/+uwbdecoder/estimateCir.m`（`individual_values`）、
> `testdata/uwb_radar/export_uwb_radar_golden.m`（`local_estimate_cir`，generator-of-record）

## 1. 背景与结论

现状：realtime C++ `radar_cir_one` 把 `cir_repetitions` 个 SYNC repetition **相干平均**成
每个 pulse 一个 CIR（`x410_cg400_hrp_echo_cir*.py` 默认 skip=10、reps=0→`sync_reps-10`）。
MATLAB 参考本就支持逐 repetition 的 `cir.individual_values`。

本报告回答「能否每个 repetition 输出一个 CIR」的**离线可行性**：

- **能**。离线参考实现与 canonical golden 逐 tap 一致，逐 rep 平均与平均 CIR 在机器精度内相等。
- 逐 repetition CIR 的独立价值：**相干平均会在 `fs/(2*SPS)=491.34 kHz` 处产生零点**，
  逐 rep 不受该零点影响（见 §4）。这正是 jammer/CFO 研究可能被抹掉的部分。
- realtime 侧要不要做（算力 ×reps、数据 ×reps）见 §6，本报告不涉及 realtime 改动。

## 2. 方法

`verify_cir_individual.py` 是 `local_estimate_cir` + `estimateCir.m` 的 numpy 移植：

- `sampled_code`：HRP code-9 经 spreading=4、SamplesPerPulse=2 上采样到 1016（energy=64）。
- 平均路径：`avg = mean(窗口[skip..nSync-1])`，`raw = conv(avg, conj(code)[::-1], 'valid') / code_energy`。
- 逐 rep 路径：对每个 repetition 独立相关，得到 `individual`（`tap_count × valid`）。
- 逐 rep 相位斜率 → 残余 CFO：`f = slope * fs / (2π * SPS)`。

## 3. Golden 硬门（`python3 testdata/uwb_radar/verify_cir_individual.py`）

```
clean      raw_L2=2.183e-08 norm_L2=1.914e-08 mean_L2=9.18e-18 valid=54 peak=18  ok
delay_int  raw_L2=2.541e-08 norm_L2=2.969e-08 mean_L2=3.37e-18 valid=54 peak=55  ok
delay_frac raw_L2=2.283e-08 norm_L2=2.183e-08 mean_L2=7.08e-18 valid=54 peak=30  ok
PASS
```

- `norm_L2 < 1e-5`（与 `verify_uwb_radar_golden.m` 同阈值）：离线平均 CIR == MATLAB golden。
- `mean(individual, axis=1)` 与平均 CIR 相差 **~1e-17**（卷积与平均的线性性，机器精度）。
- peak tap 18/55/30 与 `metadata.json` 的 `peak_tap_measured_*` 一致。

## 4. 重复平均零点演示（`--null-demo`）

对 `rx_clean_998p4.cf32` 注入 CFO：

| cfo | avg\|raw[peak]\| | avg/avg0 | per-rep `<\|x\|>` | coh |
|---|---|---|---|---|
| 0 | 0.463187 | 1.000000 | 0.463187 | 1.0000 |
| f_null/4 | 0.0154838 | 0.033429 | 0.452507 | 0.0342 |
| f_null/2 | 0.0110371 | 0.023829 | 0.421438 | 0.0262 |
| **f_null** (491.34 kHz) | **2.008e-17** | 0.000000 | **0.310864** | 0.0000 |

平均在 `f_null` 完全抵消（2e-17），而逐 repetition CIR 仍有 ~0.31 的路径幅度。
`f_null = fs/(2*SPS) = 998.4e6/2032 = 491338.6 Hz`，与 `x410_cg400_hrp_echo_cir_jam.py`
注释一致。

## 5. 真实 jam 采集对照（`--capture logs/x410_jam_clip_check_20260919`）

数据：native 737.28 MS/s SC16（`capture.iq` + `capture.jsonl`），20 pulses，
`preamble_length=128`、skip=10 → 118 reps/pulse，`use_predicted_timing=true`。

- native→998.4 用 scipy `resample_poly(65,48)`，与仓库 65/48 PDU FIR 存在常数栅格偏移，
  脚本用 live sync template 粗对齐、再用 C++ `cir.cf32` 移位不变细化。
- **每个 pulse 的平均逐-rep CIR 与 C++ `cir.cf32` 的移位不变 `|xcorr|` = 0.9864–0.9867**
  （peak tap 完全一致，`reps=118`）。剩余 ~1.3% 来自 scipy FIR 与仓库 65/48 FIR 的差异。
- 逐 rep 相干增益 `|mean x| / mean|x|` = 0.9640–0.9644（相位相当稳定）。
- 逐 rep 单次相位噪声较大，per-rep CFO 线性拟合（+150…+240 Hz）**基本是噪声受限**，
  不足以作为 CFO 测量；需要跨 pulse 平均或多 tap 联合估计才能降噪。

## 6. 结论与下一步

1. **离线路径可用且已通过验证**：逐 repetition CIR 的算法与平均 CIR 自洽，golden 硬门通过，
   真实采集对照 |xcorr|≈0.986。
2. 若要把逐 rep 做到 **bit-exact**，需要 work-domain（998.4）PDU dump，而不是从 native 用
   scipy 反推；当前 native 重采样只到 ~0.986 形状一致。
3. 若要上 realtime（每个 repetition 一个 CIR PDU，`repetition_index` meta），代价是
   相关运算 ≈×reps（本例 118）、文件/UDP ≈×reps；单 worker 在 300 Hz 下会过载，
   需 SIMD（`uwb_cir_fir_simd.h`，radar core 尚未使用）、多 worker 或 stride 抽选。
   **未做**：realtime C++ 改动、硬件 soak。本报告不得写成硬件验收。

## 7. 复现命令

```bash
# golden 硬门
python3 testdata/uwb_radar/verify_cir_individual.py
# 零点演示
python3 testdata/uwb_radar/verify_cir_individual.py --null-demo
# 真实采集对照
python3 testdata/uwb_radar/verify_cir_individual.py --capture logs/x410_jam_clip_check_20260919
```
