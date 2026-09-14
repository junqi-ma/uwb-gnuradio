# 分析：X410 双 TX —— 离线数字注入的通信干扰 CFO 零点

> 状态：**P5 离线数字注入已完成（无硬件）**
> 分支：`feature/uwb-monostatic-radar`
> 日期：2026-09-14
> 脚本：`gr-uwb/apps/jam_offline_inject.py`
> 上级方案：[`开发方案_X410双TX_UWB通信干扰CFO研究.md`](开发方案_X410双TX_UWB通信干扰CFO研究.md)
> 数据：`/tmp/opencode/cg600_rx100/`（100 帧 jam-off X410 echo，code 9，
> `sync_repetitions=128`，`cal_delay_native=501`，737.28 MS/s）
> 输出：`/tmp/opencode/jam_offline/degradation.csv`

---

## 1. 结论

**是：参考 CIR 流水线在 491.34 kHz 处出现清晰的深度抑制零点。**

| 项 | 结果 |
|---|---|
| 自检（参考 CIR vs 出厂 `cir.cf32`，100 帧） | 复相关系数 **1.000000**（min 也是 1.000000），\|CIR\| 幅度比 **0.999969**，峰值 tap 一致率 **1.000**，帧 0 最大逐点差 **2.15e-8** |
| 零点位置（实测） | **491.339 kHz**（= `f_work/(2·SPS)` = **491.3386 kHz**） |
| 零点深度（相对 Δf=0） | EVM 功率 3771 → **3.17e-6**，抑制 **≈ 90.8 dB** |
| Δf=0 的干扰响应 | peak/floor = 3.75，EVM=3771（干扰远高于噪底） |
| 零点处的 peak/floor | **0.96**（干扰消失，退化为纯噪底） |
| 有效相干次数 `N_eff` | **114**（不是 118；见 §5.2） |
| Dirichlet 零点梳齿间距 | **8619.975 Hz** = `f_work/(N_eff·SPS)`；491338.58 = **57.0000 × 8619.975** |
| 零点宽度（半功率） | ≈ **4.0 kHz**；-10 dB ≈ 1.6 kHz；相邻零点 ±8620 Hz |
| 理论公式 | **成立**：491.34 kHz 是零点；但它是 `x = Δf·SPS/f_work = 1/2` 的"半 SYMBOL 周期"零点，**不是最低频零点**（见 §5.3） |

一句话：**干扰抵消失败的窄零点被离线实验精确复现**；零点深度 ~90 dB，零点宽度只有
几 kHz，因此任何 >±4 kHz 的未知 Δf 或符号时钟漂移都会离开零点——这支持"商用设备抵消
不理想的主因是 CFO/时钟，而非算法本身"的判断。

---

## 2. 方法（纯数字，未打开任何 UHD 设备）

### 2.1 输入

* `capture.iq`：737.28 MS/s 交织 int16（÷32768 → CF32 原生窗），100 帧，每帧 211192 样点。
* `capture.jsonl`：每帧 `file_offset_samples / sample_count / pre_guard_samples`。
* `metadata.json`：`code_index=9`、`sync_repetitions=128`、`cal_delay_native_samples=501`、
  `rate_native_hz=737.28e6`、`echo.publish_native=95440`。
* `cir.cf32`：出厂生产块对同批帧算出的 116 复数/帧 CIR，**仅用于自检**。

### 2.2 干扰波形

复用现有发射机制（`import x410_cg400_hrp_echo_cir as base`）：

```python
src = base.uwb.hrp_packet_source(psdu, 128, "4z2", jam_code_index=10,
                                 0.8, pri, False, insert_sts=True, False,
                                 "legacy", 2.5, 200.0, "")
work = np.asarray(src.samples(), dtype=np.complex64)
work = work[:128*1016]                 # --jam-mode preamble（默认，纯周期 SYNC）
jam_native = base.NativeRateProfile(737.28e6).tx_native(work)
```

* 干扰码 **code 10**，被干扰的 sensing 帧是 **code 9**。
* `--jam-mode packet` 保留 SFD/STS；默认 `preamble` 截到 `sync_reps*SPS`，得到严格周期
  信号，最适合看零点。
* 注入位置默认 `jam_delay_native = pre_guard + cal = 1475 + 501 = 1976`，使干扰
  preamble 落在 sensing 的 CIR 原点（work 样本 2704 = `map(1976)`）附近。
* 复合信号：`frame + jam_scale · jam_native_shifted · exp(j2πΔf·n/fs)`，`jam_scale=1`。

### 2.3 参考 CIR（numpy，`CirOperator`）

逐字复刻 `gr-uwb/include/gnuradio/uwb/uwb_radar_cir_core.h` +
`uwb_radar_cir_estimator.h` + `uwb_pdu_rational_resampler_ccf_65_48.cc` 的映射：

1. 65/48 多相重采样到 998.4 MS/s：`scipy.signal.upfirdn(taps, x, 65, 48)`，**taps 用
   仓库真实文件** `testdata/resampler_65_48/taps_quality_minorder.txt`（2707 taps，
   DC 增益 65）。
2. 原点：
   `origin = map(0) + (map(pre) - map(0)) + llround(cal_native·f_work/f_native)`，
   其中 `map(p) = (130p + (T-1) + 48) // 96`。本数据 → `origin = 28 + 1998 + 678 = 2704`。
3. 相干平均 `avg = mean_k rx[origin + k·SPS - cir_pre : + wlen]`，
   `k = 10 … 10+reps-1`，`wlen = 1016+116-1 = 1131`。
4. 与采样参考码相关：`raw[nn] = Σ_c avg[nn+8c]·code9[c] / 64`，输出 116 复数 tap。

> **与任务书的一处有意分歧**：任务书建议 `scipy.signal.resample_poly`。但
> `resample_poly` 会自行设计滤波器，**不复刻仓库 65/48 块**：对 golden
> `uwb_in/out` 的最大误差 0.65；而 `upfirdn` + 仓库 taps 的误差 **2.4e-7**，与实际
> streaming 块逐样本误差 **6e-8**。因此参考实现用的是仓库 taps + `upfirdn`。

### 2.4 线性分解

`estimate_radar_cir` 对 RX 窗是**严格线性**的（平移 → 平均 → 相关，无门限/门控）。
因此 `CIR(frame + jam) = CIR(frame) + CIR(jam)`，且干扰项与帧无关（同一波形、同一位置）。
这样每帧只需重采样一次 baseline，每个 Δf 只需重采样一次纯干扰项；已用直接复合验证，
最大偏差 **3.7e-10（相对 6e-9）**，与"对复合信号直接算 CIR"等价。

---

## 3. 自检（关键交付）

用 `--frames 100` 的参考 CIR 对比出厂 `cir.cf32`：

```
SELF-CHECK vs .../cir.cf32:
  corr mean = 1.000000   corr min = 1.000000
  |CIR| scale = 0.999969
  peak-tap agree = 1.000
  max|diff|(frame 0) = 2.15e-08
```

另外用**真实 GR 块链**（`vector_source → rational_resampler_ccf_65_48 →
pdu_rational_resampler_ccf_65_48 → radar_cir_estimator`，仍不开 UHD）对帧 0
复算，与 `cir.cf32[0]` 的最大差 **2.13e-8**，复相关系数 0.9999999，norm
0.0038692 vs 0.00386931。

结论：参考实现与生产估计器在同一数据上**数值等价**（差异仅来自 float32/float64）。

---

## 4. Δf 扫描结果

`jam_scale=1.0`、`jam_mode=preamble`、code 10、100 帧平均。CSV 列：
`df_hz, floor, peak, peak_over_floor, evm_power`。摘录（kHz）：

| Δf (kHz) | floor | peak | peak/floor | evm_power |
|---|---|---|---|---|
| 0 | 0.015864 | 0.059491 | 3.750 | 3771 |
| 50 | 0.000668 | 0.002002 | 2.997 | 3.93 |
| 200 | 0.000373 | 0.000702 | 1.884 | 0.278 |
| 411.339 | 0.000354 | 0.000336 | 0.949 | 0.165 |
| 450 | 0.000342 | 0.000311 | 0.909 | 0.093 |
| 480 | 0.000355 | 0.000546 | 1.538 | 0.175 |
| 490.339 | 0.000330 | 0.000305 | 0.926 | 0.032 |
| **491.339** | **0.000324** | **0.000311** | **0.961** | **3.17e-6** |
| 492.339 | 0.000330 | 0.000408 | 1.237 | 0.032 |
| 500 | 0.000324 | 0.000315 | 0.972 | 5.96e-5 |
| 700 | 0.000347 | 0.000453 | 1.302 | 0.134 |
| 900 | 0.000617 | 0.000750 | 1.216 | 2.78 |
| 982.68 | 0.014601 | 0.020321 | 1.392 | **2588.8** |
| 1100 | 0.000485 | 0.000594 | 1.226 | 1.25 |

* 实测最小：`df = 491338.6 Hz`，`evm = 3.17e-6`。
* 相对 Δf=0 的干扰抑制：`3771 / 3.17e-6 = 1.19e9` → **90.8 dB**。
* Δf=0 的 jammer 峰在 tap **58**；零点处 peak/floor < 1（无干扰峰，只剩 3.2e-4 噪底）。
* **982.68 kHz 是峰不是零点**（evm 2589，接近 Δf=0）——这与 §5 的 Dirichlet 核一致，
  也说明方案文档里"整数点也是零点"的说法需要更正。

---

## 5. 零点位置与理论对比

### 5.1 Dirichlet 核

CIR 在固定格点 `origin + k·SPS` 相干平均 `N_eff` 次，对周期性干扰的频响为

```
D_N(x) = sin(π·N·x) / (N·sin(π·x)),   x = Δf · SPS / f_work = Δf · T_sym
```

* 零点：`x = m/N`（m 非 N 的倍数），即 `Δf = m · f_work/(N·SPS)`；
* 峰值：`x = 整数`（即 `Δf = m·f_work/SPS = m·982.68 kHz`），`|D|=1`；
* `x = 1/2`（`Δf = f_work/(2·SPS) = 491.34 kHz`）在 **N 为偶数**时正好是零点
  （`m = N/2`）。

### 5.2 N_eff = 114，不是 118

估计器请求 `cir_reps = sync_reps - cir_skip = 128 - 10 = 118`。但生产链只把
`publish_native = 95440` 个原生样点送进重采样器，得到 129297 个 work 样点；
`origin=2704`、`wlen=1131` 时，`k=124…127` 的窗越过末尾被丢弃，**实际有效 114 次**。

→ 零点间距 = `998.4e6 / (114 × 1016) = 8619.975 Hz`。

### 5.3 实测零点梳齿

* 细扫（0–1.1 MHz，2 kHz 步长）的全局最小落在 **862.000 kHz = 100 × 8619.975 Hz**；
* 491.3386 kHz = **57.0000 × 8619.975 Hz**，即 `m = N/2`；
* 因此 491.34 kHz 确实是零点，但**它不是最低频零点**：Dirichlet 第一个零点是
  `m=1` → 8.62 kHz。方案文档里的 `f_work/(2·SPS)` 公式对应的是
  "每 SYMBOL 相位推进半个周期"的那一族零点（m 为 N/2 的奇数倍），不是 m=1。
  两者不矛盾：491.34 kHz 属于梳齿，只是序号 m=57。

> 需要修正方案文档 §1.3 的两处：① `N` 应为实测 **114**（8.62 kHz 而非 8.3 kHz）；
> ② "N 为偶数时整数点与半整数点都是零点"应改为"**半整数点是零点，整数点是峰值**"。

---

## 6. 零点宽度（200 Hz 细扫）

以零点两侧第一个 Dirichlet 旁瓣峰（±4310 Hz 处，evm≈0.219）为参考：

| 判据 | 宽度（总） | 相对零点 |
|---|---|---|
| −3 dB | ≈ 4.0 kHz | ±2.0 kHz |
| −6 dB | ≈ 2.8 kHz | ±1.4 kHz |
| −10 dB | ≈ 1.6 kHz | ±0.8 kHz |
| −20 dB | ≈ 0.4 kHz | ±0.2 kHz |
| 相邻零点 | 17.24 kHz | ±8.62 kHz |

旁瓣峰相对 Δf=0 约 `1/N² ≈ −41 dB`，与理论 `1/114²` 一致。**零点极窄**：
±4 kHz 之外就基本离开抑制区。这正是"商用晶振未知 Δf + 温漂 + 符号时钟 ppm 差"能
轻易毁掉零点的量化依据。

---

## 7. 诚实局限

1. **参考 CIR ≠ 生产块（但有对照）**：本文用 numpy 参考实现；虽然与出厂 `cir.cf32`
   自检 corr=1.000000、与真实 GR 块链差 2e-8，但仍不是生产块本身。已尽量逐字复刻其
   数学（含 65/48 映射、原点、`cir_reps=0→reps-skip`、`sample_count` 截断导致的
   114 次有效平均）。
2. **重采样器**：生产块用仓库 65/48 FIR core（VOLK/AVX2 macroblock）；本文用
   `scipy.signal.upfirdn` + 同一 taps 文件。二者对 golden 差 ≤2.4e-7，对 streaming
   块差 ≤6e-8，量级可忽略。任务书建议的 `resample_poly` 因会自设滤波器而未采用。
3. **单时钟**：干扰与 sensing 共用同一采样时钟，**没有建模符号时钟偏差/滑移**。
   真实商用设备每 114 次平均内若有 ~1 个样点滑移，会破坏周期性并抬高零点底部——本
   实验只能给出"无时钟误差下的算法极限"。
4. **纯相加、无 RF 非理想**：数值直接相加，未建模 RX 压缩、IMD、PA 非线性、IQ 失衡、
   相位噪声、天线/多径。任务书要求的 `--jam-scale` 从低起（实机 <50% FS）对应这一项。
5. **幅度任取**：`jam_scale=1.0`（干扰 rms 0.159 对 capture rms 0.049）使干扰远高于
   sensing 噪底，便于看零点形状；绝对深度（90.8 dB）依赖此幅度与 100 帧平均，但**零点
   位置与宽度与幅度无关**。
6. **`preamble` 模式为主**：默认截断为纯周期 SYNC。`--jam-mode packet`（含非周期
   SFD/STS/payload）会抬高零点底部，本次未展开。
7. 本数据 `cal_delay_native=501`（CG400 的朴素比例），并非 CG600 实测的 1092.5；因此
   sensing 自身泄漏不在 CIR 窗内，baseline CIR 基本是噪底。这不影响零点结论（零点由
   干扰的重复平均决定），但意味着本实验**没有**同时验证"sensing 泄漏 + 干扰"的相互
   作用。

---

## 8. 复现

```bash
# 完整默认扫描（100 帧，30 个 Δf，写 degradation.csv + png）
python3 gr-uwb/apps/jam_offline_inject.py

# 细扫零点宽度（示例：491.34 kHz ±10 kHz，200 Hz）
python3 gr-uwb/apps/jam_offline_inject.py --no-plot \
  --df-max-hz 1100000 --df-step-hz 2000 --output /tmp/opencode/jam_offline/scan_dense

# 查看帮助（不需要 GR/UHD 绑定）
python3 gr-uwb/apps/jam_offline_inject.py --help
```

输出 CSV 列：`df_hz, floor, peak, peak_over_floor, evm_power`，其中
`evm_power = mean|CIR_jam − CIR_jam0|² / mean|CIR_jam0|²`（对帧与 tap 求平均）。

---

## 9. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-09-14 | P5 离线数字注入完成：自检精确复现 `cir.cf32`；实测 491.339 kHz 零点，深度 90.8 dB，半功率宽 ~4 kHz；确认 N_eff=114、梳齿 8619.975 Hz |
