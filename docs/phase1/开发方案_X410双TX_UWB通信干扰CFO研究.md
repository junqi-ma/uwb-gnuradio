# 开发方案：X410 双 TX —— UWB 通信干扰对 sensing CIR 的 CFO 研究

> 状态：**开发中（v1 规划已定，P1/P2/P5 并行开工）**
> 分支：`feature/uwb-monostatic-radar`
> 日期：2026-09-14
> 范围：新增一个 **jamming app**，用 X410 第二路 TX 生成一路不同 preamble code 的
> HRP 通信干扰，CFO 由 NCO 精确设定；不改动现有 sensing CIR 算法、不改动通信链。

---

## 1. 背景与目标

### 1.1 问题

现有"CFO 干扰研究"只有一个手段：`x410_cg400_hrp_echo_cir_sweep.py` 把 **sensing 的
TX+RX 一起**平移 Δ，让外部商用设备（DW3000 / QM35）在 sensing 基带落在 `CFO = Δ`
（见 `使用手册_X410_CG400频率扫描_echo_cir_sweep.md` §1）。

这条链的干扰源是**外部商用设备**，其载波 CFO 不可控且在漂移、其符号时钟与 X410 不同
源、其脉冲形状未知。因此无法区分：

- 干扰抵消不理想，是**商用 TX 质量/时钟**造成的？
- 还是 **sensing 算法 / 接收链**本身的极限？

### 1.2 目标

让 X410 **自己**发第二路干扰：

```
sense  TX ch0  code 9  @ f0
jammer TX ch1  code 10 @ f0 + Δf      ← 新增
RX        ch3  @ f0
```

- Δf 由 jammer 通道的 NCO 精确设定（亚 Hz 分辨率、零漂移、与 sensing 同参考）；
- jammer 与 sensing **同一个采样时钟**，其 preamble 重复格点与 sensing 的 1016 样本
  **完全一致**；
- jammer 波形、code、脉冲形状、SFD 全部可控。

这就能做**受控对照**：

| 实验 | 干扰源 | 结论含义 |
|---|---|---|
| A | 离线数字注入 | 纯算法/理论极限 |
| B | X410 双 TX | 真实 RF 链 + 精确 CFO |
| C | 商用设备 | 真实场景 |

- 若 **B 仍然抵不干净** → 问题不在商用 TX，而在算法/接收链/波形失配；
- **C 与 B 之差** → 商用设备 TX/时钟的贡献。

### 1.3 理论（为什么是 491 kHz）

CIR 估计器在每个重复偏移 `origin + k·SPS` 处**相干平均** `k = cir_skip .. sync_reps-1`。
对基带为 Δf 的周期性干扰，平均后的等效响应是 Dirichlet 核：

```
D_N(x) = sin(pi*N*x) / (N*sin(pi*x)),     x = df * SPS / f_work
f_work = 998.4e6,  SPS = 1016
峰间距（"梳齿"）= f_work / SPS = 982.68 kHz
```

- 零点在 `x = k/N`（k 不是 N 的倍数）→ 零点梳齿间距 `f_work/(SPS*N)`；
- **整数点 `Δf = m·982.68 kHz` 是峰不是零**；`491.34 kHz` 是 `x = 0.5`，
  因为 N 为偶数而恰好落在零点上（它是相邻两个峰的中点）。

> **离线数字注入实测（P5，2026-09-14）**：参考实现对 100 帧注入 code-10 干扰，
> 自检与生产 `cir.cf32` 逐 tap 差 ≤2.15e-8、相关 1.000，所以结论可信：
> - 有效平均次数 **N_eff = 114**（不是 118）：`sync_reps-skip=118` 个窗里只有发布
>   前缀够 114 个，所以零点梳齿间距是 **8.619975 kHz**；
> - `491.3386 kHz = 57 × 8.619975 kHz`，正好是一个零点；
> - 该点 EVM 相对 Δf=0 降低 **≈ 90 dB**（无噪声、无 RF 非理想的参考实现）；
> - 半功率零点宽度 ≈ **4.0 kHz**（±2 kHz），−10 dB 宽 ≈1.6 kHz；`982.68 kHz`
>   处是**峰**（EVM 回到高位），印证上面"整数点是峰"。

**这直接给出"商用设备抵消不理想"的高概率主因**：零点梳齿只有 8.62 kHz 间距、
每个零点只有 ±2 kHz 量级宽，而商用晶振的 Δf 未知且温漂；再叠加其符号时钟与 X410
的 ppm 差（N=114 次里累积样本滑移，破坏周期性）、脉冲形状失配、非 preamble 段
（SFD/PHR/payload 非周期）、码互相关旁瓣、RX 压缩非线性。**TX EVM 很可能是次要
因素。** 这正是双 TX 实验要证伪/证实的。


---

## 2. 现有机制与局限

- sensing 链：`TimedUhdEcho`（python）或 `CppPduEcho`（cpp-pdu，默认）→
  `pdu_rational_resampler_65_48` → `radar_cir_estimator` → `cir_writer` + `CirUdpSink`。
- 两个后端都是**单 TX + 单 RX**：
  - `TimedUhdEcho`：`sa_tx.channels = [tx_ch]`，`_send()` 发 `(1, N)`。
  - `CppPduEcho`：C++ `UwbRealtimeEchoTimer`，TX 由调度 PDU 驱动，与几何强耦合。
- `--rx-freq-offset` **不是**"sensing↔通信 CFO"旋钮：它把 RX 相对 TX 偏置，外部设备与
  sensing 在基带同时偏移，**相对 CFO 仍为 0**，只是把 sensing 自己弄坏（实测 −23.9 kHz
  时 `metric_mean` 0.071 → 5.3e-3）。造相对 CFO 只能动干扰源（或 TX+RX 一起动）。

---

## 3. X410 双 TX 能力（本机已实测）

| 项 | 结果 |
|---|---|
| TX / RX 通道 | 各 **4 个**，1–8000 MHz 可调（ZBX） |
| 端口映射 | ch0=DB0/RF0, ch1=DB0/RF1, ch2=DB1/RF0, ch3=DB1/RF1 |
| 每通道 SMA | `TX/RX0`、`RX1`（+`CAL_LOOPBACK`/`TERMINATION`） |
| 现接法 | sense TX=ch0(`TX/RX0`@DB0)，RX=ch3(`RX1`@DB1) |
| 多通道 TX 流 | `tx[0,1]`/`[0,2]`/`[0,3]`/`[0,1,2,3]` 均可建 |
| TX+RX 共存 | `tx[0,1]` 与 `rx[3]` 同开后仍可建 |
| 精确频率差 | `set_tx_freq(ch1, f0+491.3e3)` → 回读 `delta = 491300.0 Hz` |

→ jammer 用 **ch1**（与 sense TX 同 DB0、同参考）。

> 复现命令见 §9.1 的 smoke 脚本；上述结论用 `uhd_usrp_probe` + `MultiUSRP` 直接测得。

---

## 4. 系统设计

### 4.1 数据通路

```
sense TX ch0 ─ code 9  @ f0        ┐
jammer TX ch1 ─ code10 @ f0+Δf     ├─ 单条 2 通道 TX 流, 逐脉冲 sample 对齐
                                    ┘
RX ch3 @ f0 → native SC16 → 65/48 PDU → radar_cir_estimator → cir_writer + UDP
```

- **单条 `channels=[tx_ch, jam_ch]` TX 流**：逐脉冲发一个 `(2, L)` 缓冲：
  - 第 0 行 = sensing native（零填充到 L）
  - 第 1 行 = jammer native 按 `--jam-delay-us` 移位
- 两路天然共享同一个 `t_tx`、同一个时钟；jammer 的 preamble 重复格点精确等于 sensing
  的 1016，`491.34 kHz` 零点可以打深。
- **RX 窗几何**：`rx_geometry(..., tx_native_samples=L)`，其中
  `L = max(len(sense), delay_native + len(jam))`。新 app 通过基类新增的
  `tx_geometry_native` 钩子传入 L，保证 CIR 窗覆盖 jammer。

### 4.2 为什么 v1 只用 python 后端

`CppPduEcho` 的 TX 由 C++ `UwbRealtimeEchoTimer` 的调度 PDU 驱动，TX 命令、几何、
fragment 计划全部耦合在 `uwb_echo_scheduler_core.h` / `uwb_uhd_burst_backend.cc`。
加第二 TX 通道要动核心调度 + backend + bindings + dry-run + QA，风险远大于 v1 需要。

jammer 研究是**低速固定频**实验；python 后端足够，且它自带 `--dump-sc16`，产出的正是
分析所需的"sensing+干扰"混合原始 IQ。v1 对新 app **强制 python 后端**，`--echo-backend
cpp-pdu` 明确报错并给出替代命令，不静默降级。

### 4.3 电平与线性度

- sense TX native 已归一化到峰值 0.8；jammer 幅度由 `--jam-scale` 相对 sense 峰值给出。
- 目标：到 RX 的 jammer 幅度 ≈ sensing 泄漏同量级（当前泄漏峰 ~7845/32768 ≈ 24% FS，
  还有 ~12 dB 余量）。jammer 过强 → RX 压缩 → IMD 产生**非周期**寄生，反而破坏零点。
- `--jam-gain-tx` 与 `--jam-scale` 都从低开始；smoke 先看 RX SC16 峰值不超 ~50% FS。

---

## 5. CLI 设计（新 app）

```
--jam-enable                     # 总开关；不开=退化为 base app(单 TX)
--jam-mode {align,continuous}    # align=逐脉冲同缓冲(默认) continuous=独立 TX 流
--jam-channel 1                  # 必须 != --tx-channel/--rx-channel
--jam-antenna TX/RX0
--jam-gain-tx 20
--jam-freq-offset 491340.0       # Hz, 可负; 相对 --freq
--jam-freq-offsets ""            # 可选: "0,100e3,245e3,491.34e3,982.68e3" 占空扫描
--jam-dwell 50                   # 每个 offset 的脉冲数
--jam-code-index 10
--jam-preamble-length 128
--jam-psdu-hex <comm psdu>
--jam-waveform {packet,preamble} # preamble=只取 SYNC 段(纯周期,最适合测零点)
--jam-pulse-shape legacy
--jam-pulse-sigma-ns / --jam-pulse-bw-mhz / --jam-pulse-taps
--jam-delay-us 0.0               # jammer 在 RX 窗内相对 sense 脉冲的位移
--jam-scale 0.3                  # jammer 幅度 / sense TX 峰值(0.8)
--jam-repeat-pri-us ...          # continuous 模式
--dry-run                        # 打印 jam plan + 缓冲几何后退出, 不碰 UHD
```

校验（`validate_jam_args`）：

- `jam_channel ∉ {tx_channel, rx_channel}`；
- `jam_channel ∈ [0,3]`；
- `abs(jam_freq_offset) <= 1e6`；
- `jam_code_index ∈ {9,10,11,12}`；
- `jam_scale > 0`；
- `jam_mode == continuous` 时 `jam_repeat_pri_us > 0`。

---

## 6. 模块与接口（冻结）

### 6.1 新纯函数模块 `gr-uwb/apps/echo_cir_jam_plan.py`

纯 Python（只依赖 numpy，不 import UHD/GR），便于无硬件单测：

```python
JAM_MODES = ("off", "align", "continuous")
DEFAULT_JAM_CODE_INDEX = 10
DEFAULT_JAM_CHANNEL = 1

def jam_freq_hz(center_hz, offset_hz) -> float
def placement_native(delay_us, native_hz) -> int
def combined_tx_len(sense_len, jam_len, delay_native) -> int
def compose_tx_native(sense, jam, delay_native) -> np.ndarray   # (2, L) complex64
def validate_jam_args(dict) -> None                            # ValueError
```

### 6.2 基类扩展 `TimedUhdEcho`（向后兼容）

```python
# __init__(..., tx_channels=None)
self.tx_channels = list(tx_channels) if tx_channels else [self.tx_ch]
sa_tx.channels = self.tx_channels
self.tx_geometry_native = 0          # 0 = 用 len(self._native)
self._tx_payload = self._native      # _send() 发送的负载(1-D 或 (C,N))

# _send(wave,...): wave.ndim==1 -> reshape(1,-1); ndim==2 原样
# set_tx_native(): rx_geometry(tx_native_samples = self.tx_geometry_native or len(self._native))
```

默认路径（单通道、`_tx_payload = _native`）**行为完全不变**。

### 6.3 新 app `gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`

- `build_parser()`：`parents=[base.build_parser(add_help=False)]` + §5 参数组。
- `class JamTimedUhdEcho(base.TimedUhdEcho)`：建 2 通道 TX 流；`set_jam_native()`；
  `_compose()` 生成 `(2, L)`；`_send` 发送 composite。
- `main()`：仿 base，强制 python 后端；建 jammer 波形；打印 jam plan；写 summary。
- 每脉冲记录 `jam_freq_hz` / `jam_freq_offset_hz` / `jam_delay_us`（沿用 sweep 的
  `freq_by_pulse` join 模式）。

---

## 7. 任务分解

| # | 并行 | 任务 | 文件（唯一属主） | 状态 |
|---|---|---|---|---|
| **P1** | ✅ | `echo_cir_jam_plan.py` + 单测 | `echo_cir_jam_plan.py`、`test_echo_cir_jam_plan.py` | **完成**（35 测试通过，numpy-only） |
| **P2** | ✅ | base `TimedUhdEcho` 多通道扩展 + 回归测试 | `x410_cg400_hrp_echo_cir.py`、`test_echo_tx_channels.py` | **完成**（默认单通道不变，sweep dry-run 不回归） |
| **P5** | ✅ | 离线数字注入验证（无硬件） | `jam_offline_inject.py` + 报告 | **完成**（自检 2.15e-8；零点 491.34 kHz，深度 ~90 dB） |
| **S1** | ⛔ P1+P2 | 新 app + main + summary + CMake | `x410_cg400_hrp_echo_cir_jam.py`、`test_jam_app_args.py`、`apps/CMakeLists.txt` | **完成**（dry-run 通过；实机待做） |
| **S2** | ⛔ S1 | 实机 E2E + Δf 扫描 + 报告 | `docs/phase1/测试报告_X410双TX_jammer.md` | 待做（需接好 ch1 天线） |
| **S3** | ⛔ S1 | continuous 模式（独立 TX 流） | 同 S1 app | 待做（v1 已留 `--jam-mode`/`--jam-repeat-pri-us` 接口） |
| **S4** | ⛔ S1 | `--jam-freq-offsets` 占空扫描 | 同 S1 app | **完成**（app 内建 dwell 扫描 + 群重锚） |


---

## 8. 验证阶梯与判据

1. **离线数字注入（P5）**：jam-off dump + 合成 code-10 jammer，细扫 Δf（491.34 kHz
   附近 1 kHz 分辨率）→ 无任何 RF 非理想，先证明理论零点存在且位置正确。
2. **双 TX smoke（S2 前置）**：确认 jammer 落在 Δf、RX 不压缩、sense 定时不受影响。
3. **双 TX Δf 扫描（S2）**：得到 `metric_mean`、径外底噪功率、`peak/floor`、相对
   jam-off 的 CIR 误差向量功率 vs Δf 曲线。**判据：零点若仍浅 → 非商用 TX 问题。**
4. **商用设备对照（后续）**：与 3 的差 = 商用 TX/时钟贡献。

评价指标：
- `cir.jsonl` 的 `cir_peak_metric` / `metric_mean`；
- CIR 径外（排除泄漏峰及其邻域）平均功率；
- `peak/floor`；
- 相对 jam-off 基线的 CIR 误差向量功率。

---

## 9. 风险、边界与回退

| 风险 | 处置 |
|---|---|
| 同 DB 双 TX 的 ATR/DSA 冲突 | smoke 先验证；ZBX 每通道独立两 pin，预期无冲突 |
| jammer 过驱 → RX 压缩 → IMD 破坏零点 | `--jam-scale`/`--jam-gain-tx` 从低起；盯 SC16 峰值 <50% FS |
| python 后端 retune/servo 时 TX underflow | v1 固定频、不用 peak servo；扫描低速 + re-anchor |
| ch1 口未接负载 → 反射 | OTA smoke **必须先接好第二天线**；未接前不发射 |
| cpp-pdu 不支持 jammer | 明确报错 + 替代命令，不静默降级 |

回退：`--jam-enable` 不开 → 行为等同 base app（单 TX）。

---

## 10. 明确不做（v1）

- cpp-pdu 后端的 jammer；
- 真解调 / 解 payload；
- `CAL_LOOPBACK` 内部注入；
- 器件级自动电平标定；
- 与 DW3000 的逐包对照结论。

---

## 11. 变更记录

| 日期 | 变更 |
|---|---|
| 2026-09-14 | 初稿：能力实测、理论零点、v1 架构、任务分解 |
| 2026-09-14 | P1/P2/P5/S1/S4 完成；§1.3 按离线实测更正（N_eff=114、零点梳齿 8.62 kHz、整数点是峰、深度 ~90 dB）；QP 42 项 41 通过（唯一失败为既有环境相关吞吐门限） |
