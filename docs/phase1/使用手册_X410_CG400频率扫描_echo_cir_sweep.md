# 使用手册：X410 CG400 echo CIR 频率扫描

> 入口：[`gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py`](../../gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py)
> 频率计划/命令解析：[`gr-uwb/apps/echo_cir_freq_plan.py`](../../gr-uwb/apps/echo_cir_freq_plan.py)
> 单测：[`gr-uwb/apps/test_freq_plan.py`](../../gr-uwb/apps/test_freq_plan.py)
> 基础链路说明：[`使用说明_X410_CG400自发自收雷达.md`](使用说明_X410_CG400自发自收雷达.md)
> 本机 UHD 绕过：[`../本机UHD_DPDK_AVX512问题与绕过.md`](../本机UHD_DPDK_AVX512问题与绕过.md)

本手册面向“在运行时调节 X410 中心频率、研究 DW3000 在不同 CFO 下对
自发自收 CIR 的干扰”。脚本复用基础脚本
[`x410_cg400_hrp_echo_cir.py`](../../gr-uwb/apps/x410_cg400_hrp_echo_cir.py)
的整条链路，只在**突发之间**改 TX/RX 中心频率。

---

## 1. 它做什么、和基础脚本的关系

```text
UwbHrpPacketSource (998.4, code 9, 64 SYNC, 4z2)
  -> 32/65 一次下采样                  # 491.52 MS/s CG400 原生
  -> TimedUhdEcho（定时 TX+RX 突发）    # 每 burst 之间可 retune
  -> PDU 65/32                        # 升回 998.4
  -> UwbRadarCirEstimator             # 默认按预测时刻估 CIR
  -> UwbCirWriter (+ 可选 UDP)
```

- **TX 与 RX 同步平移同一个 Δ**：`f = --freq(标称) + Δ`。单站自回波相对
  CFO 恒为 0；外部 DW3000 相对 X410 的 CFO 就等于 Δ。这是本脚本的核心。
- 每个 burst 仍是独立的 `num_done` 收流，所以**在 burst 之间 retune 安全**。
- 除频率外，波形/重采样/CIR 估计器与基础脚本完全一致；UDP 格式也完全一致。
- 频率计划与命令解析在 `echo_cir_freq_plan.py`，不依赖 GNU Radio/UHD，
  可离线单测。

---

## 2. 前置条件

| 项 | 值 |
|---|---|
| FPGA | **CG_400**（原生 491.52 MS/s，不是 UC200 737.28） |
| 采样率 | 491.52 MS/s（TX=R1X 同一 radio） |
| TX | 前方面板 ch1 = UHD ch0 / `TX/RX0` |
| RX | 前方面板 ch4 = UHD ch3 / `RX1` |
| 默认地址 | `addr=192.168.10.2`（以 `uhd_usrp_probe --args ...` 实测为准） |
| 标称载波 | `--freq` 默认 6489.6 MHz（DW3000 频点） |
| 默认增益 | TX 40 dB / RX 50 dB |

本机构建 **`ENABLE_UHD_BACKEND=OFF`**：C++ 不链 UHD，射频只走 PyUHD。脚本会
自动补 `PYTHONPATH` 和 DPDK 绕过目录。每次开机后先准备 UHD 绕过（`/tmp`
重启即丢）：

```bash
python3 gr-uwb/apps/prepare_uhd_eal_noret.py
```

需要已构建的绑定：

```text
gr-uwb/build/python/uwb/bindings/uwb_python*.so
```

离线自检（无设备、无 UHD）：

```bash
python3 gr-uwb/apps/test_freq_plan.py
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py --dry-run --freq-mode scan \
  --freq-start 6487.6e6 --freq-stop 6491.6e6 --freq-step 1e6 --freq-dwell 20 \
  --output /tmp/x410_sweep_dry
```

---

## 3. 三种模式速查

| 模式 | 用途 | 关键参数 |
|---|---|---|
| `fixed`（默认） | 只用标称频率，等同基础脚本 | `--freq` |
| `scan` | 从起点按步进扫到 max，每点 dwell 发 | `--freq-start/--freq-stop/--freq-step/--freq-dwell/--freq-scan` |
| `manual` | 运行时 stdin 输入频率跳频 | `--freq-mode manual --pulses 0` |

最小示例：

```bash
# fixed（等价基础脚本，A/B 对照用）
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py \
  --args addr=192.168.10.2 --freq-mode fixed \
  --gain-tx 40 --gain-rx 50 --no-udp --output /tmp/x410_fixed

# scan：±2 MHz，步进 1 MHz，每点 20 发，单次扫完即停
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py \
  --args addr=192.168.10.2 --freq-mode scan \
  --freq-start 6487.6e6 --freq-stop 6491.6e6 --freq-step 1.0e6 \
  --freq-dwell 20 --rate-hz 100 --duration-s 10 \
  --gain-tx 40 --gain-rx 50 --no-udp \
  --output /tmp/x410_sweep_pm2mhz

# manual：跑到输入 q 为止（--duration-s 0 让 pulses 变 0 -> 无限）
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py \
  --args addr=192.168.10.2 --freq-mode manual \
  --rate-hz 100 --duration-s 0 \
  --gain-tx 40 --gain-rx 50 --no-udp \
  --output /tmp/x410_manual_hop
```

---

## 4. 频率模式详解

### 4.1 `scan`

- 频率序列：`start + i*step`，`i = 0..n-1`，`n = floor((stop-start)/step)+1`
  （`stop` 恰好落在格点上时包含 `stop`）。
- 每个频点发 `--freq-dwell` 个脉冲。
- `--freq-scan once`（默认）：**单次扫完即停**，总脉冲
  `pulses = n × dwell`，会覆盖 `--pulses`（只打印一条告警）；`--duration-s`
  对长度无效，`--rate-hz` 仍用于设 PRI。
- `--freq-scan cycle`：扫完回到 `start` 继续；长度由 `--pulses` 或
  `--rate-hz × --duration-s` 决定。
- 每个频点内 `pulse_id // dwell == 常数`，写进产物的 `dwell_index`。
- 每次 retune 后重新对时 `now + --freq-settle-s`，所以 dwell 别设太小。

### 4.2 `manual`

后台 daemon 线程读 stdin；命令在下个 burst 边界生效并保持，直到下一条命令。

| 输入 | 含义 |
|---|---|
| `+50` | 相对标称 **+50 kHz**（裸数字默认按 kHz） |
| `491` | 相对标称 +491 kHz |
| `-2.5` | 相对标称 −2.5 kHz |
| `+5MHz` | 相对标称 +5 MHz（显式单位覆盖默认） |
| `-10e6` | 相对标称 −10 MHz（科学计数法按 Hz） |
| `6489600kHz` | 绝对频率 6.4896 GHz |
| `6489.6MHz` | 绝对频率 6.4896 GHz |
| `f 6.5e9` | 绝对频率（显式） |
| `off +1MHz` / `offset -1MHz` | 显式偏移 |
| `status` / `?` | 打印当前目标频率与偏移（kHz） |
| `q` / `quit` / `exit` | 结束运行 |
| 空行 | 等同 `status` |

**裸数字默认单位 = `--freq-unit`（默认 `khz`）**：无后缀、无指数的数字按
kHz 解释（`+50` = +50 kHz，`491` = 491 kHz）。带单位后缀
（`GHz/MHz/kHz/Hz`，大小写不敏感）或科学计数法（`10e6` = 10 MHz，按 Hz）
时以显式为准。运行长度：

- `--pulses 0` 或 `--rate-hz N --duration-s 0` → 跑到 `q`；
- 否则跑给定脉冲数（默认 8）。

### 4.3 `fixed`

只用 `--freq`。频率计划为单点，行为与基础脚本一致，便于对照。

---

## 5. 参数总表

### 5.1 频率控制（本脚本新增）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--freq-mode` | `fixed` | `fixed` / `scan` / `manual` |
| `--freq-start` | `--freq` | scan 起点（Hz） |
| `--freq-stop` | 无 | scan 终点（Hz，含落点）；scan 必填 |
| `--freq-step` | `0` | scan 步进（Hz，>0）；scan 必填 |
| `--freq-dwell` | `20` | 每个频点脉冲数（≥1） |
| `--freq-scan` | `once` | `once` 扫完即停 / `cycle` 循环 |
| `--freq-settle-s` | `0.05` | 每次 retune 后到下个定时突发的间隔（s） |
| `--freq-unit` | `khz` | manual 裸数字默认单位（`hz/khz/mhz/ghz`）；显式单位/科学计数法优先 |
| `--peak-target-tap` | `0` | 把首峰锁到该 tap（0=关闭，用固定 `--cal-delay-native`） |
| `--peak-first-rel` | `0.5` | 首峰判定：第一个 `≥ 该比例 × max(|CIR|)` 的 tap |
| `--peak-search-start` | `0` | 从该 tap 起才找首峰 |
| `--dry-run` | 关 | 只打印频率序列，不打开 UHD |

### 5.2 继承参数（与基础脚本相同）

**调度 / 长度**

| 参数 | 默认 | 说明 |
|---|---|---|
| `--pulses` | `8` | 脉冲数；scan once 被覆盖，manual 下 0=无限 |
| `--pri-s` | `0.05` | 脉冲间隔（s），默认 20 Hz |
| `--rate-hz` | `0` | 若 >0：`PRI=1/rate`，`pulses=round(rate*duration)` |
| `--duration-s` | `10` | 配合 `--rate-hz`；0 在 manual 下=无限 |
| `--arm-delay-s` | `0.25` | 首次对时提前量 |
| `--min-lead-s` | `0.002` | 低于此 lead 记为 `late` |
| `--pre-guard-us` | `2.0` | RX 窗在 TX 前的保护 |
| `--tail-guard-us` | `20.0` | RX 窗尾部保护 |
| `--rx-pad-us` | `8.0` | TX 突发后额外 native 采样 |

**RF**

| 参数 | 默认 | 说明 |
|---|---|---|
| `--args` | `addr=192.168.10.2` | UHD 设备参数 |
| `--freq` | `6489.6e6` | **标称**中心频率（TX+RX） |
| `--gain-tx` / `--gain-rx` | `40` / `50` | 增益（dB） |
| `--tx-channel` / `--rx-channel` | `0` / `3` | UHD 通道 |
| `--tx-antenna` / `--rx-antenna` | `TX/RX0` / `RX1` | 天线口 |

**波形 / 前导 / CIR**（必须 TX 与估计器一致）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--code-index` | `9` | preamble code 9–12 |
| `--preamble-length` | `64` | SYNC 重复数 32/64/128/256/512/1024 |
| `--sync-reps` | — | 旧别名，另接受 2048 |
| `--no-sts` | 关 | 去掉 STS |
| `--pulse-shape` | `linear` | `linear/minphase/legacy/gaussian/blackman/external` |
| `--pulse-taps` | — | 外部脉冲核（配 `external`） |
| `--pulse-sigma-ns` / `--pulse-bw-mhz` | `2.5` / `200` | 高斯/带宽类脉冲参数 |
| `--psdu-hex` | 内置 | PSDU 字节（hex） |
| `--taps` | 内置 | 65/32 重采样系数文件 |
| `--template` | 自动 | CIR 模板；默认写到 `<output>/sync_template_live.cf32` |
| `--cal-delay-native` | `334` | 泄漏峰校准（native 样本） |
| `--sfd-search-margin` | `128` | SFD 搜索半窗（998.4 样本） |
| `--sfd-threshold` | `0.12` | SFD 门限 |
| `--sync-refine-margin` / `--sync-refine-threshold` | `32` / `0.02` | SYNC 细化 |
| `--est-queue` | `64` | 估计器队列；溢出=真丢帧 |

**产物 / 调试**

| 参数 | 默认 | 说明 |
|---|---|---|
| `--output` | 必填 | 输出目录 |
| `--dump-rx` | 关 | 每脉冲存 `rx_iq/pulse_XXXX.cf32` |
| `--dump-sc16` | 关 | 存原生 SC16：`capture.iq`/`capture.jsonl`/`metadata.json`/`tx_491p52.sc16` |
| `--require-sfd` | 关 | 恢复 SFD 门控（默认按预测时刻估 CIR） |

**UDP**

| 参数 | 默认 | 说明 |
|---|---|---|
| `--udp-host` | `133.133.133.132` | 目标 IP；空串=关闭 |
| `--udp-port` | `12345` | 目标端口 |
| `--no-udp` | 关 | 关闭 UDP（只落盘） |

---

## 6. 产物与数据格式

以 `--output DIR` 为例：

| 文件 | 内容 |
|---|---|
| `cir.cf32` / `cir_norm.cf32` | 每 ok 帧 116 个 complex64 tap |
| `cir.jsonl` | 每帧一行：`pulse_id/status/peak_tap/cir_peak_metric/...` |
| `freq_sweep.jsonl` | **每脉冲频率**（见下） |
| `echo_timing.jsonl` | 每脉冲定时 + 频率字段 |
| `summary.json` | 运行汇总 + **逐频表 `cir_by_freq`** + `freq_plan_hz` |
| `--dump-sc16` | `capture.iq`/`capture.jsonl`（含频率）/`metadata.json`/`tx_491p52.sc16` |

### 6.1 `freq_sweep.jsonl`

每脉冲一行：

```json
{"pulse_id": 40, "freq_hz": 6489600000.0, "freq_offset_hz": 0.0,
 "tx_freq_actual": 6489599872.0, "rx_freq_actual": 6489599872.0,
 "dwell_index": 2}
```

- `freq_offset_hz = freq_hz - --freq`，即 **DW3000 相对 X410 的 CFO**（当
  `--freq` 设成 DW3000 频点 6489.6 MHz 时）。
- `tx_freq_actual`/`rx_freq_actual` 是 UHD 读回的实际锁定值（有量化）。

### 6.2 `summary.json` 的 `cir_by_freq`

按 `pulse_id` join `cir.jsonl` 后按频率聚合，无需 C++ 改动：

```json
"cir_by_freq": [
  {"freq_hz": 6487.6e6, "freq_offset_hz": -2.0e6, "n": 20, "ok": 20,
   "fail": 0, "status_hist": {"ok": 20},
   "peak_tap_mean": 22.0, "metric_mean": 0.017, "metric_max": 0.021},
  ...
]
```

### 6.3 距离轴

`cir.jsonl` 里 `zero_delay_tap`（默认 16）、`peak_tap`；每 tap 距离
`range_m_per_tap = c/(2×998.4e6) ≈ 0.150 m`，峰相对校准时延
`peak_delay_from_calibration_ns = (peak_tap - zero_delay_tap)/998.4e6 × 1e9`。

### 6.4 CIR 首峰对齐（`--peak-target-tap`）

**默认 CIR 不是对齐到首峰的**：CIR 轴锚在“预测 TX 时刻 + 标定延迟”，
`zero_delay_tap = cir_pre = 16` 只是标定参考；实际首峰落在硬件给出的
`peak_tap`（本机约 22）。想让首峰固定在第 N 个 tap，用：

```bash
python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_sweep.py \
  --args addr=192.168.10.2 --freq-mode fixed \
  --peak-target-tap 30 \
  --gain-tx 40 --gain-rx 50 --no-udp --output /tmp/x410_align30
```

原理：CIR 网格满足

```text
tap = cir_pre + (path_delay_work - cal_work)
```

所以 `cal_work` 每 +1 work sample，首峰就 −1 tap。脚本在每个脉冲后读回
CIR taps，按

```text
cal_work += (first_peak_tap - target_tap)
```

修正，并把新的 `calibration_delay_native_samples` 放进**下一个脉冲**的
metadata（`65/32` 映射回 work 域）。无需 C++ 改动，通常 1–2 个脉冲收敛。

- 首峰定义：从 `--peak-search-start` 起，第一个
  `≥ --peak-first-rel × max(|CIR|)` 的 tap。
- 日志：启动打印 `[align] first peak -> tap 30 ...`，结束打印
  `[align] target=30 frames=... applied=... last_first_peak=30 ...`。
- 产物：`summary.json` 增 `align_*` 与 `peak_*`；`freq_sweep.jsonl` /
  `echo_timing.jsonl` 每脉冲记录 `align_first_peak_tap`、`align_error`、
  `cal_delay_native`。
- **与扫频配合**：retune 引起群时延漂移时，servo 会自动跟随，使跨频点首峰
  落在同一 tap，便于逐频比较。这正是做 CFO 扫描时想要的效果。
- `--peak-target-tap 0`（默认）关闭，保持固定 `--cal-delay-native`。

**注意**：

- 若强干扰（如 DW3000）先于自泄漏出现且幅度更大，`first_rel` 可能选中它；
  用 `--peak-search-start` 跳过前方、或调 `--peak-first-rel`。
- 目标 tap 要小于 `cir_pre+cir_post=116`，并给后续多径留空间。
- servo 只改每个脉冲 metadata 里的校准值，不覆写命令行 `--cal-delay-native`；
  `summary.json` 同时记录 `align_cal_delay_base_native` 与最终值。

**想手调**：`cal_delay_native += (peak_tap - target_tap) × 32/65`。
例如从 `peak_tap=22` 到 30：`334 + (22-30)×32/65 ≈ 330.06`。

---

## 7. 实时 UDP

- **默认开启**（除非 `--no-udp` 或 `--udp-host ""`）。每脉冲一帧，非阻塞。
- **新旧脚本统一发 `UCR2`**（44 字节头 + 116×`complex64`，失败也发、taps 填 0）：

  ```text
  magic "UCR2" | pulse_id u32 | status u16 | tap_count u16
  | sfd_metric f32 | cir_peak_metric f32 | peak_tap i32 | estimator_us u32
  | freq_hz f64 | freq_offset_hz f64
  ```

- 基础脚本 `x410_cg400_hrp_echo_cir.py` 频率固定，发
  `freq_hz=--freq`、`freq_offset_hz=0`；本脚本发当前中心频率与相对 `--freq`
  的 CFO。
- 频率用 `f64`：6.5 GHz 下 `f32` 分辨率约 512 Hz，会吃掉 kHz 级 CFO。
- 每个脉冲的频率在发 CIR 之前就按 `pulse_id` 登记；查不到时两项填 `NaN`
  （接收端显示 `-`），格式不变。
- 对端 `cir_udp_recv.py` 支持 `UCR2`/旧 `UCR1`/裸 taps：

```bash
python3 gr-uwb/apps/cir_udp_recv.py --bind 0.0.0.0 --port 12345
```

对端日志会打印当前帧的频率，例如
`... freq=6494.600000MHz(off+5000.000kHz) ... v=2`。

---

## 8. live 行怎么读

每秒一行：

```text
live dt=1.001 echo_ok_hz=20.0 cir_ok_hz=20.0 cir_fail_hz=0.0 est_q=0
     est_drop=0 wr_hz=20.0 udp_hz=0.0 udp_ok_hz=0.0 udp_eagain=0
     service_us_mean=356 max=380 pri_hz=20.0
```

| 列 | 含义 |
|---|---|
| `echo_ok_hz` | 定时突发成功速率（射频好≠CIR 好） |
| `cir_ok_hz` / `cir_fail_hz` | CIR 成功/失败速率 |
| `est_q` / `est_drop` | 估计器队列深度 / 累计丢帧 |
| `wr_hz` | 写盘速率 |
| `udp_hz` / `udp_ok_hz` | **已启用时**的发送速率 / 其中 ok 速率 |
| `udp_eagain` | 累计发送失败 |
| `service_us_mean/max` | 单帧 CIR 估计耗时 |

**UDP 三列全 0 且 `udp_eagain=0` = UDP 被关闭**；若要确认看启动日志有没有
`udp_cir <host>:<port> framed=UCR2 ...`。启用但发不出去会是
`udp_eagain` 持续增长。

`sched` 行：

```text
sched 200/2147483648 ok=200 fail=0 late=0 sc16=0 host_s=10.201 freq=6489.600MHz off=+0.000MHz retune=0
```

`retune` 是累计 retune 次数；scan once 正常应为约 `n-1` 次。

---

## 9. CFO 实验流程建议

1. 先 `--freq-mode fixed` 跑一遍，确认基线（`late=0`、`cir_ok=总数`、
   `peak_tap` 稳定）。
2. 小范围 scan 验证：
   `--freq-start 6487.6e6 --freq-stop 6491.6e6 --freq-step 1e6 --freq-dwell 20`。
   检查 `summary.json.cir_by_freq` 与 `late`。
3. 扩大范围/加 dwell 做正式测量；频点不要超过前端可用范围。
4. 用 `--dump-sc16` 需要逐脉冲原始 IQ 时再开（占盘大）。
5. 分析：直接读 `summary.json` 的 `cir_by_freq`；或自行 join。

```python
import json, sys
import numpy as np
out = sys.argv[1]
freq = {}
for ln in open(f"{out}/freq_sweep.jsonl"):
    r = json.loads(ln); freq[r["pulse_id"]] = r
rows = []
for ln in open(f"{out}/cir.jsonl"):
    r = json.loads(ln)
    fr = freq.get(r["pulse_id"])
    if fr and r.get("status") == "ok":
        rows.append((fr["freq_offset_hz"], r["cir_peak_metric"], r["peak_tap"]))
a = np.array(sorted(rows))
for off in sorted(set(a[:, 0])):
    m = a[a[:, 0] == off]
    print("%+8.3f MHz  n=%3d  metric=%.5f  peak_tap=%.2f"
          % (off / 1e6, len(m), m[:, 1].mean(), m[:, 2].mean()))
```

---

## 10. 注意事项 / 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| UDP 三列全 0 且 `eagain=0` | UDP 关闭：去掉 `--no-udp`，或设有效 `--udp-host` |
| `udp_eagain` 增长 | socket 发不出去：检查路由/接收端 |
| `late>0` | 增大 `--arm-delay-s` / `--freq-settle-s`，加大 `--freq-dwell`，或降 `--rate-hz` |
| `retune` 正常但不跳频 | scan 起点=标称时首个频点不 retune；manual 需输入命令 |
| `retune_fail>0` | 频点超设备范围/异常，保留旧频点；退出码 3 |
| 主峰位置随频率漂 | `--cal-delay-native` 是定频校准；必要时 `--require-sfd` |
| DW3000 在时大量 `sfd_failed` | 默认用预测时刻估 CIR；只有 `--require-sfd` 才会被 SFD 门控打掉 |
| `est_drop>0` | 估计器跟不上（队列 64）；参考基础脚本调 `--sfd-search-margin` |
| `--freq-dwell 1` 且高频 | 每个 burst 都 retune，开销大易 `late`；建议 dwell ≥20 |

**频率方向**：scan 默认从 `--freq-start`（默认标称）向 `--freq-stop` 递增；
`--freq-start` 也可设得比标称低。步进/起止都用 Hz 写（如 `-2e6`、`0.5e6`）。

---

## 11. 退出码

| 码 | 含义 |
|---|---|
| 0 | 成功 |
| 3 | 有失败：`late`、CIR 缺号/失败、写盘不符、`retune_fail` 等 |

`--dump-sc16` 时的判据为 `echo_ok == pulses_done`、`late == 0`、
`sc16_packets == pulses_done`；否则另加 `wr_ok == pulses_done`、
`cir_ok == pulses_done`、`missing_count == 0`、`retune_fail == 0`。

---

## 12. 与基础脚本的取舍

- 只研究单一频率 / 追求最小依赖：用
  [`x410_cg400_hrp_echo_cir.py`](../../gr-uwb/apps/x410_cg400_hrp_echo_cir.py)。
- 需要运行时改频（CFO 研究）：用本脚本；`--freq-mode fixed` 即等价基础脚本。
- 基础脚本的 `parse_args()` 已拆出 `build_parser()`，本脚本通过
  `argparse parents` 继承其全部选项，二者参数不会漂移。
- 首峰对齐 servo（`--peak-target-tap`）目前只在本脚本；基础脚本仍用固定的
  `--cal-delay-native`。

**未声称**：扫频实机 soak（retune 抖动/`late` 统计）、跨频点群时延校准、
DW3000 逐频干扰结论。首次实机使用请按第 9 节从小范围做起。
