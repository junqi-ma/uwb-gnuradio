# 测试报告：X410 双 TX HRP jammer —— UWB 通信干扰对 sensing CIR 的 CFO 零点

> 状态：**实测完成（v1 align 模式）**
> 分支：`feature/uwb-monostatic-radar`　commit：`ee447c4` + 本次记录
> 日期：2026-09-14
> 设备：X410 serial 2607179，FPGA `CG_600`，UHD `4.6.0.HEAD-0-g50fa3baa`，737.28 MS/s，DPDK
> 目标文件：`gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py`（见同目录开发方案文档）

---

## 1. 目的

验证通信干扰（不同 preamble code 的 HRP 信号）对单站 sensing CIR 的影响随 CFO 的变化，
特别是"CFO 处于 491 kHz 时 preamble 干扰自我抵消"这一说法，并回答：

> 外部商用设备抵消不理想，是**商用 TX 质量**，还是**算法/时钟**？

用 X410 自己发第二路干扰，CFO 由 NCO 精确设定且与 sensing 同钟，从而排除商用设备的
CFO/时钟不确定性。

---

## 2. 配置

```
sense  TX ch0 (TX/RX0, DB0/RF0)  code 9   @ 6489.6 MHz
jammer TX ch1 (TX/RX0, DB0/RF1)  code 10/9 @ 6489.6 MHz + df   (第二根天线,靠近 RX)
RX        ch3 (RX1,   DB1/RF1)   @ 6489.6 MHz
```

| 参数 | 值 |
|---|---|
| preamble-length | 128（sensing 与 jammer 都是） |
| jammer 波形 | `preamble`（纯 SYNC 段，周期性最好） |
| gain-tx / gain-rx | 50 / 60 dB |
| jam-gain-tx / jam-scale | 60 dB / 0.8（强干扰工况） |
| PRI | 20 ms（50 Hz）——python 后端 200 Hz 会 `late`/TX underflow，CIR 崩 |
| 每点 dwell | 20 脉冲 |

**判据**：CIR 泄漏峰在 tap≈16；观察量取 **径外底噪 floor = mean(|CIR|[40:116])**
（峰值 `cir_peak_metric` 是 sensing 泄漏，不随干扰变化；干扰只抬 floor）。

---

## 3. 结果 A：jammer 通道与耦合验证

| 检查 | 结果 |
|---|---|
| ch1 NCO 频偏（回读） | `6490091340.000002 Hz` = **+491340.0 Hz** 精确 |
| 双通道 TX 流 `tx_channels=[0,1]` | 建立/发送正常 |
| 开启第二通道是否扰动 sensing | 不扰动（metric 0.00630 → 0.00618 @ jam-scale 0.001） |
| jammer 是否到达 RX | 到达：`--jam-gain-tx 60 --jam-scale 0.8` 时 burst 功率 **+16.2 dB** |
| 定时 | 320/320、520/520 ok，`late=0`，retune 15/25 次 |

---

## 4. 结果 B：code × CFO 矩阵（决定性好坏对照）

jammer 保持强干扰（gain 60 / scale 0.8），只改 code 与 Δf：

| 工况 | jam code | Δf | floor(40:116) | peak(13:24) | peak/floor |
|---|---|---|---|---|---|
| jam off（基线） | — | — | **0.00018** | 0.00630 | 34.2 |
| jam on | 10 | **0** | **0.00796** | 0.02069 | **2.6** |
| jam on | 10 | **491.34 kHz** | **0.00020** | 0.00613 | 30.8 |
| jam on | 9 | 0 | 0.00124 | 0.12210 | 98.5 |
| jam on | 9 | **491.34 kHz** | **0.00020** | 0.00613 | 30.7 |

- Δf=0：干扰**把 CIR floor 抬高 44×（+16.4 dB）**，peak/floor 从 34 塌到 2.6 —— 通信
  干扰确实会毁掉 sensing CIR。
- Δf=491.34 kHz：floor **回到 jam-off 本底**（0.00020 vs 0.00018），peak/floor 恢复 30.8。
  **自抵消在真机上成立**，且与 code 无关（code 9/10 都回到本底）。
- code 9 且 Δf=0 时 floor 只有 0.00124、但 peak 涨到 0.12210：同码干扰**相干叠加成一条
  真实信道**（表现为一个强 tap），不是噪声；换句话，同码时 CIR 被"注入假目标"，比
  异码更危险。

---

## 5. 结果 C：Δf 扫描曲线（16 点，320/320 ok）

| Δf (kHz) | floor | peak/floor |
|---|---|---|
| **0** | **0.00796** | **2.6** |
| 100 | 0.00030 | 20.4 |
| 200 | 0.00022 | 27.1 |
| 300 | 0.00021 | 29.8 |
| 400 | 0.00022 | 28.0 |
| 450 | 0.00019 | 32.5 |
| **491.34** | **0.00019** | **32.8** |
| 500 | 0.00019 | 32.9 |
| 550 | 0.00020 | 31.5 |
| 600 | 0.00021 | 28.4 |
| 700 | 0.00020 | 30.5 |
| 800 | 0.00021 | 29.4 |
| 900 | 0.00034 | 17.9 |
| **982.68** | **0.00686** | **2.3** |

**干扰峰值精确落在 Δf = 0 与 982.68 kHz（= f_work/SPS，重复梳齿）**；两者之间整段被压制
到本底。491.34 kHz（两峰中点）确实是零点，但并非特殊 —— 只要不落在梳齿峰附近，都被抵消。

---

## 6. 结果 D：主瓣细扫（26 点，520/520 ok）—— Dirichlet 结构确认

基线 floor（jam off）= 0.00018：

| Δf（相对 0） | floor | 相对基线 | 读法 |
|---|---|---|---|
| **0 Hz** | 0.00799 | **+16.5 dB** | 梳齿峰 |
| 2 kHz | 0.00724 | +16.0 | 峰内 |
| 4 kHz | 0.00537 | +14.7 | 峰内 |
| 6 kHz | 0.00294 | +12.1 | 峰裙 |
| **8.62 kHz** | **0.00019** | **+0.3 dB** | **第一零点** |
| 11 kHz | 0.00156 | +9.4 | 第一旁瓣 |
| 14 kHz | 0.00148 | +9.2 | 旁瓣 |
| 18 kHz | 0.00041 | +3.6 | |
| 25 kHz | 0.00038 | +3.3 | |
| 35 kHz | 0.00023 | +1.0 | |
| ≥50 kHz | ~0.00033 | +2~3 dB | 旁瓣本底 |

| Δf（相对 982.68 kHz） | floor | 相对基线 |
|---|---|---|
| **982.68 kHz** | 0.00688 | **+15.8 dB** |
| 986.68 kHz | 0.00464 | +14.1 |
| 988.68 kHz | 0.00255 | +11.5 |
| **991.30 kHz** | **0.00020** | **+0.5 dB** ← 第二零点 |
| 993.68 kHz | 0.00133 | +8.7（旁瓣） |
| 1000.68 kHz | 0.00038 | +3.2 |

**理论预测**：零点在 `x = k/N`（N 为相干平均次数），即间距 `f_work/(SPS·N) = 998.4e6/(1016·114) = 8619.98 Hz`。
**实测零点：0 峰旁 8620 Hz、982.68 kHz 峰旁 991300−982680 = 8620 Hz。逐点吻合。**

---

## 7. 结论

1. **X410 双 TX 干扰通道完全可用**：第二路 HRP 信号、精确可控 CFO（+491340.0 Hz 回读）、
   与 sensing 同钟、逐脉冲 sample 对齐。
2. **通信干扰确实会严重破坏 sensing CIR**：Δf=0 时 floor 抬 44×，peak/floor 34→2.6。
3. **"preamble 自抵消"在真机成立**：Δf 落在零点时 floor 回到本底，与 preamble code 无关。
4. **零点是"梳状"而不是单点**：峰在 `Δf = m·982.68 kHz`（宽度仅 ±~7 kHz），零点在峰旁
   `8.62 kHz` 及其整数倍。**491.34 kHz 只是两峰中点的一个零点。**
5. **对"商用设备抵消不理想"的解释需要修正**：由于"坏区"（梳齿峰）只有 ±7 kHz 宽、
   占空比 ~1.4%，随机/漂移的 CFO **大概率**落在被压制区。所以商用场景下残余干扰更可能来自
   **周期性被破坏**（商用设备符号时钟 ≠ X410，118 次平均里采样相位滑移）、脉冲形状失配、
   非 preamble 段与 RX 非线性 —— 而**不是** CFO 恰好落在梳齿峰上，也难归因于单纯 TX EVM。
6. 同码干扰（code 9）会**相干注入假目标**（peak 0.0063→0.1221），比异码干扰更危险。

---

## 8. 已知限制 / 待办

| 项 | 说明 |
|---|---|
| sensing 电平偏低 | 本次 `metric_mean≈0.0062`，低于今日下午同配置的 `0.0705`；**未改动的原版 base app（cpp-pdu 与 python）同样复现**，故与本 app 无关。怀疑 ch1 天线紧贴 RX 天线造成互耦失谐/散射。**需把 ch1 天线挪远 20–30 cm 后复测**（jammer 有 +21 dB 余量，可承受耦合下降）。 |
| python 后端速率 | 200 Hz 会 `late`/TX underflow；**≤50 Hz 稳定**。 |
| 零点深度 | 受本底限制只能证到"回到本底"（≥16 dB）；更深需要更强 jammer 或更低本底。 |
| 时钟不匹配未建模 | 本次 jammer 与 sensing 同钟，周期性完美。要复现商用设备，需给 jammer 一个**不同的符号时钟**（例如按非 1016 的晶格重采样）再测。 |
| 非 preamble 段 | 本次 jammer 用 `preamble` 纯周期波形；`--jam-waveform packet` 尚未系统测。 |

---

## 9. 复现命令

```bash
# 单点强干扰（Δf=0 与 491.34 kHz 对照）
sudo -E python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py \
  --use-dpdk --mgmt-addr 192.168.20.133 --args addr=192.168.10.2 \
  --jam-enable --jam-gain-tx 60 --jam-scale 0.8 --jam-code-index 10 \
  --jam-preamble-length 128 --jam-waveform preamble --jam-channel 1 \
  --jam-freq-offset 491340 \
  --preamble-length 128 --gain-tx 50 --gain-rx 60 --pri-s 0.02 --pulses 20 \
  --no-udp --output /tmp/x410_jam

# Δf 扫描（app 内建 dwell 扫描）
sudo -E python3 gr-uwb/apps/x410_cg400_hrp_echo_cir_jam.py \
  ... --jam-freq-offsets 0,2e3,4e3,6e3,8.62e3,11e3,18e3,50e3,491.34e3,982.68e3 \
  --jam-dwell 20 --pulses 0 ...
```

分析：`cir.jsonl`（pulse_id→status）+ `cir.cf32`（每帧 116 taps），按 `summary.json` 的
`jam_sweep` 把 pulse_id 映射回 Δf。
