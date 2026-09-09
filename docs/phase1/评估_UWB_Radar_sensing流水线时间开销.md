# 评估：UWB Radar sensing 流水线时间开销与最大 packet 速率

> 日期：2026-09-09
> 分支：`feature/uwb-monostatic-radar`
> 主机：12th Gen Intel Core i7-12700（20 逻辑核，Hyper-V 全虚拟化）
> 工具：`gr-uwb/apps/benchmark_radar_pipeline.cc`
> 复现：`./gr-uwb/build/apps/benchmark_radar_pipeline 16 256`
> 状态：**本机软件实测**。不含 X410 UHD send/recv、late_drop、overflow。

---

## 1. 结论

生产 sensing 路径（RX 突发已在 host 上之后）是：

```text
native SC16 窗（79.5 µs）
  → PDU 65/48 或 65/32（scheduler 线程，handler 内 FIR）
  → UwbRadarCirEstimator（单 worker：SFD → SYNC refine → CIR）
  → UwbCirWriter（单 worker）
```

| 约束 | 本机结果 | 含义 |
|---|---|---|
| 射频占用（TX 空中时长） | 191.2 µs → **5175 pulse/s** | 64-SYNC 完整 packet，不是 host 上限 |
| Host 突发吞吐（256 帧、depth 8、0 drop） | **UC200 ≈ 680 pulse/s，CG400 ≈ 725 pulse/s** | CIR worker 瓶颈 |
| CIR 服务时间 | mean ≈ 1.26–1.34 ms，p50 ≈ 1.26 ms，p99 ≈ 3.3–4.5 ms | 单 worker 硬上限 ≈ `1/mean` |
| 默认 PRI = 5 ms（200 pulse/s） | mean 余量 ≈ 6.5×，p99 通常 < 5 ms | **现网目标安全** |
| 建议本机可持续上限 | **400 pulse/s 需排队吸收 p99；500+ 接近 mean 上限** | 见第 5 节 |

**推荐口径（本机、quality_minorder、单 CIR worker、无 UHD）：**

- **200 pulse/s（PRI=5 ms）**：已有 30 s 软件 soak，平均服务时间远低于 5 ms，作为生产默认。
- **≤350 pulse/s**：mean 仍有 ≥3× 余量；p99 偶发 3–5 ms，靠 estimator 队列（默认 64）吸收。
- **≈680–740 pulse/s**：短突发饱和吞吐，是 CIR mean 的倒数，不是可承诺的硬实时 PRI。
- **不得**把 5175 pulse/s 射频上限或 680 pulse/s 突发写成 X410 已验收。

UC200（737.28 + 65/48）与 CG400（491.52 + 65/32）的 **host 开销几乎相同**：CIR 始终在 998.4 MS/s 上跑，重采样比 CIR 便宜且与 CIR 流水重叠。

---

## 2. 计量对象与非目标

### 2.1 生产几何（与 `x410_uwb_radar_validate.py` 默认一致）

`pre_guard=2 µs`，`range=15 m` 往返，`tail=4.1 µs`，64-SYNC + 4z2 SFD。

| | UC200 737.28 MS/s | CG400 491.52 MS/s |
|---|---|---|
| TX 样点 / 空中时长 | 140982 / **191.2 µs** | 93988 / **191.2 µs** |
| pre / SYNC / SFD / range / tail | 1475 / 48018 / 6003 / 74 / 3023 | 983 / 32012 / 4002 / 50 / 2015 |
| RX 窗 | 58593 样点 / **79.5 µs** | 39062 样点 / **79.5 µs** |
| 占用 `pre + max(TX, RX−pre)` | **193.2 µs** | **193.2 µs** |

RX 从 `t_tx − 2 µs` 开始，与 TX 重叠；下一脉冲不能早于 TX 结束，所以射频下限由 **完整 TX packet 空中时长** 决定，不是 79.5 µs 的 CIR 分析窗。

工作域等价窗（CIR 输入）`pre+SYNC+SFD+range+tail = 79342` 样点 @998.4。本评估用 TX golden 嵌入该窗，SFD 落在预测点（`sfd=67021`，`origin=1997`，`peak_tap=18`）。

### 2.2 并行关系

重采样在 GNU Radio **消息 handler**（scheduler 线程）里一次性 `process+flush`；CIR 在 **独立 worker**；写盘在 **独立 writer**。稳态吞吐是

```text
pps ≈ 1 / max(T_resample_handler, T_cir_service, T_write)
```

不是三段相加。单脉冲空流水线延迟才是三段之和。

### 2.3 明确没测

- EchoTimer / UHD `send`/`recv`、fragment、command 提前量
- 硬件 soak、late_drop、overflow
- 连续 host 流式 65/48 或 65/32（已关闭）
- 多 CIR worker（当前块保证顺序，禁止无重排序的并行）

---

## 3. 分段时间（本机，quality_minorder，AVX2/FMA kernel）

下列为 `rounds=16`、`pdus=256` 一次完整跑；p99 在 Hyper-V 上会跳，**不要把单次 p99 当硬上界**。更稳的是 p50 / mean。

### 3.1 CIR core（无 scheduler，生产工作域窗 79342 点）

| 阶段 | mean | p50 | p95 | p99 | 占比（相对 radar_cir_one mean） |
|---|---|---|---|---|---|
| SFD 搜索 ±64（模板 8128） | 748 µs | 711 µs | 900 µs | 967 µs | ~58% |
| SYNC refine ±8 | 13 µs | 13 µs | 13 µs | 13 µs | ~1% |
| `estimateCir` 54 repetition | 260 µs | 252 µs | 305 µs | 305 µs | ~20% |
| `radar_cir_one` 合计 | 1288 µs | 1262 µs | 1327 µs | 1793 µs | 100% |

SFD 是算法热点：约 129 个起点 × 8128 点全速率相关。CIR 本身只对 54 个 SYNC 做平均再 116 tap 匹配，比 SFD 便宜。

### 3.2 重采样 core（生产 RX 窗）

| | mean | p50 | p99 |
|---|---|---|---|
| 65/48 quality（2707 taps，N=58593） | 683 µs | 673 µs | 785 µs |
| 65/32 quality（2707 taps，N=39062） | 698 µs | 670 µs | 930 µs |
| 65/48 realtime（1319 taps） | 390 µs | 386 µs | 417 µs |
| 65/32 realtime（1055 taps） | 367 µs | 356 µs | 460 µs |
| 65/48 quality **整包 TX+tail**（QA loopback 窗） | 2184 µs | 2167 µs | 2374 µs |
| SC16→FC32 UC200 / CG400 | 20 / 13 µs | 19 / 13 µs | 31 / 13 µs |

生产必须只升 **79.5 µs 分析窗**。若误把完整 TX（191 µs）送进 PDU 重采样，FIR 升到 ~2.1 ms，会和 CIR 抢预算。

### 3.3 PDU 块（含 PMT / 发布）

Handler 墙钟略高于 FIR，多出来的是 metadata + `c32vector` 发布。

| 块 | 墙钟 mean / p50 | 内部 FIR | SC16 转换 | publish |
|---|---|---|---|---|
| PDU 65/48 SC16 quality | 1111 / 1097 µs | 712 µs | 17 µs | 94 µs |
| PDU 65/32 SC16 quality | 1536 / 1451 µs* | 732 µs | 12 µs | 432 µs* |
| PDU 65/48 FC32 realtime | 728 / 626 µs | 417 µs | 0 | 78 µs |
| CirEstimator 串行（含入队） | 1610 / 1461 µs | service mean **1312 µs**（p95 1472，p99 4543） | — | — |
| CirWriter 116 tap + JSONL | 303 / 294 µs | — | — | — |
| LoopbackEcho 整包（仅 QA） | 865 / 857 µs | — | — | — |

\* 65/32 SC16 这一轮 publish/p99 偏高（VM 噪声）；同日 64 帧跑 FIR mean 710 µs、handler 864 µs，与 65/48 同级。

Estimator **service**（`radar_cir_one` 墙钟）mean 1.31 ms，与 Step 7–8 QA（1.26–1.39 ms）一致。

---

## 4. 饱和 e2e（生产窗，SC16 → 重采样 → CIR → 写盘）

保持 8 个 in-flight，256 帧，**全程 0 drop**：

| 路径 | 墙钟 | 吞吐 | estimator service mean | 队列水位 |
|---|---|---|---|---|
| UC200 65/48 quality | 0.376 s | **681 pulse/s** | 1325 µs | 7 / 64 |
| CG400 65/32 quality | 0.353 s | **725 pulse/s** | 1257 µs | 7 / 64 |
| UC200 65/48 realtime | 0.347 s | 738 pulse/s | 1238 µs | 7 / 64 |
| CG400 65/32 realtime | 0.355 s | 720 pulse/s | 1264 µs | 7 / 64 |

Realtime taps 几乎不提高 e2e：CIR ~1.26 ms 已经压过 FIR ~0.4–0.7 ms。换短滤波器只能降低 scheduler 占用，不能把单 worker CIR 再加快。

64 帧短突发曾到 705–725 pulse/s，与 256 帧同量级，说明不是只靠 cache 热身虚高。

`1 / 1.31 ms ≈ 763 pulse/s`，实测 680–725 是 handler 发布、队列与写盘的合理折扣。

---

## 5. 最大 packet 速率怎么定

### 5.1 三层上限

```text
射频     :  1 / 193.2 µs  ≈ 5175 pulse/s     （完整 TX 空中时长）
host mean:  1 / 1.31 ms   ≈  760 pulse/s     （CIR worker）
host 实测:                  680–725 pulse/s  （256 帧饱和 e2e）
host p99 :  1 / 4.5 ms    ≈  220 pulse/s     （无排队、硬实时 p99 < PRI）
```

默认 200 pulse/s 落在 p99 安全区附近，并留出 VM/调度抖动。

### 5.2 建议工作点

| PRI | pulse/s | 相对 CIR mean 余量 | 判定 |
|---|---|---|---|
| 5.0 ms | **200** | 6.5× | 生产默认。软件 30 s soak 已过。 |
| 3.3 ms | 300 | 4.3× | 本机软件可跑；p99 偶发贴近 PRI，靠队列。 |
| 2.5 ms | 400 | 3.3× | mean 够；p99 3–5 ms **会超过 PRI**，队列水位会抬头，不能当硬实时。 |
| 1.47 ms | 680 | 1.0× | 突发饱和，无余量。任何抖动即堆积。 |
| 0.19 ms | 5175 | — | 仅射频几何；host 差一个数量级。 |

**本机建议承诺：软件可持续 200 pulse/s；试验上限 300–350 pulse/s；不要把 ≥400 写成已支持，除非加 CIR worker 重排序或把 SFD 搜窗/模板算量降下来，并做对应 soak。**

### 5.3 若要再提高速率，先动哪

1. **SFD 搜索**（~0.7 ms）：缩小 `sfd_search_margin`、降采样相关、或 SIMD 模板相关。这是唯一能明显抬 CIR worker 上限的算法点。
2. 多 CIR worker + 按 `pulse_id` 重排序（计划明确禁止无 QA 的并行）。
3. 不要指望 realtime taps 或 CG400 更短窗：e2e 已被 CIR 限死。
4. 不要把整包 TX 送进 PDU 重采样。

### 5.4 硬件还要扣的预算

EchoTimer 每脉冲还要：TX 3 个 fragment（140982 / 65536）、RX 1 个 fragment、timed command 提前量、SC16 PDU 组装。这些在 200 pulse/s 下每脉冲有 5 ms 墙钟，通常可被射频等待覆盖；在 400+ pulse/s 会与 handler 争 CPU。X410 soak 未做，**400 pulse/s 硬件能力未验证**。

---

## 6. 与既有 QA 的对照

| 来源 | 数字 | 本次 |
|---|---|---|
| Step 7–8 `service_time_200pps` | mean 1.26–1.39 ms，P99 2–4 ms | service mean 1.31 ms，P99 3.3–4.5 ms |
| Step 9 `e2e_soak_200pps_30s` | 200 PDU/s × 30 s，0 drop | 未重跑 30 s；256 帧饱和 0 drop @ ~700/s |
| 计划退出条件 | 200 pulse/s 下平均服务 ≪ 5 ms | 满足（1.3 ms vs 5 ms） |

---

## 7. 复现

```bash
cmake --build gr-uwb/build --target benchmark_radar_pipeline -j
./gr-uwb/build/apps/benchmark_radar_pipeline 16 256
```

参数：`rounds`（core 重复次数）、`pdus`（块与 e2e 帧数）。Taps 为 `testdata/resampler_65_{48,32}/taps_{quality,realtime}_minorder.txt`。TX golden 为 `testdata/uwb_radar/tx_{998p4,737p28,491p52}.cf32`。
