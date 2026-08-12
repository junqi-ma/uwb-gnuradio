# QM35 离线解调 / 调度锁 / 重采样性能 — 问题总结（供 Codex 分析）

> 日期：2026-08-12  
> 范围：`gr-uwb` 周期旁路截取 → PDU 65/48 → 实时解调；真实 QM35 SC16 捕获  
> 目的：汇总**当前事实、数据、瓶颈归因与开放问题**，供 Codex 做根因分析与方案设计。  
> **本文是问题快照，不是实现任务清单。** 请基于下文证据分析，勿用旧的「连续流升采样必须实时」假设覆盖 PDU 路径结论。

---

## 1. 系统上下文

### 1.1 目标链路

```
X410 @737.28 MS/s SC16
  → (理想) RFNoC 65/48 → 998.4 MS/s
  → UwbScheduledExtractor（已知 t0/T 截雷达窗）
  → [当前 offline] host PDU 65/48 → 998.4
  → UwbRealtimeDemodulator（code-9, 64 SYNC, 4z2）
```

- 标称雷达周期 **T = 5000 µs**（@737.28 → `period_samples = 3686400`）。
- 种子：`t0 = 3543552`（0-based sample @737.28）。
- 窗几何（offline 默认）：`pre=30000, capture=160000, post=10000` → **200000 samp/窗**。
- 解调域：998.4 MS/s；模板 `testdata/reference_preamble.bin`。

### 1.2 主入口与模块

| 角色 | 路径 |
|------|------|
| Offline 脚本 | `testdata/offline_qm35_gr_demod.py` |
| 截取 | `UwbScheduledExtractor` + `core::ScheduledWindowCore` |
| 调度锁 | `core::ScheduleLockTracker`（header-only in `uwb_scheduled_extractor_core.h`） |
| PDU 重采样 | `UwbPduRationalResamplerCcf65_48` → `core::RationalResampler65_48Core` |
| 解调 | `UwbRealtimeDemodulator` + `uwb_demod_core.h` |
| 捕获 | `/mnt/f/UWB基带数据/qm35_6489p6MHz_737p28Msps_0p5s_sc16_20260811_01.dat`（0.5 s，~1.4 GiB SC16） |

### 1.3 相关历史文档（勿重复踩坑）

- `docs/performance/分析与下阶段建议_GNURadio软件升采样_65_48.md`  
  **连续流** host 65/48 最优约 **0.29× RT**，已判定不可生产；主架构改为 **原生率截窗 + PDU 升采样**。
- `docs/performance/规格_固定65_48重采样core契约.md` — core 契约 / golden。
- `docs/phase1/开发总结_QM35825周期旁路雷达截取.md` — Extractor 设计。

---

## 2. 已完成的功能改动（本轮）

### 2.1 调度锁：学固定偏差后冻结（learn-then-freeze）

**动机**：标称 T=5000 µs 与真实间隔差一个近似固定的 δ（SFO 量级 ppm）；t0 有固定 bias b。  
不应用 FCS 快环 / 连续 IIR PLL（曾导致中后段 timing 变差）。

**状态机**（`ScheduleLockTracker`）：

| 状态 | 含义 |
|------|------|
| Searching (0) | 无观测 |
| Learning (1) | 缓冲 `(k, detected_start)`，线性 LS 拟合 `t0`、`T` |
| Hold (2) | 冻结；仅连续大残差才 re-learn |

**模型**：

```
predicted(k) = (t0_seed + b) + k * (T_nom + δ)
```

**观测源（优先级）**：

1. Demod **SYNC/preamble timing**（`timing.ok` → `schedule_feedback` → `lock_obs`）  
2. 可选 radar verification 强峰  
3. **FCS 不作快传感器**（反馈里可带 `fcs_pass` 诊断，但不门控 observe）

**收敛**：≥3 不同 slot + 残差 ≤128 samp + `|T−T_nom|/T_nom` ≤ 2%。  
**Hold 维护**：残差 >512 samp 连续 5 次才回 Learning。  
**应用**：`update_locked_params` 优先绝对学到的 t0；不安全则 `next_k` continuity。

**API**：`locked_delta_period_samples()` / `locked_bias_t0_samples()`；Python 已绑定。  
**默认**：lock **OFF**（`set_schedule_lock_enabled(true)` 才开）。

### 2.2 Offline 脚本

- 默认：**整段 SC16→CF32 预载 RAM**，经 `/dev/shm` `file_source` 喂入（避免跑时读 `/mnt/f`）。  
- `--stream-from-disk`：旧 file_source 路径。  
- `--workers=N`、`--no-lock`、`--max-slots=N`。  
- 打印：`load_s` / `wall_s`(process) / 解调分阶段 µs / lock 状态。  
- 等待逻辑：idle-stop（注意：曾用 **3 s idle** 导致 process wall **虚高**）。

### 2.3 宽搜索 timing

`Qm35825Profile`：`timing_search_margin=40960`，`timing_coarse_stride=16`（覆盖 ppm 漂移 + seed 偏差）。

---

## 3. 正确性 / FCS 实测（真实 QM35）

捕获：`qm35_…_0p5s_…01.dat`，`t0=3543552`，T=5 ms，**最多 99 个完整窗**。

### 3.1 短跑 A/B（20 slots，早期）

| | lock ON | lock OFF |
|--|--------:|---------:|
| results | 19 | 19 |
| fcs_pass | **17** | **17** |
| lock | Hold, updates=1, δ≈+51 samp | 无 |

**判决：FCS 打平。** Hold 收敛，δ 量级合理（数十 samp）。

### 3.2 全长 A/B（99 slots）

| | lock ON | lock OFF |
|--|--------:|---------:|
| results | 99 | 99 |
| **fcs_pass** | **71 (71.7%)** | **71 (71.7%)** |
| success / payload_failed / timing_failed | 71 / 12 / 16 | 71 / 14 / 13 |
| wall（当时 disk-stream） | ~14 s | ~14 s |

Lock-ON 学到示例：`δ≈+37.5 samp`，`bias_t0≈−353`，Hold，updates=2。

**det−pred（success）**：

| | ON | OFF |
|--|----|-----|
| med | ~−316 | ~+1718 |
| max | ~+34 | ~+3996 |

OFF 在固定 T_nom 下 residual 随 SFO **正向漂移**；ON freeze 后偏晚但更稳。  
**全长仍 FCS 打平** — 0.5 s 上宽搜索已够；lock 的价值更可能在更长流 / 更窄 margin。

### 3.3 失败形态

- `timing_failed`：timing 阶段即退出（~1.8–2 ms），peaks=0。  
- `payload_failed`：SYNC/SFD 往往还在，payload/CFO 异常。  
- 少数 `fcs_failed`。  
- 成功包 PHR 长度几乎全是 **54 B**（非 golden 127 B 路径）。

---

## 4. 性能事实（重点）

### 4.1 端到端 offline（RAM preload + 4 workers + no-lock）

| 阶段 | 时间 | 备注 |
|------|------|------|
| Load SC16→RAM（+写 /dev/shm） | **~7.2 s** | `/mnt/f` 读盘 |
| Process（extract+resamp+demod） | **~6 s 计时** | 含 idle-stop 空等，**真实干活约 3–4 s** |
| 吞吐 | **~16–17 slot/s** | 需 200 slot/s 才满 5 ms 实时 |
| FCS | ~70/98 | 与 worker 数无关 |

Worker 扩展（全链路 wall）：

| workers | process wall | slot/s | demod stage med |
|--------:|-------------:|-------:|----------------:|
| 1 | ~14.4 s (disk era) | ~6.9 | ~7.2 ms |
| 2 | ~5.7 s (RAM) | ~17 | ~7.4 ms |
| 4 | ~5.9 s (RAM) | ~17 | ~7.5 ms |

**4 worker 几乎不加速 end-to-end** → 供给/前端限速，不是 demod 算力。

### 4.2 解调单包分阶段（success，中位）

| 阶段 | 中位 µs | 约占 |
|------|--------:|-----:|
| **SFD** | **~3860** | **~51%** |
| **Timing（宽搜）** | **~2160** | **~29%** |
| **CIR** | **~1030** | **~14%** |
| CFO | ~330 | ~4% |
| payload / PHR / ns_sfd | &lt;100 | &lt;2% |
| **TOTAL** | **~7500** | 100% |

CIR 内：`cir_softfir` ~80%，estimate/post 次要。  
`queue_delay` 中位 **~30 µs** → worker **很闲**，包来得慢。

相对 5 ms 周期：单包 demod 中位 **~7.5 ms ≈ 1.5× 周期**；  
**2–4 worker 的 CPU 容量理论可 >200 pkt/s**，但 offline 喂包远低于此。

### 4.3 重采样：core 正常，归因曾被误判

#### Core 微基准（N=200000，与 offline 窗同长，本机）

| taps | per-win | 输入 MS/s |
|------|--------:|----------:|
| quality_minorder (T=2707，offline 默认) | **2.15 ms** | **~93** |
| realtime_minorder (T=1319) | **1.24 ms** | **~161** |

99 窗 core-only ≈ **0.21 s**。

#### PDU 块（消息路径，同 N）

| 路径 | per-win | 输入 MS/s |
|------|--------:|----------:|
| Core | 2.15 ms | 93 |
| **PDU `handle_packet`** | **3.33 ms** | **~60** |

PDU 额外 ~1.2 ms：`reset` oneshot、输出 `pmt::init_c32vector(~271k)`、meta。  
`pmt.init_c32vector(200k)` 单独可到 **~10 ms** 量级（大块拷贝税）。

#### 与「全流连续升采样」文档结论的关系

- 连续 737 MS/s host FIR：**从未实时**（最优 ~0.29× RT）。  
- PDU 路径按占空比设计：窗 **200k / 3.686M ≈ 5.4%**；  
  quality_minorder core **2.15 ms &lt; 5 ms** → **窗级实时预算够用**。  
- **Offline 墙钟慢 ≠ PDU FIR 坏了。**

#### Offline process 时间构成（量级模型）

| 成分 | 估计 |
|------|------|
| 扫完全流 ~3.66e8 samp @~174 MS/s（Extractor） | **~2.1 s** |
| PDU resampler ×99 | **~0.33 s** |
| Demod CPU/4 | **~0.18 s** 下界 |
| idle-stop 空等 | **可达 ~2–3 s**（测量 artifact） |
| 实测 process wall | ~6 s（含水） |

**结论**：慢的主因是 **全捕获时间线扫描 + PMT 大包 + 测量空等**，不是 core 在 GR 里数量级退化。

---

## 5. 实时性判断（当前）

| 能力 | 判断 | 依据 |
|------|------|------|
| Offline 全链路实时解包 | **否** | RTF≪1（0.5 s 数据要数秒～十几秒） |
| 仅 demod 算力 @200 pkt/s | **2–4w  theoretically OK** | med 7.5 ms → 4w ~550 pkt/s |
| 仅 PDU 65/48 @每 5 ms 一窗 | **算力上 OK** | 2–3 ms/窗 &lt; 5 ms |
| 连续流 host 65/48 | **否** | 文档 0.29× RT |
| 截窗 SC16 落盘 | **可能** | 满 slot ~160 MB/s SC16；未在 X410 上最终验收 |

生产意图仍是：**RFNoC 升采样 + 截窗**，host 避免连续 FIR。

---

## 6. 当前问题清单（请 Codex 聚焦）

### P1 — 性能归因与测量可信度

1. Offline `wall_s` 含 **idle-stop**，高估 process；需干净的「首包→末包」计时。  
2. Process 中 **Extractor 全流扫描 vs PDU 重采样 vs PMT 拷贝** 未在同一 run 用同一时钟拆开；缺 `fir_us` / `pmt_us` / `extract_stream_us` 计数器。  
3. 4 worker 不加速的根因：确认是 **msg handler 串行 resampler** 还是 **extractor 供给** 还是两者。

### P2 — 重采样路径工程化

4. PDU 路径相对 core 的 1.5×：能否避免双份大 `c32vector` 拷贝（zero-copy / 池化）？  
5. Offline 默认 **quality_minorder** 是否过重？`realtime_minorder` 对 QM35 FCS 是否可接受？  
6. 窗长 200k（pre+cap+post）是否可在保证 FIR 群时延 guard 下缩短，直接降 MAC？  
7. `EmitPolicy::CaptureOnly` 对 demod 种子/坐标是否仍正确？

### P3 — 调度锁与解调正确率

8. 全长 FCS ON=OFF=71/99：lock 在 0.5 s 无增益；何时有增益？更长 capture？收窄 `timing_search_margin`？  
9. Hold 后 det−pred med≈−300：SYNC 观测是否系统偏到 preamble 中部？是否应锁 SFD 时刻或 first-peak？  
10. `timing_failed` 16/99：与 residual / 干扰 / 空 slot 关系？是否应用能量门跳过空窗？  
11. payload_failed 与错误 CFO（负几千 Hz）是否同一根因？

### P4 — 架构

12. Offline 是否应 **跳过 host 65/48**（若已有 998.4 域数据或 RFNoC）以便隔离 demod 性能？  
13. 实时存储路径：`UwbPacketWriter` 在 200 Hz × 0.8 MB 下的 drop/背压未测。

---

## 7. 关键代码锚点

```
gr-uwb/include/gnuradio/uwb/uwb_scheduled_extractor_core.h  # ScheduleLockTracker, ScheduledWindowCore
gr-uwb/lib/uwb_scheduled_extractor.cc                      # lock_obs, note_lock_observation
gr-uwb/lib/uwb_realtime_demodulator.cc                     # schedule_feedback on timing.ok
gr-uwb/lib/uwb_pdu_rational_resampler_ccf_65_48.cc         # oneshot process+flush, PMT out
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h  # AVX2 FIR core
gr-uwb/include/gnuradio/uwb/uwb_phy_profile.h              # timing_search_margin=40960
testdata/offline_qm35_gr_demod.py                          # RAM preload, workers, metrics
testdata/resampler_65_48/taps_quality_minorder.txt         # T=2707，offline 默认
```

---

## 8. 复现命令

```bash
# 环境
export PYTHONPATH=$PWD/gr-uwb/build/test_modules
export LD_LIBRARY_PATH=$PWD/gr-uwb/build/lib

# 全长 + RAM preload + 4w + 无锁
python3 testdata/offline_qm35_gr_demod.py --no-lock --workers=4

# 调度锁 ON
python3 testdata/offline_qm35_gr_demod.py --workers=4

# Core 窗级微基准（若本地有 /tmp/bench_core_resamp 或自建）
# quality_minorder @200k → ~2.1 ms/win, ~93 MS/s

# CTest（正确性）
cd gr-uwb/build && ctest --output-on-failure
```

会话内证据目录（可能已清理，数字以本文表格为准）：

- `/tmp/qm35_long_ab/` — 全长 lock ON/OFF  
- `/tmp/qm35_ram_bench/` — RAM preload worker 对比  
- `/tmp/qm35_stage_profile/` — 分阶段 breakdown  

---

## 9. 给 Codex 的分析请求（建议输出）

请基于本文，产出：

1. **瓶颈因果图**：load / extract stream / PDU FIR / PMT / demod stages / 测量 artifact 各自占比与依赖。  
2. **「core 正常但 offline 慢」** 的严格论证或反驳（用可复现实验设计）。  
3. **P1–P4 的优先级排序**与每个问题的：假设 → 最小验证实验 → 通过/失败判据。  
4. 若目标是「5 ms 周期实时解包+存窗」：host-only 与 RFNoC 两条路径的可行性边界。  
5. 调度锁：在何种观测模型下 δ/b 估计无偏；是否应改用 SFD 时刻。  

**非目标**：不要重开「连续 737 MS/s host FIR 冲实时」主线（文档已否决），除非新数据推翻。

---

## 10. 一句话现状

> **功能上**：QM35 0.5 s 离线约 **72% FCS**，learn-then-freeze 能收敛且与纯宽搜索 FCS 打平。  
> **性能上**：demod/SFD 单包 ~7.5 ms、PDU 重采样 ~3 ms/窗在周期预算内；offline 墙钟被 **读盘 + 全流扫描 + PMT + 计时空等** 主导，**不是 core 在 GR 里算坏了**。  
> **开放**：精确拆分计时、锁观测无偏、空窗/timing_failed、以及 RFNoC 路径落地。
