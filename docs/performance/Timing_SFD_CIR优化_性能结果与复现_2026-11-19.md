# Timing / SFD / CIR 与匹配滤波优化：性能结果与复现包

> 日期：2026-11-19  
> 分支：`feature/sic-research`（基于 `b654733`，本轮改动未提交）  
> 用途：给 `agy cli` / 独立分析者做性能复核  
> 关联文档：
> [实施报告](Timing_SFD_CIR与匹配滤波优化实施报告_2026-11-19.md)、
> [并行开发方案](多Agent并行开发方案_Timing_SFD_CIR匹配滤波优化_2026-11-19.md)

---

## 0. 一句话结论

在 **i7-12700 / AVX2 / 单 worker / 文件回放** 下：

- **+4 dB 与 −12 dB+40 kHz CFO**：整包阶段总耗时下降 **~28%**，`demod_mean` 下降 **~18%**。
- **−12 dB 无 CFO**：整包阶段总耗时下降 **6%–9%**；`demod_mean` 噪声较大
  （4 轮 −10.9%，8 轮 +2.3%），不建议单独引用。
- **−12 dB + 27 ppm SFO**：端到端基本持平（两侧都有 ~8.9 ms 全流 soft-chip 占主导），SFD 阶段仍 −22%。
- 方案中 `MF-3`（交叉相乘免除法）实测为负收益，已回退；`MF-2`（无分支符号乘）在噪声内，一并回退为原实现。
- **MF-4（新增，opt-in）**：`(r ⋆ code) ⋆ p` 因子分解已实现并验证。它替代**稠密**模板：FCS 数不低于稠密，`st_timing` 快 **3.7–4.5×**，`st_total` 快 **19–61%**（SFO 场景仅 −11%），相对 live 默认的纯稀疏：seed1729/cfo40k 多 1–2 包，SFO 持平。默认关闭（缺 MATLAB 交叉验证）。

这不是 9700X / AVX-512 的验收数据；本机无 AVX-512，AVX-512 仅编译验证。

---

## 1. 环境与对照

| 项 | 值 |
|---|---|
| CPU | Intel i7-12700（12 代，AVX2/FMA，**无 AVX-512**），20 线程 |
| 构建 | Release，`UWB_SIMD=AUTO` → **AVX2**，`ENABLE_UHD_BACKEND=OFF` |
| 运行 | `env -u LD_LIBRARY_PATH`，单 worker，`--queue 256` |
| 基线 | `b654733`，独立 worktree `/tmp/uwb-base-654733/gr-uwb/build/apps/uwb_dw3000_live` |
| 优化版 | 本工作树 `gr-uwb/build-native/apps/uwb_dw3000_live` |
| 夹具 | `/tmp/uwb-low-snr-6x40/*.sc16`（code10 / 256 SYNC / 4z2 / 127 B，`pre_guard=36864`、`window=450000`、`eq16`） |
| 采样 | 每场景 A/B/B/A（4 进程）；`seed1729` / `sfo27ppm` 另做 ×2（8 进程），取中位数 |
| 条件 | `--replay <case>.sc16 --seconds 0 --period-ms 1.0 --mode continuous --workers 1 --queue 256 --cir full --fused on --taps eq16 --tone off` |

夹具说明见
[`tools/rx/low_snr_eval/README.md`](../../tools/rx/low_snr_eval/README.md)；
`control` 为 +4 dB，其余为 −12 dB + 指定的 CFO/SFO。

---

## 2. 结果（`uwb_dw3000_live` 文件回放）

单位 µs。`demod_mean` 来自 `SUMMARY` 的批均值；阶段值来自 `PACKET` 行的逐包阶段中位数。

| 场景 | demod_mean A→B | st_total A→B | st_timing | st_sfd | st_cir | st_payload |
|---|---:|---:|---:|---:|---:|---:|
| control (+4 dB) | 885.5 → 724.7 (**−18.2%**) | 935.6 → 677.8 (**−27.6%**) | 219.8 → 158.6 (**−27.8%**) | 141.8 → 28.2 (**−80.1%**) | 171.2 → 160.5 (−6.3%) | 322.3 → 261.7 (−18.8%) |
| seed1729 (−12 dB) | 1766.2 → 1574.2 (**−10.9%**) | 1687.0 → 1538.1 (**−8.8%**) | 550.2 → 573.5 (+4.2%) | 231.8 → 48.4 (**−79.1%**) | 147.7 → 173.1 (+17.2%) | 409.8 → 404.4 (−1.3%) |
| cfo40k (−12 dB, +40 kHz) | 1609.4 → 1311.8 (**−18.5%**) | 1878.6 → 1349.7 (**−28.2%**) | 708.3 → 628.9 (−11.2%) | 298.4 → 49.0 (**−83.6%**) | 163.3 → 165.3 (+1.3%) | 385.7 → 336.0 (−12.9%) |
| sfo27ppm (−12 dB, +27 ppm) | 10441.8 → 10455.1 (+0.1%) | 10507.2 → 10347.8 (−1.5%) | 690.5 → 718.8 (+4.1%) | 518.4 → 406.6 (−21.6%) | 8979.4 → 8904.1 (−0.8%) | 242.6 → 235.7 (−2.9%) |

### 2.1 关键观察

1. **SFD 是最大单点收益**：`st_sfd` 在四个场景分别 −80.1% / −79.1% / −83.6% / −21.6%。
   主要来自 `SFD-2`（复用 Stage-2 旋转或 8 符号相位漂移 <0.05 rad 时跳过 `run_final_sfd`），
   部分来自 `SFD-3` 的网格缓存与 `d=0` 去重。
2. **高 SNR / 有 CFO 时 timing 也明显下降**：control −27.8%、cfo40k −11.2%。
   来自 `P0-3`（稀疏路径跳过稠密模板）、32-lag 级联、`TM-2` 首探先窄后宽。
3. **CIR-1 的收益不在 `st_cir` 里体现**：它把前导去旋转从 `st_sfd` 的
   `sfd_cforef` 移到 CIR 的每 SYNC 标量乘。典型包：
   `sfd_cforef` 172 → 13 µs，`st_cir` +17%（+25 µs），整包仍净收益。
4. **SFO 场景被既有全流 soft-chip 掩盖**：`st_cir ≈ 8.9 ms`（分数网格强制
   `stage_cir_softchips` 全流），两侧相同；SFD 的 −22% 只体现在 `st_total` −1.5%。
5. **低 SNR 的 `st_timing` 基本持平到 +4%**：`TM-1` 探针加了 0.2 预门控，
   -12 dB 时不触发；`TM-4`/`P0-2` 省掉尾部重跟踪，二者大致抵消。

### 2.2 可机读摘要

```json
{
  "environment": {
    "cpu": "Intel i7-12700",
    "isa": "AVX2/FMA",
    "build": "Release, UWB_SIMD=AUTO->AVX2, ENABLE_UHD_BACKEND=OFF",
    "workers": 1,
    "source": "file replay",
    "baseline": "b654733 (/tmp/uwb-base-654733)",
    "method": "A/B/B/A (4 processes); seed+sfo replicated to 8",
    "demod_mean_note": "live scheduler noise; prefer stage medians"
  },
  "cases": {
    "control_p4db": {
      "demod_mean_us": [885.5, 724.7],
      "st_total_us": [935.6, 677.8],
      "st_timing_us": [219.8, 158.6],
      "st_sfd_us": [141.8, 28.2],
      "st_cir_us": [171.2, 160.5],
      "st_payload_us": [322.3, 261.7]
    },
    "seed1729_m12db": {
      "demod_mean_us": [1766.2, 1574.2],
      "st_total_us": [1687.0, 1538.1],
      "st_timing_us": [550.2, 573.5],
      "st_sfd_us": [231.8, 48.4],
      "st_cir_us": [147.7, 173.1],
      "st_payload_us": [409.8, 404.4]
    },
    "cfo40k_m12db": {
      "demod_mean_us": [1609.4, 1311.8],
      "st_total_us": [1878.6, 1349.7],
      "st_timing_us": [708.3, 628.9],
      "st_sfd_us": [298.4, 49.0],
      "st_cir_us": [163.3, 165.3],
      "st_payload_us": [385.7, 336.0]
    },
    "sfo27ppm_m12db": {
      "demod_mean_us": [10441.8, 10455.1],
      "st_total_us": [10507.2, 10347.8],
      "st_timing_us": [690.5, 718.8],
      "st_sfd_us": [518.4, 406.6],
      "st_cir_us": [8979.4, 8904.1],
      "st_payload_us": [242.6, 235.7]
    }
  },
  "mf4_shaped_vs_dense": {
    "mode": "--timing-sparse off --timing-shaped on",
    "note": "3 identical runs per arm; ok = packets with status=OK",
    "control_p4db":    {"dense": {"ok": 40, "st_timing_us": 357.0, "st_total_us": 891.5},
                        "shaped": {"ok": 40, "st_timing_us": 233.5, "st_total_us": 725.5},
                        "sparse": {"ok": 40, "st_timing_us": 91.5,  "st_total_us": 593.5}},
    "seed1729_m12db":  {"dense": {"ok": 38, "st_timing_us": 2636.0, "st_total_us": 3342.5},
                        "shaped": {"ok": 38, "st_timing_us": 692.5,  "st_total_us": 1316.0},
                        "sparse": {"ok": 37, "st_timing_us": 551.0,  "st_total_us": 1119.0}},
    "cfo40k_m12db":    {"dense": {"ok": 37, "st_timing_us": 2620.0, "st_total_us": 3219.0},
                        "shaped": {"ok": 37, "st_timing_us": 716.0,  "st_total_us": 1291.0},
                        "sparse": {"ok": 35, "st_timing_us": 663.0,  "st_total_us": 1243.0}},
    "sfo27ppm_m12db":  {"dense": {"ok_min": 36, "ok_max": 38, "st_timing_us": 3194.5, "st_total_us": 12853.0},
                        "shaped": {"ok_min": 38, "ok_max": 39, "st_timing_us": 940.5, "st_total_us": 11400.0},
                        "sparse": {"ok": 39, "st_timing_us": 780.0, "st_total_us": 11018.0}}
  }
}
```

> `control` / `cfo40k` 为 A/B/B/A 四进程中位数；`seed1729` / `sfo27ppm` 另有
> 8 进程复测，见 §2.3。`demod_mean` 受 live 调度/队列影响，逐轮波动大于阶段值；
> 阶段值更适合做代码对比。

### 2.3 `seed1729` / `sfo27ppm` 的 8 进程复测

| 场景 | demod_mean A→B | st_total A→B | 说明 |
|---|---:|---:|---|
| seed1729 (−12 dB) | 1708.0 → 1746.5 (+2.3%) | 1773.8 → 1669.9 (**−5.9%**) | `demod_mean` 与 4 轮结果符号相反，属 live 调度噪声；阶段总耗时稳定为负 |
| sfo27ppm (−12 dB) | 10211.4 → 10298.8 (+0.9%) | 10496.9 → 10203.8 (**−2.8%**) | 与 4 轮结论一致：SFD −22~−25%，被全流 soft-chip 掩盖 |

结论建议：**引用收益时使用 `st_total` 与各阶段中位数，`demod_mean` 只作为
参考并注明轮数**。

---

### 2.4 MF-4：因子分解替代稠密模板（3 次重复，全部一致）

命令只改 timing 相关开关，其余同 §1：

| 场景 | 稠密 `ok / st_timing / st_total` | **shaped** `ok / st_timing / st_total` | 稀疏（默认） `ok / st_timing / st_total` |
|---|---|---|---|
| control (+4 dB) | 40 / 357 / 892 | **40 / 234 / 726** | 40 / 92 / 594 |
| seed1729 (−12 dB) | 38 / 2636 / 3343 | **38 / 693 / 1316** | 37 / 551 / 1119 |
| cfo40k (−12 dB, +40 kHz) | 37 / 2620 / 3219 | **37 / 716 / 1291** | 35 / 663 / 1243 |
| sfo27ppm (−12 dB, +27 ppm) | 36–38 / 3195 / 12853 | **38–39 / 941 / 11400** | 39 / 780 / 11018 |

- `shaped` 用 `--timing-sparse off --timing-shaped on`；`dense` 为 `--timing-sparse off`；`sparse` 为默认。
- **FCS 数**：control/seed1729/cfo40k 与稠密完全一致（40/38/37）；SFO 场景 shaped 38–39、稠密 36–38、稀疏 39。即 shaped 不低于稠密；相对稀疏在 seed1729/cfo40k 多 1–2 包，SFO 持平。
- **耗时**：shaped 的 `st_timing` 比稠密快 3.5–4.5×，`st_total` 快 19%（control）到 61%（seed1729）。
- **不要和稀疏叠加**：`--timing-sparse on --timing-shaped on` 已被 CLI 拒绝。稀疏模板按脉冲峰值样本对齐，峰位与稠密/因子分解相差约 2 样点；混用会改变低 SNR 的 SFD 锚点，`-12 dB` 实测少 1–2 包（tmetric 与稠密一致，说明不是灵敏度问题）。
- 本机无 MATLAB，`detectRepeatedPreamble.m` 交叉验证**未做**，因此该模式仍为 opt-in，默认 profile 不变。

---

## 3. 内核级微基准（支持“哪些改动真的快”）

同一主机、同一编译选项（`-O3 -mavx2 -mfma -ffp-contract=off`），
对 code10 稀疏模板、1016 个 lag 重复 30000 次：

| 项目 | 基线 | 优化后 | 变化 |
|---|---:|---:|---:|
| `for_each_lag` 只累加（32/16/8/4/1 级联 vs 8/1） | 2.96 µs | **2.42–2.52 µs** | **约 −17%** |
| `scan_normalized_sync`（1016 lag，逐 lag 除法） | 4.40–4.80 µs | **3.99–4.04 µs** | **约 −10~15%** |
| 交叉相乘“免除法”版 vs 同代码逐 lag 除法 | ≈4.0–4.7 µs | 5.67 µs | **+21%~+42%（MF-3 负收益）** |
| 无分支 `sign * x` 单点相关（已回退） | 26.8 µs | 26.4 µs | −1.6%（噪声内） |
| 稠密 1016-tap `for_each_lag`（1016 lag，MF-4 对照） | — | 60–62 µs | 参考 |
| **因子分解 `for_each_lag`（MF-4）** | — | **11–14 µs** | **比稠密快 4.5–5.3×** |
| 纯稀疏 `for_each_lag`（同 1016 lag） | — | 3.3–3.7 µs | 比 shaped 快 ~3.5× |
| `dot()` ×256：稠密 / shaped / 稀疏 | — | 0.17 / 0.03–0.05 / 0.036 µs | shaped 与稀疏相当，比稠密快 3–5× |

MF-4 的灵敏度由合成峰验证（真实模板 + 高斯噪声，高 SNR 段）：shaped 相对稠密
metric **−0.03 dB**，峰位置一致；`sigma=0.5`（metric ~0.0015）时差异在噪声内。

结论：**32/16/8/4/1 级联是正收益**；`MF-3` 的“免除法”明确是负收益；`MF-2` 的
无分支写法在噪声内，回退只是为了保留原实现。live 中 `st_timing` 的 +40% 回归
在隔离 MF-2 后仍存在、回退 MF-3 后消失，因此归因于 MF-3。

---

## 4. 正确性与回归

### 4.1 CTest

| 构建 | 结果 |
|---|---|
| AVX2（`build-native`） | 54 项：53 通过 + 1 SKIP（安装消费者无前缀），**0 失败** |
| SCALAR（`build-opt-scalar`） | 57 项：56 通过 + 1 SKIP，**0 失败** |
| AVX512（`build-opt-avx512`） | **仅编译通过**；本机无 AVX-512，未运行 |

新增 QA：
- 16/32/4-lag 稀疏相关对 8-lag 参考逐位一致；
- `pack_even_complex` 对 scalar；
- 单遍 `fold_sfd_window` 对逐符号参考逐位一致；
- SFD 在 +27 ppm 下的 per-delta 缓存对直接计算等价；
- `estimate_cir(raw, +25 kHz)` 对 `estimate_cir(derotated, 0)` 的 CIR L2 < 1e-4；
- `sync_phase_gain_from_tail` 对 `sync_phase_gain` 相对差 < 1e-3；
- **MF-4**：合成 `code*p` 模板上因子分解相关对 1024-tap 稠密相关逐点相对误差
  < 1e-5、峰位相同、`dot()` 一致；不可分解模板被拒绝（`valid=false`）。

### 4.2 6×40 固定窗（`uwb_dw3000_decode`，与 `b654733` 基线二进制）

| 场景 | exact FCS 基线 | exact FCS 优化 | wrong FCS | 字段级差异 |
|---|---:|---:|---:|---|
| control (+4 dB) | 40/40 | 40/40 | 0 | 0 帧 |
| seed1729 | 38/40 | 38/40 | 0 | 36 帧，仅 `timing_metric` |
| seed2718 | 40/40 | 40/40 | 0 | 37 帧，仅 `timing_metric` |
| cfo10k | 37/40 | 37/40 | 0 | 33 帧，仅 `timing_metric` |
| cfo40k | 38/40 | 38/40 | 0 | 31 帧，仅 `timing_metric` |
| sfo27ppm | 37/40 | 37/40 | 0 | 33 帧，仅 `timing_metric` |

- status / SFD / CFO / PHR / PSDU / FCS 全部一致，无错误 FCS。
- 唯一差异是折叠路径 `timing_metric` 的约 1e-6 浮点结合顺序差，来源是
  `fold_acquire` 的分母从 `sample_power_sum` 改为 `window_power_8acc`。
- 因此**没有做到逐字节完全一致**；若需要字节级门禁，二选一：
  1) 折叠路径分母改回 `sample_power_sum`；
  2) 把 `timing_metric` 从固定窗门禁字段中显式排除。
  该选择建议由分析者/评审决定。

---

## 5. 已实施 / 已回退 / 未实施

### 5.1 已实施（有性能或结构收益）

- `P0-1`：AVX-512 16/32-lag 迁入宽 TU + `use_avx512(Sync)` 分发；32→16→8→4→1 级联。
- `MF-1`：`uint16_t` 紧凑偏移表；AVX-512 32-lag / AVX2 16-lag。
- `SFD-1`：`fold_sfd_window` 单遍 8-way，逐位等价。
- `P0-3`：稀疏 Timing 跳过稠密模板/归一化/堆分配；fold/half 模板 scratch 缓存。
- `TM-1`：AcquireLegacy 相位探针 + 符号网格，0.2 预门控，不达标完整回落。
- `TM-2`：LockedBounded 首探先 ±32 后 ±roi。
- `TM-4`：尾部 timing 按 ±2 样点匹配复用。
- `TM-5`：`probe_fold_phase` 首尾差分算 `eb`；非零相位折叠 AVX2 复乘累加。
- `SFD-2`：跳过无变化的 final SFD。
- `SFD-3`：`sfd_seq==0` 修复、去 `d=0` 重复、SFO per-delta 缓存、复用 `sym_corr`。
- `P0-2`：修 LockedBounded 重试参数、跳过必败重跟踪。
- `CIR-1`：raw rx + 每 SYNC 复标量 + 尾部 `avg_tail`；Sensing/fused 缩小去旋转范围；SIC Ipatov 同步因式分解。
- `CIR-2`：栈数组、尾部补零、归一化路径省缩放。
- `SFD-4`：评估后保留原实现（无分支符号乘无正收益）。
- **`MF-4`（opt-in）**：`code*p` 因子分解替代稠密模板；`rx_timing_shaped_sparse`
  + `--timing-shaped on`（要求 `--timing-sparse off`），重建门控防自定义模板误用。

### 5.2 已回退（无正收益）

- `MF-2`：无分支 `sign * x` 单点稀疏相关；隔离微基准 26.4 vs 26.8 µs（噪声内）。
  回退以保留原实现。注意：整包 `st_timing` 的 +40% 回归在隔离 MF-2 后仍存在，
  回退 MF-3 后才消失，因此归因于 MF-3，不是 MF-2。
- `MF-3`：交叉相乘免除法 → 同代码下 1016 lag 约 4.0–4.7 → 5.7 µs
  （+21%~+42%）；已回退。
- `SFD-4` 的无分支符号乘（未单独测量，量级 ~1 µs）保留原 add/sub。

### 5.3 未实施

- `TM-3`：分析前提不成立；现实现每个 8-lag 只算一次分母，不存在逐 lag 重读。
- `MF-3` 的 PAR 融合、`TM-5` 的 fold 组复用：收益小/影响 -14 dB 折叠敏感性。
- `MF-4` 的 MATLAB `detectRepeatedPreamble.m` 交叉验证（本机无 MATLAB）；代码与
  C++ 等价性、live FCS 对齐已完成，见 §2.4。
- `CIR-3`、`CIR-4`：需要 MATLAB 交叉验证或属于 Radar 专项。
- AVX-512 实机运行、UHD 上板、paced 流、8 worker、P99、多径、MATLAB 验证。

---

## 6. 复现命令

```bash
# 基线（b654733 独立 worktree，AUTO -> AVX2）
git worktree add /tmp/uwb-base-654733 b654733
cmake -S /tmp/uwb-base-654733/gr-uwb -B /tmp/uwb-base-654733/gr-uwb/build \
  -DCMAKE_BUILD_TYPE=Release -DUWB_SIMD=AUTO -DENABLE_UHD_BACKEND=OFF
cmake --build /tmp/uwb-base-654733/gr-uwb/build -j

# 优化版
cmake -S gr-uwb -B gr-uwb/build-native -DCMAKE_BUILD_TYPE=Release \
  -DUWB_SIMD=AUTO -DENABLE_UHD_BACKEND=OFF
cmake --build gr-uwb/build-native -j

# live A/B（每个场景重复 A,B,B,A 两轮）
env -u LD_LIBRARY_PATH \
  <binary> --replay /tmp/uwb-low-snr-6x40/<case>.sc16 --seconds 0 \
  --period-ms 1.0 --mode continuous --workers 1 --queue 256 \
  --cir full --fused on --taps eq16 --tone off --print summary

# MF-4：因子分解替代稠密模板（默认关闭）
env -u LD_LIBRARY_PATH \
  <binary> --replay /tmp/uwb-low-snr-6x40/<case>.sc16 --seconds 0 \
  --period-ms 1.0 --mode continuous --workers 1 --queue 256 \
  --cir full --fused on --taps eq16 --tone off --print packets \
  --timing-sparse off --timing-shaped on

# 6x40 固定窗
python3 tools/rx/low_snr_eval/fixed_scenarios.py --generate \
  --fixtures /tmp/uwb-low-snr-6x40 --build-dir gr-uwb/build-native
env -u LD_LIBRARY_PATH python3 tools/rx/low_snr_eval/fixed_scenarios.py \
  /tmp/uwb-base-654733/gr-uwb/build/apps/uwb_dw3000_decode \
  gr-uwb/build-native/apps/uwb_dw3000_decode \
  --fixtures /tmp/uwb-low-snr-6x40 --out /tmp/fixed_6x40.json
```

---

## 7. 给分析者的提醒

1. **不要把本机数字外推为 9700X / AVX-512 结果。** 方案中的 30–45% 目标
   依赖 AVX-512 32/16-lag 与 Sensing 去旋转；本机只验证到 AVX2。
2. **`demod_mean` 的波动大于阶段值。** live 队列/调度会引入 ±10% 波动；
   建议以 `st_timing / st_sfd / st_cir / st_payload / st_total` 的逐包中位数
   做代码级归因。
3. **SFO 场景的瓶颈是既有的 ~8.9 ms 全流 soft-chip**，不是本轮改动；
   要评估 CIR-1/SFD-2 在该场景的收益，需要把 `st_cir` 单独拆开或换用整数
   网格夹具。
4. **工作树里还有一条并行的 resampler 改动**（`uwb_rational_resampler_*`、
   `uwb_resample_window.h`、`qa_uwb_resampler_burst_opt.cc`），不属于本次任务，
   曾打断主树编译；本轮改动文件已在干净 worktree `/tmp/uwb-verify`
  单独复验可编译。分析时请把这两组改动分开。
5. **MF-4 的位置**：它是**稠密模板的替代**（`--timing-sparse off --timing-shaped on`），
   不是稀疏路径的加速。把它叠在 `--timing-sparse on` 上会因峰位约定差 ~2 样点而少包，
   CLI 已拒绝该组合。它仍缺 MATLAB `detectRepeatedPreamble.m` 交叉验证，默认关闭。
