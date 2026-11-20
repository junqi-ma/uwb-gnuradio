# 多 Agent 并行开发方案：BPM-BPSK 解调与译码算法优化（2026-11-20）

- **适用仓库与分支**：`/home/junqima/workspace/uwb-gnuradio`，分支 `feature/sic-research`
- **基线提交**：`2d8f49b`（上一轮性能工程优化完成后的 HEAD；`st_payload` 已从 `265 → 97.6 µs`，`pay_derot → 0`）
- **执行工具**：OpenCode 多 Subagent 并行开发（Wave 1 三路独立 worktree 零冲突并行 + Wave 2 集成与消融验收）
- **工作边界**：严格遵守仓库根目录 `AGENTS.md`：
  - 仅限原生 UHD/C++ 解调链路（`uwb_core` / `uwb_native`），**不碰 Radar（`uwb_radar_*`）、不推进硬件/TWR/SIC 里程碑**。
  - 保留分数网格回退、融合失败回退（`FailedToFallback` → `run_full_stream_fallback`）与 `timed` fail-closed 语义。
  - 不放宽既有 QA 容差；全量测试必须使用 `env -u LD_LIBRARY_PATH`；禁止 `-march=native`。
  - 区分仿真、文件回放与上板证据；未做的 MATLAB / AVX-512 运行时验证须明确标注。

---

## 1. 背景与核心目标

### 1.1 上一轮优化结果与新瓶颈定位（基线 `2d8f49b`）

上一轮（[Payload 与解调链路优化实验报告](Payload与解调链路优化实验报告_2026-11-20.md)）在**逐包等价**约束下已完成工程级加速：
- `st_total`：`687.3 → 505.6 µs`（1.36×，`control`）；`1537.2 → 1193.4 µs`（`seed1729`）；`1319.9 → 1073.9 µs`（`cfo40k`）。
- `st_payload`：`265.0 → 97.6 µs`（2.71×）；`pay_derot`：`67.2 → 0 µs`；`pay_rs`：`14.7 → 4.4 µs`（低 SNR 5.5×）。
- **暴露出的两大深层瓶颈**：
  1. **性能瓶颈已转为内存带宽**：`pay_soft` 停在 `80–96 µs`（L1 微基准仅 `36–45 µs`），原因是每个符号仍盲搜 **4 个候选窗口**，1208 符号合计读取约 `4 MB` 复数样本（L3/DRAM 带宽受限）。
  2. **算法层面未充分利用 BPM-BPSK 物理层结构**：6 组标准测试夹具在低 SNR / 大 CFO / 大 SFO 下仍有明显丢包（`seed1729` 38/40、`cfo10k` 37/40、`cfo40k` 38/40、`sfo27ppm` 37/40），核心原因在于：
     - 盲搜未知 hop 引入了 2 个纯噪声/多径旁瓣自由度；
     - 4-元双正交（Bi-orthogonal）星座被强行拆分为两个独立边际 LLR，扭曲了 Viterbi 路径竞争；
     - Payload 全程冻结静态相位，未利用 PHR 高增益导频和符号级 DD-PLL 跟踪残余 CFO；
     - 大 SFO（如 27 ppm）触发融合门限失败，被迫回退慢速 legacy 路径。

### 1.2 本轮核心目标（灵敏度 + 吞吐双提升）

| 维度 | 基线 (`2d8f49b`) | 本轮目标 | 主要驱动原理 |
| :--- | :---: | :---: | :--- |
| **低 SNR / CFO / SFO 解码率** | `seed1729` 38/40, `cfo10k` 37/40, `cfo40k` 38/40, `sfo27ppm` 37/40 | **各场景严格不降且显著提升**，并在更低 SNR 压力集（-13/-14 dB）测出明确 dB 增益；**错误 FCS = 0** | 已知 Hop 去噪 + 联合双正交 ML Viterbi + PHR 精校 + DD-PLL + Terminated 尾比特 |
| **`pay_soft`（control）** | `80.1 µs`（读 4 MB） | **`42–55 µs`**（读 2 MB） | Known-Hop 4 候选 → 2 候选，直接击碎内存带宽墙 |
| **`st_payload`（低 SNR）** | `147.8 µs`（硬判失败后再软判） | **`70–95 µs`** | 一步联合 ML 软 Viterbi 替代“硬→软”二次译码 |
| **`sfo27ppm` 融合命中率** | 回退 legacy | **原生走融合路径**（零回退或大幅降低回退） | 符号级 SFO 滑窗 + 8-相位分数延迟 CIR 模板库 |

---

## 2. 物理层优化原理详解

### 原理 A：联合双正交 ML 分支度量（Joint Bi-orthogonal ML Viterbi）

给定跳时位置后，IEEE 802.15.4 HRP BPM-BPSK 每个符号将 2 个卷积编码比特 $(g_0, g_1) \in \{0, 1\}^2$ 映射到二维正交空间 $(y_0, y_1)$ 的 **4 个双正交星座点**：
$$(0,0)\!\to\!(+A,0),\quad (0,1)\!\to\!(-A,0),\quad (1,0)\!\to\!(0,+A),\quad (1,1)\!\to\!(0,-A)$$

设两个 BPM 半区的相干匹配滤波实部输出为 $(m_0, m_1)$，在独立同分布高斯噪声下，负对数似然（去掉与 $(g_0, g_1)$ 无关的公共项 $m_0^2 + m_1^2 + A^2$）精确等价于：
$$\mu_t(g_0, g_1) = -(1 - 2 g_1)\cdot m_{g_0}[t]$$
即：
- $(0,0) \to -m_0[t]$
- $(0,1) \to +m_0[t]$
- $(1,0) \to -m_1[t]$
- $(1,1) \to +m_1[t]$

**为何优于现有拆分 LLR**：现有代码先计算 `llr0 = |m0| - |m1|`、`llr1 = max(m0,m1) - max(-m0,-m1)` 再线性相加。例如当 $m_0=+1.0, m_1=-0.8$ 时，真实第二可能分支 $(1,1)$ 的精确 ML 度量为 $-0.8$（紧追最优 $-1.0$），但拆分 LLR 却算出 $+0.4$（被错判为最差分支），丢掉了 BPM 位置与 BPSK 极性之间的强条件耦合。直接用 $\mu_t(g_0, g_1)$ 驱动 CL-3 软 Viterbi **零近似、更简单、更快**。

同时，IEEE 802.15.4 在最后一个 RS 码字后固定附加 2 个归零尾比特（`0, 0`），使 CL-3 编码器精确终止于 **状态 0**。在校验标准帧一致后，启用 `terminated=true`（强制终态 = 0）可直接强化末尾 16-bit CRC-16 FCS 区域的抗噪能力。

### 原理 B：已知跳时位置（Known-Hop 2 候选）+ 复数 DD-PLL 相位跟踪

1. **Known-Hop（4 候选 → 2 候选）**：
   - IEEE 802.15.4-2020 §15.3.3.2 规定 BPRF（$N_{\text{hop}}=2$）每符号突发跳时索引由 15-bit Scrambler 的首位决定：`hop = (pn[8*s] < 0) ? 1 : 0`（与项目 TX `uwb_hrp_mod_core.h:653` 完全一致）。
   - 每符号只需计算 2 个半区窗口：
     - `half0`: `win0 + hop * 16`（`8 chips × 2 spc = 16 samples`）
     - `half1`: `win0 + hop * 16 + 64`（`32 chips × 2 spc = 64 samples`）
   - 内存读取量减半（`~4 MB → ~2 MB`），同时消除 2 个纯噪声/多径拖尾候选带来的虚警。
2. **判决引导锁相环（DD-PLL）**：
   - 127B 长包（~176 µs）上仅 1.5 kHz 残余 CFO 就会引起 $>95^\circ$ 相位旋转导致后半包 BPSK 极性翻转。
   - 融合相关已算出复数输出 $(M_0, M_1)$。对每个符号先用当前跟踪相位 `rot_cur` 旋转得到 $(z_0, z_1)$，取复数能量更大者 $z_{\text{win}} = (|z_0|^2 \ge |z_1|^2) ? z_0 : z_1$，计算归一化 BPSK 去调制鉴相误差：
     $$e_s = \frac{\text{sign}(\text{Re}(z_{\text{win}}))\cdot \text{Im}(z_{\text{win}})}{|z_{\text{win}}| + \epsilon}$$
   - 通过二阶 PI 环路（比例增益 $\alpha$ + 频偏积分增益 $\beta$，带幅度门限与频偏限幅 $|\Delta\omega| \le \Delta\omega_{\max}$）逐符号更新相位，将信号能量稳定锁定在实轴上。

### 原理 C：PHR 高增益导频细频偏校准 + 符号级 SFO 分数相位模板库

1. **PHR 导频精校**：
   - PHR 21 个符号采用 64 chips/burst（处理增益比 Payload 高 **9 dB**），且经 CL-3 + SECDED 译码验证。
   - 剥离已知极性得 21 个高 SNR 复数样本 $\tilde{Z}_k = (1 - 2 g_1[k])\cdot\text{pol}\cdot Z_k$。当相干性 $|\sum \tilde{Z}_k| / \sum |\tilde{Z}_k| > \tau$（如 0.6）时，利用加权相邻差分估计残余频偏 $\Delta w$ 与 Payload 入口处的精确初始相位 `rot_refined`。
2. **融合路径原生 SFO 跟踪**：
   - 27 ppm SFO 在 127B Payload（~155k samples）上累积漂移 ~4.2 samples（2.1 chips），当前整数网格门限被迫回退 legacy。
   - 预计算 `K = 8` 个子样点相位 $\delta \in \{0, 1/8, \dots, 7/8\}$ 的分数延迟 CIR 模板库（4-tap Catmull-Rom 或窗 Sinc 插值，仅 $8 \times 38 \times 8\text{ B} \approx 2.4\text{ KB}$ 常驻 L1D），每符号按 `base0_double + s * 64.0 * spc_actual` 取整样点偏移 `win0_int` 与相位索引 `phase_bin`，无需回退即可吃下 ±60 ppm SFO。

---

## 3. 多 Agent 并行分工与文件所有权（零冲突矩阵）

### 3.1 执行拓扑

```
Wave 1（三路独立 worktree 并行，严格零文件重叠）:
  ├─ Agent A [JOINT-ML-VITERBI]   : 联合双正交 ML Viterbi + Terminated + 软限幅 + Trace/Rescue
  ├─ Agent B [KNOWN-HOP-DDPLL]    : Known-Hop 2 候选 AVX2 FMA 内核 + 符号级 DD-PLL
  └─ Agent C [PHR-PILOT-SFO-GRID] : PHR 导频细频偏校准 + 8-相位 SFO 分数延迟 CIR 库

        ▼  三路各自独立编译通过 + 新增 QA 100% 绿灯后，依次 --no-ff 合入

Wave 2（单一集成 Agent）:
  └─ Agent D [INTEGRATE-ABLATE]   : Profile 开关 + Scratch/PHR/Payload/Pipeline 串联 + 实采标准一致性核验 + 消融实验 + ABBA 报告
```

### 3.2 严格文件所有权矩阵

| 文件 | Agent A | Agent B | Agent C | Wave 2 (D) | 说明 |
| :--- | :---: | :---: | :---: | :---: | :--- |
| `gr-uwb/include/gnuradio/uwb/uwb_demod_fec.h` | **WRITE** | — | — | READ | 仅 A 修改，保持旧接口不变 |
| `gr-uwb/lib/qa_uwb_demod_core.cc` | **WRITE** | — | — | READ | 仅 A 新增双正交 ML / 尾比特 / 限幅 QA |
| `gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h` | — | **WRITE** | — | READ | 仅 B 新增 Known-Hop 2 候选 + DD-PLL + 可选 SFO bank 入参 |
| `gr-uwb/lib/qa_uwb_rx_fused_cmf.cc` | — | **WRITE** | — | READ | 仅 B 新增 2 候选等价性 + DD-PLL 频偏 QA |
| `gr-uwb/include/gnuradio/uwb/uwb_rx_phase_sfo.h` | — | — | **CREATE** | READ | C 新建纯 DSP 头文件（零外部依赖） |
| `gr-uwb/lib/qa_uwb_rx_phase_sfo.cc` | — | — | **CREATE** | READ | C 新建独立 QA 测试源文件 |
| `gr-uwb/lib/CMakeLists.txt` | — | — | **WRITE** | READ | 仅 C 注册 `qa_uwb_rx_phase_sfo.cc` |
| `gr-uwb/include/gnuradio/uwb/uwb_phy_profile.h` | — | — | — | **WRITE** | Wave 2 新增显式算法开关 |
| `gr-uwb/include/gnuradio/uwb/uwb_demod_scratch.h` | — | — | — | **WRITE** | Wave 2 扩展 2 候选/PHR 复数缓存 |
| `gr-uwb/include/gnuradio/uwb/uwb_demod_phr.h` | — | — | — | **WRITE** | Wave 2 保存 21 个 PHR 复数度量 |
| `gr-uwb/include/gnuradio/uwb/uwb_demod_payload.h` | — | — | — | **WRITE** | Wave 2 串联 Known-Hop + DD-PLL + 联合 ML + 安全回退 |
| `gr-uwb/lib/uwb_demod_pipeline.cc` | — | — | — | **WRITE** | Wave 2 接入 PHR 精校与 SFO 融合门限 |
| `docs/performance/BPM_BPSK解调算法优化实验报告_2026-11-20.md` | — | — | — | **CREATE** | Wave 2 输出最终消融与 ABBA 报告 |

> **并行安全铁律**：Wave 1 中三个 Agent 绝对不得触碰彼此的写文件，也不得修改 `uwb_demod_payload.h` / `uwb_demod_pipeline.cc` / `uwb_phy_profile.h`。Agent B 与 Agent C 的接口通过纯原生指针/基础类型约定解耦（见 §4.2 和 §4.3）。

---

## 4. 各 Subagent 详细任务书（可直接作为 OpenCode Prompt）

### 4.1 Agent A：`JOINT-ML-VITERBI`

- **Worktree / 分支**：`.worktrees/bpm-joint-ml`（分支 `algo/bpm-joint-ml`，基于 `2d8f49b`）
- **独占文件**：`gr-uwb/include/gnuradio/uwb/uwb_demod_fec.h`、`gr-uwb/lib/qa_uwb_demod_core.cc`
- **具体任务**：
  1. **保留全部既有 API 与语义逐位不变**（`vitdec_cl3`, `vitdec_cl3_soft`, `vitdec_cl3_soft_trace`, `decode_payload_hard`, `decode_payload_soft`, `rescue_payload_soft`）。
  2. **实现联合双正交 ML Viterbi 内核**（复用上一轮的 4-state 8-edge 显式展开 + `N <= 2048` 栈回溯缓冲）：
     - 分支度量：`branch(o0, o1) = (o0 == 0) ? (o1 ? +m0[t] : -m0[t]) : (o1 ? +m1[t] : -m1[t])`（最小化路径和）。
     - `vitdec_cl3_biorth(const float* m0, const float* m1, size_t N, std::vector<int8_t>& out, bool terminated = false)`：当 `terminated == true` 时，回溯起点强制取 `best = 0`（而非 `argmin_i surf[i]`）。
     - `vitdec_cl3_biorth_trace(const float* m0, const float* m1, size_t N, Cl3TrellisTrace& tr, bool terminated = false)`：记录每步获胜路径与次优合并边差值 `margins` 及 `lose_u`，格式与现有 `Cl3TrellisTrace` 完全一致。
     - `local_cl3_trellis_flip_biorth(...)`：对应 `local_cl3_trellis_flip` 的双正交版本，供 Phase 2 Trellis-Chase 救援使用。
     - `rescue_payload_biorth(...)`：复用现有 Phase 1（RS Errors-and-Erasures `v=2,4,6`）+ Phase 2（6 single + 15 pair local trellis flips）逻辑，仅将 Phase 2 局部重算替换为 `local_cl3_trellis_flip_biorth`。
  3. **实现符号级抗脉冲干扰软限幅辅助函数**：
     - `clip_biorth_metrics(float* m0, float* m1, size_t N, float clip_factor = 4.0f)`：统计每符号主幅度 `max(|m0|, |m1|)` 的中位数（或采样抽样中位数，O(N) 且零堆分配），将超过 `clip_factor * median` 的异常值软限幅到 `±clip_factor * median`（当 `median <= 0` 或 `N < 16` 时安全跳过）。
  4. **实现端到端双正交 Payload 解码入口**：
     - `decode_payload_biorth_soft(const float* m0p, const float* m1p, const float* m0, const float* m1, size_t payload_symbols, size_t psdu_bits, PayloadResult& out, PayloadSubTimers* sub = nullptr, PayloadDecodeWorkspace* ws = nullptr, bool terminated = false, bool enable_clip = false)`：
       - 将 21 个 PHR 符号与 `payload_symbols` 个 Payload 符号拼接（复用 `ws` 缓冲，稳态零堆分配），一步执行 `vitdec_cl3_biorth_trace` → `finish_rs_crc_payload`；仅当 FCS 失败时触发 `rescue_payload_biorth`。
     - 同样为 PHR 提供 `decode_phr_biorth_soft(const float* m0p, const float* m1p, const std::vector<float>& soft_chips, const NsSfdResult& ns_sfd, PhrResult& out)`（21 符号，注意 PHR 与 Payload 连续编码，PHR 单独译码时**保持 `terminated=false`**）。
- **必须编写的 QA（在 `qa_uwb_demod_core.cc` 中）**：
  1. **精确 ML 反例测试**：构造 $m_0=+1.0, m_1=-0.8$ 等典型耦合符号序列，断言联合 ML 路径度量与手工穷举 4-state 精确 ML 逐比特一致。
  2. **AWGN 统计增益测试（固定种子）**：在低 SNR（如 $E_s/N_0$ 使拆分 LLR 误帧率约 20–40%）下跑 400 帧随机 CL-3 编码帧，断言 `vitdec_cl3_biorth` 的 BER 与 FER **严格低于** 现有拆分 `llr0/llr1 + vitdec_cl3_soft`。
  3. **Terminated vs Truncated 测试**：对含末尾 2 个 `0` 尾比特的编码帧，在包尾注入强噪声，验证 `terminated=true` 显著降低包尾最后 16 bit 误码率，且在无噪场景下与 `truncated` 逐位一致。
  4. **脉冲干扰软限幅测试**：在随机帧中随机注入 3–5 个 `50×` 幅度的突发脉冲干扰，验证 `clip_biorth_metrics` 恢复正确译码。
  5. **构建与验收命令**：
     ```bash
     cmake --build gr-uwb/build-resamp-avx2 -j6 --target uwb_qa_uwb_demod_core.cc
     env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-resamp-avx2 -R "demod_core" --output-on-failure
     ```

---

### 4.2 Agent B：`KNOWN-HOP-DDPLL`

- **Worktree / 分支**：`.worktrees/bpm-known-hop`（分支 `algo/bpm-known-hop`，基于 `2d8f49b`）
- **独占文件**：`gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h`、`gr-uwb/lib/qa_uwb_rx_fused_cmf.cc`
- **具体任务**：
  1. **保留全部既有 4-candidate API 与数值行为不变**（`correlate_field`, `correlate_payload`, `correlate_phr`, `correlate_payload_fused_cfo`, `decide_bpm`, `decide_bpm_soft`）。
  2. **新增 2-候选度量数据结构与 AVX2/FMA 共享模板核**：
     - `struct Metrics2`：`size_t symbols = 0; std::vector<float> m0, m1;`（可选 `std::vector<std::complex<float>> z0, z1;` 仅在 `write_complex=true` 时写入）。
     - `dot2_shared_fixed52(const std::complex<float>* rx0, const std::complex<float>* rx1, const std::complex<float>* tpl, std::complex<float> out[2])`：
       - 13 组 × 4 complex（无标量尾），模板 `tpl` 每组只加载一次，两个候选窗口各用 2 路独立 `fmaddsub` 累加器，非 AVX2 平台回退 `dot_multi_acc8`。
       - 与 `dot4_shared_fixed52` 相比，**每符号内存读取量减半**，寄存器压力减半。
  3. **新增 Known-Hop + DD-PLL + 可选分数相位 Bank 的融合相关入口**：
     - 配置文件结构体：
       ```cpp
       struct DdPllConfig {
           bool enable = false;
           float alpha = 0.08f;            // 比例环路增益（rad）
           float beta = 0.002f;            // 残余频偏积分增益（rad/symbol）
           float max_freq_step = 0.05f;    // 单符号最大频偏步长限幅（rad/symbol, ~60 kHz）
           float min_mag_ratio = 0.25f;    // 低于运行中位/EMA 幅度此比例时跳过鉴相更新
       };
       ```
     - 接口签名（与 Agent C 解耦：通过纯指针接收可选的分数相位 CIR bank）：
       ```cpp
       inline bool correlate_payload_known_hop_fused_cfo(
           const std::complex<float>* rx_raw,
           size_t n,
           const std::complex<float>* fir_taps,
           size_t tap_count,
           int64_t chip_start,
           double samples_per_chip_actual,     // 整数网格传 2.0；SFO 模式传实测 spc
           int64_t payload_start,
           const int8_t* pn,
           size_t symbols,
           int8_t polarity,
           std::complex<float> rot,
           double cfo_w_rad,
           const DdPllConfig& pll_cfg,
           Metrics2& out,
           Scratch& scratch,
           Timings* timings = nullptr,
           bool write_complex = false,
           const std::complex<float>* frac_cir_bank = nullptr, // 可选：phase_bins * tap_count
           size_t frac_phase_bins = 0)
       ```
     - **核心逻辑**：
       - **整数网格快路径**（`frac_cir_bank == nullptr`，`samples_per_chip_actual == 2.0`）：直接复用上一轮的 4-chip 符号对称 LUT（`lut_lo` / `lut_hi`）构建 52-tap 模板；每个符号 `s` 读取 `hop = (ps[0] < 0) ? 1 : 0`，只对 `win0 + hop*16` 和 `win0 + hop*16 + 64` 调用 `dot2_shared_fixed52`。
       - **SFO 分数相位路径**（`frac_cir_bank != nullptr && frac_phase_bins > 0`）：
         - 在包头为 `frac_phase_bins` 个相位各构建一次 4-chip LUT（例如 `8 × 5.6 KB = 45 KB`，或按需缓存，确保在 `Scratch` 内复用零堆分配）。
         - 对每个符号 `s`，计算精确中心样点 `pos_d = base0_d + s * 64.0 * spc_actual`，拆分整样点起点 `win0_int` 与相位索引 `bin`，从对应 `bin` 的 LUT 合成模板并调用 `dot2_shared_fixed52`；同时将符号级 CFO 相位精确锚定到 `win0_int`（`e^{-jw * win0_int}`）。
       - **DD-PLL 相位跟踪**（当 `pll_cfg.enable == true`）：
         - 旋转复数相关值：`z0 = rot_cur * val0`, `z1 = rot_cur * val1`（含 `pol`）。
         - 输出 `out.m0[s] = z0.real()`, `out.m1[s] = z1.real()`。
         - 取复数能量最大半区 `z_win = (norm(z0) >= norm(z1)) ? z0 : z1`，计算 `mag = sqrt(norm(z_win))`，维护幅度的简单指数滑动平均 `mag_ema`；当 `mag >= pll_cfg.min_mag_ratio * mag_ema` 时计算归一化 BPSK 鉴相误差 `err = ((z_win.real() >= 0 ? 1.f : -1.f) * z_win.imag()) / (mag + 1e-12f)`，更新二阶环路 `freq_rad = clamp(freq_rad + beta * err, -max_freq_step, max_freq_step)` 与 `theta -= (alpha * err + freq_rad)`，更新 `rot_cur = exp(j * theta)`（每符号仅一次增量复数乘 + 定期归一化，无高频 `sin/cos` 开销）。
  4. **同时为 PHR 提供 2-候选复数相关辅助**（或在 `correlate_phr` 基础上新增 `correlate_phr_known_hop(...)` 输出 `Metrics2` 及 21 个复数 `z0/z1`），供 Wave 2 接入 PHR 导频校准与双正交译码。
- **必须编写的 QA（在 `qa_uwb_rx_fused_cmf.cc` 中）**：
  1. **2-候选 vs 4-候选数值等价测试**（`pll_cfg.enable = false`, `cfo = 0` 与 `cfo = 40 kHz`）：断言 `Metrics2.m0[s]` / `m1[s]` 与 `correlate_payload_fused_cfo` 中 `hop = (pn[8*s] < 0)` 对应两项的最大绝对误差 `< 1e-5`。
  2. **DD-PLL 抗残余频偏测试**：在 1208 符号合成信号上叠加 **+2 kHz** 和 **-3.5 kHz** 残余 CFO（`cfo_w_rad = 0` 模拟未补偿残差），断言：
     - `pll_cfg.enable = false` 时包尾极性翻转（大量误码）；
     - `pll_cfg.enable = true` 时全包（跳过前几个收敛符号后）符号极性误码率为 **0**，且估计出的 `freq_rad` 正确收敛到残余频偏。
  3. **DD-PLL 零频偏低 SNR 稳定性测试**：在纯低 SNR AWGN 下确认 DD-PLL 不会发生相位滑周（cycle slip）导致性能恶化。
  4. **边界越界保护测试**：SFO 路径下最后一个符号越界必须返回 `false`，不得越界读 `rx_raw`。
  5. **构建与验收命令**：
     ```bash
     cmake --build gr-uwb/build-resamp-avx2 -j6 --target uwb_qa_uwb_rx_fused_cmf.cc
     env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-resamp-avx2 -R "rx_fused_cmf" --output-on-failure
     ```

---

### 4.3 Agent C：`PHR-PILOT-SFO-GRID`

- **Worktree / 分支**：`.worktrees/bpm-phr-sfo`（分支 `algo/bpm-phr-sfo`，基于 `2d8f49b`）
- **独占文件**：
  - `gr-uwb/include/gnuradio/uwb/uwb_rx_phase_sfo.h`（新建）
  - `gr-uwb/lib/qa_uwb_rx_phase_sfo.cc`（新建）
  - `gr-uwb/lib/CMakeLists.txt`（仅添加 `qa_uwb_rx_phase_sfo.cc` 的测试注册）
- **具体任务**：
  1. **PHR 21-符号高增益导频细相位与残余频偏校准**（`namespace gr::uwb::demod::rxopt`）：
     - 结构体 `PhrPhaseRefineResult`：
       ```cpp
       struct PhrPhaseRefineResult {
           bool applied = false;
           std::complex<float> rot_at_payload_start{1.0f, 0.0f}; // Payload s=0 处精确相位旋转
           double delta_cfo_w_rad = 0.0;                         // 残余频偏修正（rad/sample）
           float coherence = 0.0f;                               // 21 符号相干度 [0,1]
       };
       ```
     - 函数 `refine_phase_cfo_from_phr(const std::complex<float>* phr_z_win, size_t nsym, const int8_t* phr_g1, int8_t sfd_polarity, std::complex<float> rot_init, double phr_symbol_samples, double phr_last_to_payload_samples, float min_coherence = 0.65f, double max_delta_w_rad = 2.5e-5)`：
       - 输入 `phr_z_win[0..20]` 为 21 个 PHR 符号获胜半区的复数相关输出（尚未乘 `rot_init` 或已乘 `rot_init`，文档明确约定为**已剥离初始 `cfo_w_rad` 与 `rot_init` 后、乘 `sfd_polarity` 之前的复数相关值**）。
       - 剥离已知 BPSK 极性：$\tilde{Z}_k = (1 - 2\cdot\text{phr\_g1}[k])\cdot \text{sfd\_polarity}\cdot Z_k$。
       - 计算相干度：$C = |\sum_{k=0}^{20} \tilde{Z}_k| / (\sum_{k=0}^{20} |\tilde{Z}_k| + \epsilon)$；若 $C < \text{min\_coherence}$，返回 `applied = false`（保持原 `rot_init` 不变）。
       - 使用相邻共轭乘加权平均估计每 PHR 符号的残余相位步进：
         $$\Delta\theta_{\text{sym}} = \arg\!\left(\sum_{k=0}^{19} \tilde{Z}_{k+1}\tilde{Z}_k^*\right)$$
         换算为 `delta_cfo_w_rad = clamp(dtheta_sym / phr_symbol_samples, -max_delta_w_rad, max_delta_w_rad)`。
       - 用估计出的 `delta_cfo_w_rad` 把 21 个 $\tilde{Z}_k$ 统一折算到中心符号 $k=10$ 求平均相位 $\bar{\theta}_{10}$，再线性外推到 Payload 起点 $s=0$ 得到精确复数旋转 `rot_at_payload_start`（单位模长）。
  2. **8-相位分数延迟 CIR 模板库**（`build_fractional_cir_bank`）：
     - `inline constexpr size_t kSfoPhaseBins = 8;`
     - `inline bool build_fractional_cir_bank(const std::complex<float>* fir_taps, size_t tap_count, size_t phase_bins, std::complex<float>* out_bank)`：
       - `bin = 0` 严格逐位等于原始 `fir_taps`（零失真保证整数网格逐位一致！）。
       - `bin = 1 .. phase_bins-1` 对应分数延迟 $\delta = \text{bin} / \text{phase\_bins} \in (0, 1)$，采用 **4-tap 三次 Catmull-Rom（或 Hann 窗 4-tap Sinc）插值**对 `fir_taps` 重采样（边界处零填充），并保持能量归一化与原始 `fir_taps` 一致。
       - 同时提供辅助函数 `decompose_sfo_position(double pos_samples, size_t phase_bins, int64_t& int_pos, size_t& bin)`，保证 `int_pos + bin/phase_bins` 精确四舍五入到最近子样点（无向零截断偏差、无负偏移错位）。
- **必须编写的 QA（在 `qa_uwb_rx_phase_sfo.cc` 中）**：
  1. **PHR 导频校准精度测试**：
     - 注入固定相位偏置（$35^\circ$）+ 残余 CFO（+2.5 kHz / -3.0 kHz）+ 低 SNR 噪声，断言 `applied == true`、估计出的 `delta_cfo_hz` 误差 `< 200 Hz`、外推至 Payload 起点的相位误差 `< 3°`。
     - 纯随机噪声输入下断言 `applied == false`（安全门限有效拒绝）。
  2. **分数延迟 CIR 模板库测试**：
     - `bin == 0` 与原始 `fir_taps` **逐位一致**；
     - 在合成 27 ppm 与 50 ppm SFO 的 1208 符号脉冲串上，验证包尾（`s=1200`）使用 `decompose_sfo_position + frac_cir_bank` 的相关峰保持 **$> 97\%$** 理想幅度，而固定整数网格在包尾已跌至 $< 20\%$。
  3. **构建与验收命令**：
     ```bash
     cmake --build gr-uwb/build-resamp-avx2 -j6 --target uwb_qa_uwb_rx_phase_sfo.cc
     env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-resamp-avx2 -R "rx_phase_sfo" --output-on-failure
     ```

---

## 5. Wave 2 集成、标准一致性核验与消融验收（Agent D）

待 Wave 1 三个分支全部通过各自 QA 后，在主分支依次 `--no-ff` 合并 `algo/bpm-joint-ml`、`algo/bpm-known-hop`、`algo/bpm-phr-sfo`，由集成 Agent 完成：

### 5.1 开关与回退架构（确保绝对安全、可消融）

在 `Qm35825Profile`（`uwb_phy_profile.h`）增加独立开关（先默认为 `false` 做消融，验收达标后再决定默认开启策略）：
- `bool rx_bpm_known_hop = false;` — 启用 2-候选 Known-Hop 融合相关
- `bool rx_bpm_joint_ml = false;` — 启用双正交联合 ML 一步软 Viterbi（替代“先硬判再拆分 LLR 软判”）
- `bool rx_bpm_terminated_viterbi = false;` — 启用 CL-3 归零尾比特终态约束
- `bool rx_bpm_ddpll = false;` — 启用 Payload 符号级 DD-PLL 相位跟踪
- `bool rx_phr_phase_refine = false;` — 启用 PHR 21-符号导频细相位/频偏精校
- `bool rx_fused_sfo_track = false;` — 启用融合路径 ±60 ppm 分数相位 SFO 跟踪

> **安全回退铁律**：在 `stage_payload_fused_cmf` 中，若新解码路径未能通过 FCS，**仍然允许无缝回退到原有的 `run_full_stream_fallback`（4-candidate + legacy 全流重试）**。这样既保证了正常包享受全部加速与灵敏度红利，又从架构上兜底保证绝不出现“新算法反而丢了旧算法能解的包”。

### 5.2 前置 Gate 0：真实硬件采集 / Golden 向量标准一致性验证

在开启默认之前，必须跑一次全量真实/golden 向量检查：
1. 在 `qa_uwb_demod_core` 与仓库现有 `testdata/` 真实硬件采集（DW1000 / DW3000 / QM35825）上统计：
   - 满足 `winning_hop == (pn[8*s] < 0)` 的符号占比（高 SNR 包应为 100%）；
   - 解码成功帧末尾 2 个卷积尾比特为 `(0, 0)` 的占比（应为 100%）。
2. 若发现任何历史硬件采集不满足其中某一项，必须记录具体芯片/文件，将该项保持为可选或保留对应 fallback。

### 5.3 消融与性能评测方案

1. **6 组既有固定夹具逐项消融**（`tools/rx/low_snr_eval/fixed_scenarios.py`，`/tmp/uwb-low-snr-6x40`）：
   逐级打开开关，记录每一步在 `control / seed1729 / seed2718 / cfo10k / cfo40k / sfo27ppm` 上的 **精确 PSDU 命中数、错误 FCS 数（红线 = 0）、fallback 次数**：
   - `M0`: 基线 `2d8f49b`
   - `M1`: `M0 + Known-Hop`
   - `M2`: `M1 + Joint-ML Viterbi (+ Terminated)`
   - `M3`: `M2 + PHR-Refine + DD-PLL`
   - `M4`: `M3 + Fused-SFO`
2. **极低 SNR 压力扫点（拉开灵敏度曲线）**：
   - 使用现有夹具生成脚本（如 `fixed_scenarios.py` 或 `tools/` 下生成器）额外生成 **-13 dB / -14 dB / -15 dB**（或步进 1 dB 直到 FER 跨越 50%）的纯仿真夹具（放在 `/tmp/uwb-bpm-snr-sweep/`），精确画出 `M0` vs `M4` 的 **FER vs SNR 曲线**，量化实际 dB 增益！
3. **ABBA 性能对比**（`tools/rx/low_snr_eval/ab_fixed.py`，`taskset -c 5,6`，单 worker，至少 2 轮 ABBA）：
   - 对比 `M0` 与 `M4` 在 `control` / `seed1729` / `cfo40k` / `sfo27ppm` 上的 `st_total`、`st_payload`、`pay_soft`、`pay_vit`、`pay_rs`、`full_retry`。
4. **全量 CTest + AVX-512 编译检查**：
   ```bash
   env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-resamp-avx2 --output-on-failure
   env -u LD_LIBRARY_PATH cmake --build gr-uwb/build-resamp-avx512 -j6 \
     --target uwb_core uwb_native uwb_dw3000_live
   ```
5. **输出实验报告**：
   写入 `docs/performance/BPM_BPSK解调算法优化实验报告_2026-11-20.md`，按 `AGENTS.md` 记录提交哈希、CPU/ISA、构建参数、夹具哈希、消融表、低 SNR 曲线、ABBA 中位数与极值，以及未做的上板/9700X AVX-512/MATLAB 交叉验证项。

---

## 6. 风险与应对清单

| 风险 | 影响 | 预防与兜底措施 |
| :--- | :--- | :--- |
| 某些非标/旧硬件帧的 hop 映射或尾比特与标准不符 | Known-Hop / Terminated 导致该类帧失败 | 先跑 Gate 0 实采核验；且失败自动回退原有 4-candidate + truncated legacy 路径 |
| DD-PLL 在极低 SNR 下发生相位滑周（180° 翻转） | 极性全反导致帧失败 | 加入 `min_mag_ratio` 门限 + 频偏限幅 `max_freq_step`；且 PHR 精校先给准初始相位，减小环路带宽需求 |
| SFO 分数相位 bank 过大挤出 L1D | 拖慢 `pay_soft` | 仅在 `|spc - 2.0|` 超出整数门限时构建/使用 8-相位 bank，整数网格零额外开销 |
| 联合 ML 跳过硬判直接软判，在极高 SNR 下 `pay_vit` 略增 | `control` 场景多 1–2 µs | `vitdec_cl3_biorth` 本身是 4-state 8-edge 展开 + 栈回溯（~6–8 µs），而 Known-Hop 省掉 ~35–40 µs，整体 `st_payload` 仍净赚 >30 µs；且低 SNR 直接省掉整套硬判尝试 |
