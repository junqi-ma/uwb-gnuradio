# 多 Agent 并行开发方案：Payload 与解调链路优化（2026-11-20）

本方案面向 OpenCode 多 subagent 并行执行，基于 `feature/sic-research`（`c5a78f4`）在 i7-12700 上的实测分阶段耗时，重点攻克重采样优化后的第一大热点——**Payload 解调（占稳态解调 ~40%）**，并并行收口 `st_cir` 的冗余开销。

---

## 一、实测基线与优化目标

在 i7-12700（AVX2/FMA，单 worker，`--cir full --fused on`，127B 满载）上的实测拆解：

| 阶段 / 子项 | Clean Comm 稳态（`control.sc16`） | Low-SNR（`seed1729.sc16`） | 优化目标 |
| --- | ---: | ---: | ---: |
| `st_payload` 合计 | **263–295 µs**（占解调 ~40%） | **353 µs** | **~65–85 µs**（3.5×–4.5× 提速） |
| ├─ `pay_soft`（融合 CMF 模板 + 4 候选点积） | 158–175 µs | 181 µs | **~40–55 µs** |
| ├─ `pay_derot`（Payload 区间整段 CFO 去旋转） | 65–103 µs | 103 µs | **0 µs**（精确融合进模板/符号相位） |
| ├─ `pay_rs`（4 块 RS(63,55) 解码） | 15–16 µs | 51 µs | **~1–5 µs** |
| └─ `pay_vit`（1229 步 4-state CL-3 Viterbi） | 10 µs | 33 µs | **~2–5 µs** |
| `st_cir`（256 SYNC + 32 tail + 4k NS-SFD FIR） | 140–196 µs | 162 µs | **~100–130 µs** |

**硬性约束（全员遵守）**：
1. 严禁修改 Radar 文件（`uwb_radar_*`、`qa_uwb_radar_*`）。
2. 保留分数网格回退、融合失败回退（`FailedToFallback` → `run_full_stream_fallback`）及 `timed` fail-closed 语义。
3. 不放宽既有 QA 容差；验证一律加 `env -u LD_LIBRARY_PATH`；禁止 `-march=native`。
4. **文件所有权严格隔离**：Wave 1 三个 Agent 仅允许修改各自名下的文件，确保合并零冲突。

---

## 二、并行任务划分与文件所有权矩阵

```text
Wave 1（三路完全并行，0 文件交集）
├── Agent A [FEC-OPT]       : RS(63,55) GF(2^6) 查表 + Horner + CL-3 Viterbi 展开
├── Agent B [FUSED-CMF-OPT] : 4-chip 半模板 LUT + 4 候选共享 FMA + 精确 CFO 融合内核
└── Agent C [CIR-TAIL-OPT]  : estimate_cir tail 合并 + NS-SFD 4-output 批处理 FIR

Wave 2（Coordinator / 集成 Agent，在 A/B/C 合并后执行）
└── Agent D [INTEGRATE]     : pipeline 接入零 pay_derot 路径 + Scratch 缓冲复用 + ABBA 基准验收
```

| 轨道 | 独占可修改文件 | 禁止修改 | 精度契约 |
| --- | --- | --- | --- |
| **Agent A** | `gr-uwb/include/gnuradio/uwb/uwb_hrp_codec.h`<br>`gr-uwb/include/gnuradio/uwb/uwb_demod_fec.h`<br>`gr-uwb/lib/qa_uwb_demod_core.cc`（仅追加 FEC 单测） | `uwb_rx_fused_cmf.h`, `uwb_demod_payload.h`, `uwb_demod_pipeline.cc`, `uwb_demod_scratch.h`, `uwb_demod_softchips.h`, `uwb_cir_fir_simd.h` | **100% 逐位一致（bit-exact）** |
| **Agent B** | `gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h`<br>`gr-uwb/lib/qa_uwb_rx_fused_cmf.cc`（追加精确 CFO 融合 & 4-way QA） | `uwb_hrp_codec.h`, `uwb_demod_fec.h`, `uwb_demod_payload.h`, `uwb_demod_pipeline.cc`, `uwb_demod_scratch.h`, `uwb_demod_softchips.h`, `uwb_cir_fir_simd.h` | 与旧 `correlate_payload` + 精确 `derotate` 误差 `< 1e-5`（允许浮点累加顺序差异） |
| **Agent C** | `gr-uwb/include/gnuradio/uwb/uwb_cir_fir_simd.h`<br>`gr-uwb/include/gnuradio/uwb/uwb_demod_softchips.h` | `uwb_hrp_codec.h`, `uwb_demod_fec.h`, `uwb_rx_fused_cmf.h`, `uwb_demod_payload.h`, `uwb_demod_pipeline.cc`, `uwb_demod_scratch.h` | `cir_avg_tail` 与 4-output FIR 与原标量/AVX2 路径误差 `< 1e-5` |
| **Agent D (Wave 2)** | `gr-uwb/include/gnuradio/uwb/uwb_demod_payload.h`<br>`gr-uwb/include/gnuradio/uwb/uwb_demod_scratch.h`<br>`gr-uwb/lib/uwb_demod_pipeline.cc`<br>`docs/performance/Payload与解调链路优化实验报告_2026-11-20.md` | Radar 文件、已有 QA 容差 | 端到端 FCS 100% 一致（`control.sc16` / `seed1729.sc16` / `cfo40k.sc16`） |

---

## 三、各 Agent 详细原理与实现规格

### Agent A（`FEC-OPT`）：RS(63,55) 查表化 + CL-3 Viterbi 展开

#### 1. 原理与瓶颈
- **RS(63,55)**：`uwb_hrp_codec.h` 当前的 `rs_gf_mul(a, b)` 是 6 次带分支逐位循环，`rs_gf_pow` 又在循环内反复调 `rs_gf_mul`。4 个 RS 块算一次伴随式要跑 4,032 次 `rs_gf_mul`（24,000+ 迭代），在低 SNR 纠错路径（Chien + Forney）下耗时高达 51 µs。
- **GF($2^6$) 只有 64 个元素**（本原多项式 $x^6 + x^5 + 1$，`0x61`，$\alpha = 2$）：
  - 用 `constexpr` 预生成编译期查表：`kGfMul[64][64]`（`uint8_t`，4 KB）或 `kGfLog[64]` + `kGfExp[128]`（192 B）+ `kGfInv[64]`（64 B）+ `kAlphaPow[63]`（63 B）。
  - `rs_syndromes(r63, S8)` 改为 **Horner 法则**（注意 `r(x) = sum_{j=0}^{62} r[j] x^j` 是低次在前，故从 `j = 62` 递减到 `0`：`s = gf_mul(s, ai) ^ r63[j]`），每个 `i = 1..8` 只需 63 次查表 + XOR（少一半乘法，无 `ap` 更新）。
  - `rs_chien_search` 对 $j = 0..62$ 用 `kAlphaInvPow[j]` 查表（不再调用 `rs_gf_pow`），并用 Horner 递推评估 $\Lambda(\alpha^{-j})$。
  - `rs_forney` 同样用查表替代 `rs_gf_pow` / `rs_gf_inv` / `rs_gf_mul`。
- **CL-3 4-state Viterbi**（`uwb_demod_fec.h`）：
  - `poly2trellis(3, [2 5])` 只有 4 个状态、每状态固定 2 条前驱边：
    - 展开 4 个状态的 ACS（Add-Compare-Select），消除 `cl3_trellis_tables` 局部重建和 `for (st=0..3) for (u=0..1)` 双层循环。
    - 特别注意 **tie-breaking 规则必须与原实现完全一致**：原循环按 `st = 0..3, u = 0..1` 顺序用严格小于 `if (m < ns[nst])` 更新，展开后必须保持较小的 `(st, u)` 在相等时胜出！
    - 将回溯表 `back` 改为固定栈上缓冲（当 `N <= 2048` 时 `int8_t stack_back[2048 * 4]`，8 KB，零 `malloc`；超过 2048 再 fallback 到 `std::vector`），消除 `vitdec_cl3` / `vitdec_cl3_soft` 的堆分配。
    - 同步优化 `rs_bits_to_syms` / `rs_syms_to_bits` / `pack_psdu_bytes` 的内层循环。

#### 2. 验收标准
- 在 `qa_uwb_demod_core.cc` 追加（或扩展）测试用例：
  1. 穷举验证新 `rs_gf_mul(a, b)`（$64\times 64$）、`rs_gf_pow(a, n)`、`rs_gf_inv(a)` 与原逐位公式 **100% 逐位一致**。
  2. 随机 0–4 符号错误 + 5 符号不可纠错误码字，验证 `rs_decode_block` 与原实现结果 100% 一致。
  3. 随机硬/软输入（含等权 tie 边界用例），验证展开后的 `vitdec_cl3` 与 `vitdec_cl3_soft` 输出与原 `vitdec_cl3_truncated` **100% 逐位一致**。
- 编译并通过 `env -u LD_LIBRARY_PATH ctest --test-dir <build> -R "demod_core|rx_fused|uhd_sc16_pipeline" --output-on-failure`。

---

### Agent B（`FUSED-CMF-OPT`）：4-chip 半模板 LUT + 4 候选共享 FMA + 精确 CFO 融合

#### 1. 数学原理（精确无近似）
对于 payload（`chips_per_burst = 8`，`spc = 2`，`tap_count = T`，默认 `T = 38`，模板长度 `L = T + 14 = 52`），原始定义对去旋转后的样本 `rx_derot[i] = rx[i] * exp(-j w i)` 计算：
$$M(s, p) = \sum_{m=0}^{L-1} t_s[m]\cdot \Big(rx[\text{win}(s, p) + m]\cdot e^{-j w (\text{win}(s, p) + m)}\Big)$$
其中 $t_s[m] = \sum_{q + 2c = m} h[q]\cdot \text{pn}[s, c]$。提取公共起点相位因子：
$$M(s, p) = e^{-j w\cdot \text{win}(s, p)} \sum_{m=0}^{L-1} \Big(t_s[m]\,e^{-j w m}\Big)\cdot rx[\text{win}(s, p) + m]$$
再将 $t_s[m]\,e^{-j w m}$ 按 $q + 2c = m$ 拆分：
$$t_s[m]\,e^{-j w m} = \sum_{c=0}^{7} \text{pn}[s, c]\cdot e^{-j w\cdot 2c}\cdot \underbrace{\big(h[m - 2c]\cdot e^{-j w (m - 2c)}\big)}_{h_w[m - 2c]}$$
可见**窗内相位斜坡完全可分离**！进一步将 8 个 chip 拆成前 4 chip（$c=0..3$）和后 4 chip（$c=4..7$）：
- 设 $B_k[u] = \sum_{c=0}^{3} s_k[c]\cdot e^{-j w \cdot 2c}\cdot h_w[u - 2c]$（$u = 0..T+5$，默认长度 **44**），其中 $s_k[0] = +1$、$s_k[1..3] \in \{\pm 1\}$，仅 **$2^3 = 8$ 种基础模板**！
- 后 4 chip 只是把同一组 8 个基础模板乘以常量标量 $e^{-j w \cdot 8}$ 并右移 8 个样本：
  $$B_k^{\text{hi}}[u] = e^{-j w \cdot 8}\cdot B_k[u]$$
  16 个基础模板总大小 `16 * 44 * 8 B = 5.6 KB`（全部常驻 L1D cache），每包构造仅需 < 1 µs！
- 对每个符号 $s$，读取 `pn[8s .. 8s+7]`：
  - `idx_lo` = 3-bit 索引（由 `pn[0]*pn[1..3]` 决定），`sign_lo = pn[0] ∈ {±1}`
  - `idx_hi` = 3-bit 索引（由 `pn[4]*pn[5..7]` 决定），`sign_hi = pn[4] ∈ {±1}`
  - 长度 52 的精确频偏融合模板 `tpl[0..51]` 只需一次合并（无需清零）：
    - `0..7`：`sign_lo * B_lo[idx_lo][0..7]`
    - `8..43`：`sign_lo * B_lo[idx_lo][8..43] + sign_hi * B_hi[idx_hi][0..35]`
    - `44..51`：`sign_hi * B_hi[idx_hi][36..43]`
- **4 候选共享模板 SIMD 点积（`dot4_fixed52_avx2`）**：
  - 4 个候选 `p = 0, 1, 2, 3` 对应 `rx` 指针 `x0 = rx + win0 + 0`、`x1 = rx + win0 + 64`、`x2 = rx + win0 + 16`、`x3 = rx + win0 + 80`，**共享同一个 `tpl[0..51]`**！
  - 在单个 `g = 0..12`（13 组 × 4 复数 = 52 复数，**0 scalar tail**）循环内：
    - `tpl` 只 load **1 次**（同时准备 `ht` 和 `hsw = permute(ht, 0xB1)`）；
    - 对 4 个候选分别维护 2 路累加器（或 4 个独立复数累加器 `acc[0..3]`，用 `fmaddsub` 累加；注意每次迭代 4 个候选独立无数据依赖，已天然有 4 路 ILP，可彻底打满 AVX2 双 FMA 端口！）。
  - 水平归约得到 4 个复数 `m[0..3]` 后，乘以每符号-每候选的标量相位：
    $$\text{rot\_eff}(s, p) = \big(\text{polarity}\cdot \text{rot}\cdot e^{-j w\cdot \text{win0}(s)}\big)\cdot e^{-j w\cdot \text{off}[p]}$$
    - 其中 $e^{-j w\cdot \text{off}[p]}$（4 个候选）在包开始时预计算为 4 个常量；
    - $e^{-j w\cdot \text{win0}(s)}$ 用精确 `double` 相位递推（每 32 符号 re-anchor 一次 `cos/sin`，杜绝浮点漂移）。
    - 取实部直接写入 `out.signed_value[4*s + p]`（可选标志 `write_complex = false` 时跳过写入 38 KB `out.complex_value`，仅 QA 模式开启）。
- **其他热路径清理**：
  - 将 `pn` 的 `±1` 校验改为可选（或由 `prepare_fused_payload_pn` 保证），避免每包遍历 9,664 字节。
  - 将 `steady_clock::now()` 从每 64 符号块循环内移出（`timings == nullptr` 时完全零时钟调用；非空时只在循环外各记 1 次）。

#### 2. 接口契约（供 Wave 2 集成 Agent 直接调用）
在 `gr-uwb/include/gnuradio/uwb/uwb_rx_fused_cmf.h` 中：
1. **保留**原有 `correlate_payload(...)` 与 `correlate_phr(...)` 签名不变（可内部直接复用 4-chip LUT + 4-way FMA，传 `cfo_w_rad = 0.0`），确保既有 QA 无需改动即可编译通过。
2. **新增**精确 CFO 融合入口（供 Wave 2 接入原始未旋转 `rx`）：
   ```cpp
   inline bool correlate_payload_fused_cfo(
       const std::complex<float>* rx_raw,
       size_t n,
       const std::complex<float>* fir_taps,
       size_t tap_count,
       int64_t chip_start,
       size_t samples_per_chip,
       int64_t payload_start,
       const int8_t* pn,
       size_t symbols,
       int8_t polarity,
       std::complex<float> rot,
       double cfo_w_rad,        // 2*pi*cfo_hz / sample_rate; 0.0 == 无旋转
       Metrics& out,
       Scratch& scratch,
       Timings* timings = nullptr,
       bool write_complex = false);
   ```
3. `rxfusion::Scratch` 内新增预分配字段存放 `lut_lo[8][44]`、`lut_hi[8][44]` 及临时 `tpl[64]`（栈/结构体成员，零堆分配）。
4. 若 `UWB_SIMD_HAVE_AVX512` 启用，可在不引入新 TU 冲突的前提下保持编译通过（或留清晰扩展点供 9700X 实测）。

#### 3. 验收标准
- 在 `qa_uwb_rx_fused_cmf.cc` 追加单测：
  1. `cfo_hz = 0, 1000, 10000, 40000` Hz 下，比较 `correlate_payload_fused_cfo(rx_raw, ..., cfo_w)` 与先执行 `derotate_range_scalar(rx_raw -> rx_derot)` 再执行原标量参考路径的 `signed_value` 和 `complex_value`：最大绝对误差 `< 1e-5`（相对误差 `< 1e-5`），且硬判决 `g0/g1` 100% 一致。
  2. 测试 `tap_count = 38`（fixed-52 快速路径）以及非 38（如 `tap_count = 24, 48` 通用路径）、越界拒绝、非法 `pn` 拒绝等全部边界条件。
- 编译并通过 `env -u LD_LIBRARY_PATH ctest --test-dir <build> -R "rx_fused_cmf|rx_fused_phr|demod_core" --output-on-failure`。

---

### Agent C（`CIR-TAIL-OPT`）：`st_cir` 冗余消除与 4-output 批处理 FIR

#### 1. 原理与瓶颈
`st_cir` 目前占稳态解调 20–28%（140–196 µs），主要来自两处可安全压榨的冗余：
1. **`estimate_cir` 里的双遍 SYNC 叠加**（[uwb_demod_softchips.h](file:///home/junqima/workspace/uwb-gnuradio/gr-uwb/include/gnuradio/uwb/uwb_demod_softchips.h#L104-L190)）：
   - 主循环叠加 256 个 SYNC（按 `llround(k * period)`）；紧接着 L147-190 又单独读了一遍最后 32 个 SYNC（按整数网格 `k * 1016`）填充 `scratch.cir_avg_tail`，再做一遍 32 次 `cos/sin` + `vector_accumulate_scaled_complex` + `build_rotation_ramp` + `vector_mul_complex`。
   - **优化**：在主循环中，当 `k >= reps - tail_n` 且满足 `rs == start + k * 1016`（即 `integer_grid_ok` 成立时几乎 100% 满足！）时，直接把同一份 `rx + lo`（和同一个 `alpha`！）累加到 `scratch.cir_avg_tail` 中；主循环结束后直接复用已经算好的 `scratch.cir_phase_rot`（前 `tail_span` 项与主 ramp 逐位相同，无需第二次 `build_rotation_ramp`！）。若任意一个 tail SYNC 偏离整数网格，再回退到原独立 tail 循环。
   - **收益**：省掉 32×1053 样本的第二遍内存读取、32 次 `cos/sin` 和 1 次 1052-sample ramp 构建。
2. **`stage_softchips_from_cir` 的 4-output 批处理 FIR**（[uwb_cir_fir_simd.h](file:///home/junqima/workspace/uwb-gnuradio/gr-uwb/include/gnuradio/uwb/uwb_cir_fir_simd.h#L224-L250) & [uwb_demod_softchips.h](file:///home/junqima/workspace/uwb-gnuradio/gr-uwb/include/gnuradio/uwb/uwb_demod_softchips.h#L500-L508)）：
   - 生产路径 `skip_sync_fir = true` 时，仍需为 NS-SFD 计算约 4,100 个 38-tap 复数 FIR 软片（`spc_i == 2`）。
   - 当前每次只算 1 个输出 `dot_avx2_fixed38(rx0 + win, h_conj, 38)`，`h_conj` 被重复 load 4,100 遍。
   - **优化**：在 `uwb_cir_fir_simd.h` 中新增 `dot_avx2_fixed38_x4(rx0 + win, h_conj, out4)`（`spc_i == 2`，4 个相邻输出起点分别在 `win + 0, +2, +4, +6`），每个 4-tap 组 `h_conj` 只 load 1 次，同时驱动 4 个输出的 FMA 累加器（4 路独立，隐藏延迟），最后 2-tap 标量尾同样 4 路并算。边界剩余 `< 4` 个 chip 走原 `dot_avx2_fixed38`。

#### 2. 验收标准
- 不改 `uwb_demod_scratch.h`、不改任何外部函数签名。
- 编译并通过 `env -u LD_LIBRARY_PATH ctest --test-dir <build> -R "demod_core|rx_fused|uhd_sc16_pipeline" --output-on-failure`。
- 验证 `cir_avg_tail` 合并路径与原独立循环路径在 `integer_grid_ok` 下的输出误差 `< 1e-6`，`dot_avx2_fixed38_x4` 与 `dot_avx2_fixed38` 误差 `< 1e-6`。

---

### Wave 2：Agent D（`INTEGRATE`）集成、零分配收口与端到端验收

在 Agent A / B / C 全部通过各自 QA 并合并后执行：

1. **接入零 `pay_derot` 路径**（[uwb_demod_pipeline.cc](file:///home/junqima/workspace/uwb-gnuradio/gr-uwb/lib/uwb_demod_pipeline.cc#L1122-L1185) & [uwb_demod_payload.h](file:///home/junqima/workspace/uwb-gnuradio/gr-uwb/include/gnuradio/uwb/uwb_demod_payload.h#L226-L368)）：
   - 在 `try_fused_payload_decode` 中，**删除/跳过**对 payload 区间的 `rotate_uncovered_tail_range` 调用（注意：失败回退路径 `run_full_stream_fallback` 和非融合 `try_ondemand_extension_and_payload_decode` 保持原样，仍会自行补旋转，确保 fail-safe！）。
   - 将原始 `rx`（未旋转）与 `cfo_w_rad = 2.0 * M_PI * res.cfo.cfo_hz / profile.sample_rate` 传入 `stage_payload_fused_cmf` → `rxfusion::correlate_payload_fused_cfo(...)`。
   - 保持 `pay_derot = 0`（统计口径透明，不隐藏耗时）。
2. **消除 Payload 热路径 `std::vector` 临时堆分配**（`uwb_demod_scratch.h` & `uwb_demod_payload.h`）：
   - 在 `DemodScratch::FusedCmfState` 中增加复用缓冲 `g0, g1, llr0, llr1, rx_bits, decoded, rs_cw, psdu_bits_out`，在 `stage_payload_fused_cmf` / `decode_payload_hard` / `decode_payload_soft` 中复用容量，消除每包 10+ 次 `malloc/free`。
3. **验证矩阵（必须全部执行并记录数据）**：
   - **核心 CTest**：
     ```bash
     env -u LD_LIBRARY_PATH ctest --test-dir gr-uwb/build-resamp-avx2 \
       -R "demod_core|rx_fused|cfo_derotate|uhd_sc16_pipeline|resampler_transposed" \
       --output-on-failure
     ```
   - **AVX-512 编译检查**：
     ```bash
     cmake --build gr-uwb/build-resamp-avx512 --target uwb_core uwb_native uwb_dw3000_live -j
     ```
   - **端到端 ABBA 回放对照**（单 worker，`taskset -c 5,6`，对比基线 `c5a78f4` vs 优化后）：
     - `control.sc16`（clean comm，acquire + locked）：验证 40/40 FCS 及 `st_payload` / `pay_soft` / `pay_derot` / `pay_rs` / `pay_vit` / `st_cir` / `st_total` 变化。
     - `seed1729.sc16`（low-SNR，含软判决重试）：验证 FCS 一致性及低 SNR `pay_vit` / `pay_rs` 提速。
     - `cfo40k.sc16`（大频偏 40 kHz）：**重点验证 `pay_derot` 精确融合在大 CFO 下 FCS 无任何退化！**
   - **产出实验报告**：写入 `docs/performance/Payload与解调链路优化实验报告_2026-11-20.md`，明确标注未在 9700X 上执行 AVX-512 运行时测试及未做 MATLAB 交叉验证的边界。

---

## 四、可直接发给 OpenCode 的多 Subagent 启动指令

```text
请严格按照 `docs/performance/多Agent并行开发方案_Payload与解调链路优化_2026-11-20.md` 执行本轮优化：

1. Wave 1 启动 3 个并行 subagent（各自隔离工作区，严禁越界修改其他 Agent 的文件）：
   - Agent A (FEC-OPT)：仅修改 `uwb_hrp_codec.h`、`uwb_demod_fec.h`、`qa_uwb_demod_core.cc`，实现 GF(2^6) 查表 + Horner RS(63,55) + 4-state CL-3 Viterbi 展开（含 tie-breaking 一致性），保证 100% bit-exact 并跑通单测。
   - Agent B (FUSED-CMF-OPT)：仅修改 `uwb_rx_fused_cmf.h`、`qa_uwb_rx_fused_cmf.cc`，实现 4-chip 符号对称 LUT（8+8 基础半模板）+ 4 候选共享模板 AVX2 FMA 点积 + 精确 CFO 融合入口 `correlate_payload_fused_cfo()`，补齐 0/1k/10k/40kHz CFO 数值一致性 QA 并跑通。
   - Agent C (CIR-TAIL-OPT)：仅修改 `uwb_cir_fir_simd.h`、`uwb_demod_softchips.h`，实现 `estimate_cir` 整数网格 tail 合并复用 + `dot_avx2_fixed38_x4` 4-output 批处理 FIR，跑通 QA。

2. Wave 2 汇总合并 A/B/C 后执行集成与验收：
   - 接入 `uwb_demod_pipeline.cc` 与 `uwb_demod_payload.h` 的零 `pay_derot` 路径，并在 `uwb_demod_scratch.h` 复用 payload 临时缓冲。
   - 验证 `build-resamp-avx2` 核心 CTest 与 `build-resamp-avx512` 编译。
   - 在 `control.sc16`、`seed1729.sc16`、`cfo40k.sc16` 上运行单 worker ABBA 对比，重点记录 `st_payload`（及 `pay_soft`/`pay_derot`/`pay_rs`/`pay_vit`）、`st_cir`、`st_total` 与 FCS 一致性，将结果写入 `docs/performance/Payload与解调链路优化实验报告_2026-11-20.md`。
```
