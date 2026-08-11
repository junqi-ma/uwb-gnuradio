# GNU Radio 软件升采样测试报告：737.28 MS/s → 998.4 MS/s（65/48）

> 对应任务书：`开发方案_GNURadio软件升采样_737.28M到998.4M.md`
> 本报告随 Phase 演进持续更新。当前进度：**Phase 0（调查与基线）进行中**。

## 1. 开发前强制调查（§2）

### 1.1 GNU Radio 环境（§2.1）

| 项 | 值 |
|---|---|
| GNU Radio | **3.10.1.1**（Ubuntu 包，`gnuradio-config-info --version`） |
| VOLK | **2.5.1** |
| Python 绑定 | `/usr/lib/python3/dist-packages/gnuradio/` |
| 本机实现源码 | 无（Ubuntu 未装 `gnuradio` 源码包；调查依据官方 tag **v3.10.1.1**） |
| 编译环境 | gcc（见 §benchmark 运行信息） |

调查依据源码：
- `gr-filter/lib/rational_resampler_impl.h` / `rational_resampler_impl.cc`（v3.10.1.1）
- `gr-filter/lib/fir_filter.cc`（v3.10.1.1）
- 本机公开接口 `/usr/include/gnuradio/filter/rational_resampler.h`

### 1.2 内置 `rational_resampler_ccf` 语义

- **block 类型**：`gr::block`，`forecast()` + `general_work()`（**不是** sync/interp/decim）。
  - 输入 `sizeof(gr_complex)`，输出 `sizeof(gr_complex)`，taps 为 `float`。
- **taps 拆相（`install_taps`）**：
  1. `set_taps()` 把 taps 末尾补零到 `interpolation(=65)` 的整数倍；
  2. `xtaps[i % 65][i / 65] = taps[i]` —— arm `n` 持有 `taps[n], taps[n+65], taps[n+130], …`；
  3. `set_history(nt)`，`nt = 补齐后长度 / 65`。
- **`forecast(noutput_items)`**：
  ```cpp
  nreqd = max(1U, (int)((double)(noutput_items + 1) * decim / interp) + history - 1);
  ```
  使用 double 比例向上取整（`(noutput_items+1)` 保证有余量），再加 history。
- **`general_work`**：维护整数 phase 计数器 `d_ctr`：
  ```cpp
  out[i++] = d_firs[ctr].filter(in);   // 用第 ctr 个 polyphase arm
  ctr += decimation;                   // ctr += 48
  while (ctr >= interpolation) { ctr -= interpolation; in++; count++; }  // 消费输入
  ```
  `consume_each(count)`，返回 `i`（实际产出）。跨调用保持 `d_ctr`。
- **tag 传播**：`gr::block` 默认 **TPP_ALL**（scheduler 按 relative rate 映射 tag offset）。
- **SIMD**：`kernel::fir_filter<gr_complex, gr_complex, float>`（`fir_filter_ccf`）内部调
  `volk_32fc_32f_dot_prod_32fc_a` —— **内置块已用 VOLK SIMD**。
- **packed complex int16**：仅 `rational_resampler_scc`（int16 → complex，complex taps）；
  另有 `rational_resampler_fsf`（float → int16）。**无 SC16 → SC16 内置块**。

### 1.3 本项目相似 block（§2.2）

已阅读（复用以建立约定）：

- `gr-uwb/lib/uwb_scheduled_extractor.cc` + `uwb_scheduled_extractor.h`：`gr::sync_block`，
  `set_max_noutput_items(1048576)`，消息端口 + 后台 worker；大 chunk、无热路径分配。
- `gr-uwb/lib/uwb_detector_sc16.cc` + `uwb_detector_sc16.h`：SC16 检测。
- `gr-uwb/apps/benchmark_detector_sc16.cc`：`BulkRepeatSource`（循环源，work 内只 memcpy）、
  head + message_debug/null_sink、7 轮 warm-up、输出编译器/速率/格式。
- `gr-uwb/lib/CMakeLists.txt`：Boost.Test 注册到 CTest；benchmark 为独立 executable。

项目约定：SC16 stream item 为 `std::complex<int16_t>`；`uwb_scheduled_extractor_core.h`
纯算法与 adapter 分离（本项目既有模式）；QA 用 Boost.Test；不破坏现有 detector/demod QA。

### 1.4 基线（§2.3）

```bash
git status -sb
# ## master...origin/main
# ?? 开发方案_GNURadio软件升采样_737.28M到998.4M.md   (untracked 文档，非代码)

cmake --build gr-uwb/build -j   # 100% built
ctest --test-dir gr-uwb/build --output-on-failure
# 9/9 tests passed (0 failed) —— 基线干净
```

| 结论 | 值 |
|---|---|
| 基线 CTest | **9/9 通过** |
| 平台 | i7-12700，20 逻辑核，AVX2（无 AVX-512） |
| scipy/numpy | 1.8.0 / 1.21.5（本机无 MATLAB/octave，golden 用 scipy 作 `upfirdn` 等价） |

## 2. Phase 1：滤波器设计与 golden（§5）

### 2.1 设计决策

- 输入 737.28 MHz，Nyquist = ±368.64 MHz。实测 UWB 测试信号（998.4 MHz 域）：
  90% 能量 ≤ 231 MHz、99% ≤ 287 MHz、99.9% ≤ 330 MHz、−50 dBc 至 422 MHz。
- anti-image 滤波器在**虚拟速率** 65×737.28 = 47.9232 GHz 设计：
  - passband  [0, B]
  - stopband  [R_in − B, …]（首镜像从 R_in − B 开始；周期响应自动覆盖全部镜像）
  - transition = R_in − 2B（最大可用宽度 → 最少 taps）
- 两个档位（§5.2 的 Quality / Realtime）：

| 档位 | B (MHz) | taps | arms | DC(sum) | 通带纹波 | 阻带(最差) | 群时延(输出) |
|---|---|---|---|---|---|---|---|
| quality | 330 | 5363 | 83 | 65.0 | 0.00 dB | **−97 dB** | 55.85 |
| realtime | 290 | 2635 | 41 | 65.0 | 0.00 dB | **−96 dB** | 27.44 |

  规格：纹波 ≤ 0.1 dB、阻带 ≥ 70 dB，均大幅满足（经验验证，非公式估算）。
  单输出 MAC（quality）≈ 83 → 998.4M×83 ≈ 8.3e10 MAC/s，本机可行。
  GNU Radio 内置默认设计（fractional_bw=0.4）约 18.3k taps / 281 arms，是本
  quality 设计的 3.4 倍算力 → 自定义 block 有明确收益空间。

- 设计脚本：`testdata/design_resampler_737p28_to_998p4.py`（canonical，scipy）
  与 `.m`（MATLAB 交叉验证）。导出 taps 至 `testdata/resampler_65_48/`。

### 2.2 Golden 与契约（§8.1）

- golden 由 `scipy.signal.upfirdn(taps, x, 65, 48)` 生成（**注意参数顺序**：
  `upfirdn(h, x, up, down)`，h 在前）。scipy 与 MATLAB `upfirdn(x,h,p,q)`
  同一多相算法，可交叉对照。导出脚本：`export_resampler_65_48_golden.py` 与 `.m`。
- 七组 golden 存于 `testdata/resampler_65_48/golden/`：impulse / dc / tone_low /
  tone_pb(300MHz) / tone_sb(420MHz) / random / uwb，每组 `_in.cf32`(737.28M) 与
  `_out.cf32`(998.4M) 对。uwb 输入由 998.4M cfile 的 preamble 窗口经 48/65
  抗混叠抽取生成（模拟 X410 捕获），能量比 out/in = 0.987。

- **upfirdn 长度/相位约定（core 必须精确复现）**：
  - 总输出长度：`len_out = ceil(((N−1)·65 + T)/48)`（T=5363 时逐 N 验证）。
  - 相位律：`y[m] = Σ_k taps[(mM mod L) + 65k] · x[floor(mM/L) − k]`
    （输出 m 用 arm `mM mod 65`，输入窗口 `[floor(mM/48) − 82, floor(mM/48)]`）。
  - 即 upfirdn = 输入两侧零填充的完整卷积（含暖机瞬态与 EOS 尾）。

- **与 GNU Radio 内置块边界差异（已记录，非 bug）**：内置 `rational_resampler_ccf`
  对 4096 输入给 5436 输出（约减 2×群时延），其 FIR 窗口**前向非因果**，首个输出
  即为稳态（无暖机瞬态）；对起点 impulse 几乎全零。自定义 block 契约定为与
  upfirdn 一致（含瞬态与精确长度），稳态样本两者逐点一致。tag 映射、EOS、
  chunk 不变性均按 upfirdn 契约测试。

### 2.3 Level A 正确性对照（内置块 vs upfirdn）

- DC：GR 稳态 1.000000（std 1e-7）≈ golden 稳态 1.0。**一致**。
- 平均增益 = sum(taps)/L = 65/65 = 1.0（常数输入逐样本按相位 arm 变化，
  §5.3 的 DC 增益定义：滤波器 sum=65、重采样器平均增益=1.0，需在文档固定）。
- 长度/边界：GR 5436 vs golden 5658，差异 ≈ 2×群时延（见上）。

## 3. Phase 0 结论

### 3.1 Level A 内置 `rational_resampler_ccf` 吞吐（实测）

**可执行文件**：`gr-uwb/apps/benchmark_resampler_65_48`  
**命令**（满负荷默认参数）：

```bash
cd gr-uwb/build
cmake .. && cmake --build . -j$(nproc) --target benchmark_resampler_65_48
./apps/benchmark_resampler_65_48            # target_input_samples=2e9, buffer_items=1<<20
# 快速冒烟：./apps/benchmark_resampler_65_48 50000000
```

**平台**（与 §1.1 一致）：GNU Radio 3.10.1.1，VOLK 2.5.1 / `avx2_64_mmx_orc`，
gcc 11.4.0，i7-12700 20 逻辑核 AVX2+FMA。

**测量合同**：

- Flowgraph：`BulkRepeatSource(CF32)` → `head` → `rational_resampler_ccf(65,48,taps)` → FNV-1a `ChecksumSink`
- Source-only 基线：`BulkRepeatSource` → `head` → `null_sink`（分离内存/调度带宽）
- 每 profile：warm-up + 7 轮 timed（各 2e9 输入样点，单轮 wall ≈ 28–47 s）+ ~30 s sustained
- 无 `Throttle`；timed path 无文件 I/O；`work()` 内无分配
- 报告 median / min / max；checksum 防优化消除

| 模式 / taps profile | 实际 tap 数（`taps()`） | 约每相 taps | median 输入 MS/s | median 输出 MS/s | median RT ratio¹ | min / max 输入 MS/s | 30s-class sustained 输入/输出 MS/s | checksum（2e9 轮） |
|---|---:|---:|---:|---:|---:|---|---|---|
| source_only（无 resampler） | — | — | **1359.0** | — | — | 1275.7 / 1377.7 | — | n/a |
| built-in **default**（空 taps → GR 自设计） | **2145** | 33 | **67.23** | **91.04** | **0.091** | 58.57 / 71.56 | 68.54 / 92.81 | `0xcf44c15b06a903a5` |
| built-in **quality**（`taps_quality.txt` 5363 → pad 5395） | **5395** | 83 | **44.40** | **60.12** | **0.060** | 42.68 / 45.27 | 43.62 / 59.07 | `0x712a0ad7a42247ad` |
| built-in **realtime**（`taps_realtime.txt` 2635 → pad 2665） | **2665** | 41 | **49.30** | **66.76** | **0.067** | 47.49 / 50.93 | 49.08 / 66.46 | `0xb8f735a01feaf4cc` |

¹ `realtime_ratio = output_MSps / 998.4`。最低门槛要求 RT ≥ 1.0（输入 ≥ 737.28 MS/s 且输出 ≥ 998.4 MS/s）。

**Sustained / 资源**：

| profile | wall s | 输入样点 | 输出样点 | expected≈N·65/48 | output_count_drift | RSS before→after (kB) | ΔRSS |
|---|---:|---:|---:|---:|---:|---|---:|
| default | 29.43 | 2.017e9 | 2.731e9 | 2.731e9 | **−43** | 31556 → 66128 | +34.6 MB |
| quality | 45.85 | 2.000e9 | 2.708e9 | 2.708e9 | **−111** | 31716 → 66248 | +34.5 MB |
| realtime | 40.75 | 2.000e9 | 2.708e9 | 2.708e9 | **−53** | 31716 → 66060 | +34.3 MB |

- drift 量级 ≈ 滤波器 history（≈ taps/65），是 head 截断下 FIR 启动边界效应，**不是**长时间线性漂移。
- RSS 在单次 sustained 内一次性抬升约 34 MB（flowgraph buffer），未见持续爬升泄漏迹象；需更长 soak 才能严格排除。
- Source-only ≈ **1.36 GS/s** CF32（≈ 10.9 GB/s 读写），远高于合同 5.90/7.99 GB/s → **瓶颈在 resampler 算力，不是 flowgraph/内存带宽**。

**与先前估算的差异（重要）**：

- 调查笔记曾按公式估计 GR 默认设计 ≈ 18.3k taps / 281 相；**实测** `rational_resampler_ccf(65,48)` 空 taps 自设计为 **2145 taps / 33 相**，故 default 反而是三档中**最快**的，而不是最慢。
- quality（83 MAC/out）相对 default（33 MAC/out）约 1.5× 更慢（67 → 44 MS/s in），方向与算力一致，但未达线性 83/33，说明还有调度/带宽常数项。
- 先前“default 比 quality 慢约 3×”的预期**不成立**（基于错误 tap 数估计）；以实测为准。

### 3.2 是否达到最低实时门槛

| 门槛 | 要求 | Level A 最优（default taps） | 结论 |
|---|---|---|---|
| 最低可行 | 输入 ≥ 737.28 MS/s，输出 ≥ 998.4 MS/s | 输入 **67.2** / 输出 **91.0**（RT ≈ **0.091×**） | **未达到（FAIL）** |
| 生产 1.2× | 输入 ≥ 884.7 / 输出 ≥ 1198 | 同上 | **未达到（FAIL）** |

**结论**：Level A（GNU Radio 内置 `rational_resampler_ccf`，三种 taps 配置）在本机 **不能** 支撑单通道 737.28 → 998.4 MS/s 连续实时。最优配置也仅约 **9%** 实时率。Source-only 证明 host 内存路径有余量，必须进入 **Level B（固定 65/48 优化 C++ block）** 并用 profile 定位 FIR / phase 控制 / 调度开销后再优化。

CTest 回归：`ctest --test-dir gr-uwb/build --output-on-failure` → **9/9 passed**（benchmark 未破坏现有 QA）。

---

## 4. Phase 2：Level B 固定 65/48 block（`UwbRationalResamplerCcf65_48`）

> 实现规格：`docs/performance/规格_固定65_48重采样core契约.md`  
> 日期：2026-08-11  
> 平台：同 §1.1（GR 3.10.1.1，VOLK 2.5.1，gcc 11.4.0，i7-12700 AVX2+FMA）

### 4.1 Block 类型与 scheduler 语义

| 项 | 选择 | 理由 |
|---|---|---|
| Block 类型 | `gr::block` + `forecast()` + `general_work()` | 65/48 变速流，非 1:1 / 纯整抽取 / 纯整插值 |
| `set_relative_rate(65, 48)` | 整数重载 | 精确速率合同，避免 double 漂移 |
| `set_output_multiple(65)` | 是 | 稳态宏块对齐；EOS 尾部可 return &lt; grant |
| `set_max_noutput_items(1<<20)` | 是 | 大 chunk，降低调度频率 |
| `set_history` | **否** | 延迟线在 core 内维护，避免双重历史 |
| Tag 策略 | `TPP_DONT` + 手动映射 | 避免默认 float relative-rate 边界歧义 |
| `forecast` | `ceil(nout/65)*48`；upstream `done()` 时改请求 1/0 | 禁止 `(H-1)` 外加历史请求（会卡死 EOS 尾） |
| EOS | `reader->done()` 检测 + `core.flush()` 零填充尾 | 复现 upfirdn 完整长度 |

**Core 算法**：polyphase 65 臂，相位律  
`y[m]=Σ_k h[(m·48 mod 65)+65k]·x[⌊m·48/65⌋−k]`，  
`Lout=ceil(((N−1)·65+T)/48)`。热路径：每 `process()` 一次拼装  
`[hist(H−1)|inputs]` 工作缓冲，再对 oldest→newest 窗口做  
`volk_32fc_32f_dot_prod_32fc`（reversed taps）；保留 scalar 参考核  
（`force_scalar_kernel()`，与 VOLK 逐样本 max_abs=0）。

### 4.2 upfirdn golden 误差（quality T=5363，`tol_abs=2e-3`）

容差依据：float32 golden 导出 + H=83 MAC 累加；实测误差 ≪ 2e-3。

| golden | N_in | N_out | max_abs | rel_L2 | corr | gain_err | phase_err (rad) |
|---|---:|---:|---:|---:|---:|---:|---:|
| impulse | 4096 | 5658 | 0 | 0 | 1.0 | 0 | 0 |
| dc | 4096 | 5658 | 2.4e-7 | 8.3e-8 | 1.0 | −3.7e-9 | 0 |
| tone_low | 4096 | 5658 | 2.4e-7 | 8.1e-8 | 1.0 | ~0 | ~0 |
| tone_pb (300 MHz) | 4096 | 5658 | 1.9e-7 | 7.8e-8 | 1.0 | ~0 | ~0 |
| tone_sb (420 MHz) | 4096 | 5658 | 2.4e-7 | 7.9e-8 | 1.0 | ~0 | ~0 |
| random | 4096 | 5658 | 4.8e-7 | 8.8e-8 | 1.0 | ~0 | ~0 |
| uwb | 6090 | 8358 | 1.2e-7 | 8.2e-8 | 1.0 | ~0 | 0 |
| volk vs scalar (impulse) | — | — | **0** | — | — | — | — |

短输入长度（N=0,1,47,48,49,95,96）均满足 `ceil(((N−1)·65+T)/48)`。  
chunk 不变性：oneshot / 48-split / 4096 / mixed{1,2,47,48,49,rand} + flush → **逐样本一致**。

### 4.3 chunk / scheduler / tag QA

| 测试 | 结果 |
|---|---|
| chunk 不变性（core） | **PASS**（exact） |
| block vs core vs golden (random) | **PASS**（max_abs 4.8e-7） |
| 有限 source EOS + flush 全长 | **PASS**（4096→5658） |
| forecast 无活锁 / 小 buffer | **PASS** |
| stop/restart + reset API | **PASS** |
| tag map 律 `round((p·65+(T−1)/2)/48)` | **PASS**（p=0→56） |
| rx_time 偏移映射 + 值不变 | **PASS**（offsets 0/1/47/48/49） |
| rx_rate → 998.4e6 | **PASS** |
| overflow → core.reset + `resampler_reset` tag | **PASS** |
| tag_propagation_enable=false 丢弃全部 | **PASS** |
| 两个 rx_time epoch | **PASS** |

CTest：`11/11 passed`（原 9 + `qa_uwb_rational_resampler_core` + `qa_uwb_rational_resampler`）。

### 4.4 Benchmark：A（内置）vs B（固定 kernel / block）

命令（`target_input_samples=5e7`，`buffer_items=1<<20`，7 轮 median）：

```bash
cd gr-uwb/build
./apps/benchmark_resampler_65_48 builtin 50000000
./apps/benchmark_resampler_65_48 kernel  50000000
./apps/benchmark_resampler_65_48 block   50000000
```

| 模式 | profile | taps | kernel | median 输入 MS/s | median 输出 MS/s | RT ratio | vs A quality |
|---|---|---:|---|---:|---:|---:|---|
| source_only | — | — | — | **1473** | — | — | — |
| **A builtin** | default | 2145 | volk_fir (GR) | 71.0 | 96.1 | 0.096 | — |
| **A builtin** | quality | 5395 | volk_fir (GR) | **44.6** | 60.3 | 0.060 | 1.0× |
| **A builtin** | realtime | 2665 | volk_fir (GR) | 50.0 | 67.8 | 0.068 | — |
| **B kernel** | quality | 5363 | volk_32fc_32f_dot_prod | **51.2** | 69.4 | 0.069 | **1.15×** |
| **B kernel** | realtime | 2635 | volk_32fc_32f_dot_prod | 55.9 | 75.7 | 0.076 | — |
| **B block** | quality | 5363 | volk_32fc_32f_dot_prod | **61.3** | 83.0 | 0.083 | **1.37×** |
| **B block** | realtime | 2635 | volk_32fc_32f_dot_prod | **69.1** | 93.5 | 0.094 | **1.55×** vs A rt |

### 4.5 是否达到最低实时门槛 / 剩余时间去向

| 门槛 | 要求 | Level B 最优（block realtime） | 结论 |
|---|---|---|---|
| 最低可行 | 输入 ≥ 737.28，输出 ≥ 998.4 | 输入 **69.1** / 输出 **93.5**（RT ≈ **0.094×**） | **未达到（FAIL）** |
| 生产 1.2× | 输入 ≥ 884.7 / 输出 ≥ 1198 | 同上 | **未达到（FAIL）** |

**相对 Level A**：固定 block 在 **相同 quality taps** 上约 **1.37×** 于内置  
`rational_resampler_ccf`；realtime 约 **1.38×**。正确性（upfirdn）已锁定。

**Profile 结论（剩余时间）**：

1. **FIR 算术为主**（quality ≈ 83 MAC/out × 998.4e6 ≈ 8.3e10 complex-MAC/s 量级需求；  
   单核 AVX2 理论峰值不足以吃满合同速率）。  
2. 早期 scalar 参考仅 ~8 MS/s in；改为 **滑动 work 缓冲 + VOLK** 后跳到 ~50–70 MS/s，  
   证明原先瓶颈在 **per-output delay memmove / window 重建**，现已消除。  
3. Source-only ~1.4 GS/s → **非** 内存/flowgraph 带宽瓶颈。  
4. 再提速方向（需独立 profile 后择一，禁止一次改多项）：  
   - 固定 65 输出宏块展开、减少 phase 分支；  
   - 手写 AVX2 多输出并行（有 halo 的有序 chunk）；  
   - Level C SC16 融合路径降低带宽；  
   - 多 worker 有序 FIR（最后手段）。

**文件**：

```
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_ccf_65_48.h
gr-uwb/lib/uwb_rational_resampler_ccf_65_48.cc
gr-uwb/lib/qa_uwb_rational_resampler_core.cc
gr-uwb/lib/qa_uwb_rational_resampler.cc
gr-uwb/grc/uwb_rational_resampler_ccf_65_48.block.yml
gr-uwb/apps/benchmark_resampler_65_48.cc   # +kernel/block/builtin modes
gr-uwb/python/uwb/bindings/python_bindings.cc  # bind_rational_resampler_ccf_65_48
```

---

## 5. Phase 3：性能优化（evidence-first）

> 日期：2026-08-11  
> 平台：同 §1.1（i7-12700，20 逻辑核，8P+4E，AVX2+FMA，`-march=native -O3`）  
> 合同：输出 998.4 MS/s；quality H=83 → 82.9 GMAC/s；realtime H=41 → 40.9 GMAC/s  
> 单 P-core 天花板 ≈ 36 GMAC/s → 单核最多 ~0.43× RT (quality) / ~0.88× RT (realtime)

命令约定：

```bash
cd gr-uwb/build
./apps/benchmark_resampler_65_48 profile 20000000          # Stage 1
./apps/benchmark_resampler_65_48 kernel|block 50000000 1048576 <kernel> <workers>
# kernel: volk_legacy | default | avx2 | volk_macroblock | scalar_legacy
# workers: 1..N（有序 multi-worker FIR）
ctest --test-dir gr-uwb/build --output-on-failure   # 11/11 PASS
```

### 5.1 Stage 1 — Profile 归因（core，20e6 输入，chunk=1M）

| profile | kernel | assemble % | schedule % | FIR % | state % | in MS/s | out MS/s | GMAC/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| quality | volk_legacy (Phase-2) | 2.8 | 0 | **97** | ~0 | 53.6 | 72.5 | 6.0 |
| quality | volk_macroblock | 3.4 | 1.8 | **91** | ~0 | 56.8 | 76.9 | 6.4 |
| quality | avx2_fma_macroblock | 2.6 | 1.5 | **92** | ~0 | 45.4 | 61.5 | 5.1 |
| quality | scalar_legacy | 3.4 | 0 | **97** | ~0 | 46.9 | 63.5 | 5.3 |
| realtime | volk_legacy | 2.9 | 0 | **97** | ~0 | 53.0 | 71.8 | 2.9 |
| realtime | volk_macroblock | 3.5 | 2.1 | **89** | ~0 | 66.0 | 89.4 | 3.7 |
| realtime | **avx2_fma_macroblock** | 4.5 | 2.9 | **87** | ~0 | **79.5** | **107.7** | **4.4** |
| realtime | scalar_legacy | 3.3 | 0 | **97** | ~0 | 63.1 | 85.5 | 3.5 |

**结论**：

1. **FIR 算术主导**（87–97%）。work_ 拼装仅 ~3%；state 可忽略。  
2. 剩余空间在 FIR 内核本身与并行，而非 phase 控制（宏块 schedule 仅 2–3%）。  
3. **kernel 模式常慢于 block**：timed 路径内同一线程对每个输出做 FNV checksum；block 模式由 GR 调度把 `ChecksumSink` 与 resampler 叠在不同线程。属测量形态差异，不是 kernel 真的更慢。  
4. 单核实测 ~5–7 GMAC/s（quality）/ ~3–4.5 GMAC/s（realtime）≈ 单核天花板的 **10–15%**（常数开销 + 非理想向量化 + 滑动窗带宽）。

默认内核选择（构造期，profile 证据）：

- `H ≤ 48`（realtime taps）→ `avx2_fma_macroblock`  
- `H ≥ 64`（quality taps）→ `volk_macroblock`（本机长臂上 AVX2 unpack 路径输给 VOLK）

### 5.2 Stage 2 — 单核宏块内核 A/B

7 轮 median，`target=5e7`，`buffer=1<<20`。

| 模式 | profile | kernel | median in MS/s | median out MS/s | RT ratio | GMAC/s | vs Phase-2 / legacy |
|---|---|---|---:|---:|---:|---:|---|
| kernel | quality | volk_legacy | 47.6 | 64.4 | 0.065 | 5.3 | baseline |
| kernel | quality | **volk_macroblock** | **55.4** | **75.0** | **0.075** | **6.2** | **1.16×** |
| kernel | realtime | volk_legacy | 49.2 | 66.6 | 0.067 | 2.7 | baseline |
| kernel | realtime | **avx2_fma_macroblock** | **76.7** | **103.9** | **0.104** | **4.3** | **1.56×** |
| block | quality | volk_legacy | 51.2 | 69.4 | 0.070 | 5.8 | Phase-2-class |
| block | quality | **volk_macroblock** | **62.1** | **84.2** | **0.084** | **7.0** | **1.21×** |
| block | realtime | volk_legacy | 53.3 | 72.1 | 0.072 | 3.0 | Phase-2-class |
| block | realtime | **avx2_fma_macroblock** | **88.1** | **119.3** | **0.120** | **4.9** | **1.65×** |

正确性：macroblock vs legacy max_abs = 0（同 FIR）；default vs scalar max_abs < 1e-4；golden max_abs ≤ 4.8e-7。  
CTest：**11/11 PASS**。

### 5.3 Stage 3 — 单核天花板诚实检查

| 项 | quality | realtime |
|---|---|---|
| 合同所需 GMAC/s | 82.9 | 40.9 |
| 单 P-core 理论天花板 | ~36 | ~36 |
| 理论最大 RT | **0.43×** | **0.88×** |
| 实测最优单核 block RT | 0.084× | 0.120× |
| 占单核天花板 | ~17% of 36 GMAC/s | ~14% of 36 GMAC/s |
| **是否达最低门槛（in ≥ 737.28）** | **否** | **否** |

单核不可能（quality）/几乎不可能（realtime）满足 737.28 MS/s 输入门槛；停止单核微调和继续 Stage 4。

### 5.4 Stage 4 — 有序 multi-worker FIR A/B

实现：core 内 **持久线程池**；串行 schedule 物化 `(arm, win_start)`；并行 FIR；有序写回；`nworkers=1` 保留为 fallback。discontinuity 走既有 `core.reset()`（单 state）。QA：core workers=1 vs N **bitwise**；block 容差 < 1e-5；EOS 全长；overflow reset。

| 模式 | profile | workers | median in MS/s | median out MS/s | RT ratio | GMAC/s | scale vs w=1 |
|---|---|---:|---:|---:|---:|---:|---:|
| block | quality | 1 | 59.7 | 80.8 | 0.081 | 6.7 | 1.0× |
| block | quality | 8 | 167.2 | 226.4 | 0.227 | 18.8 | **2.8×** |
| block | quality | 20 | 183.8 | 248.9 | 0.249 | 20.7 | **3.1×** |
| block | realtime | 1 | 88.2 | 119.4 | 0.120 | 4.9 | 1.0× |
| block | realtime | 8 | 211.2 | 286.0 | 0.287 | 11.7 | **2.4×** |
| block | realtime | **20** | **217.0** | **293.8** | **0.294** | **12.1** | **2.5×** |
| kernel | quality | 20 | 140.1 | 189.7 | 0.190 | 15.7 | — |
| kernel | realtime | 20 | 157.8 | 213.7 | 0.214 | 8.8 | — |

扩展性在 ~8 workers（≈ P-core 数）后饱和：schedule 串行、滑动窗 cache 冲突、超订 20 线程、FIR 读带宽。全核理论 300+ GMAC/s 未触及——实现仍 memory/sync 受限。

### 5.5 最终裁决 vs 门槛

| 门槛 | 要求 | 最优配置 | 结果 |
|---|---|---|---|
| **最低可行** | in ≥ 737.28 / out ≥ 998.4（RT ≥ 1.0） | block realtime, workers=20：**in 217 / out 294（RT 0.29×）** | **FAIL** |
| **生产 1.2×** | in ≥ 884.7 / out ≥ 1198 | 同上 | **FAIL** |

相对 Phase-2 block realtime（~69 in / 0.094×）：单核优化 → **88 in / 0.12×**（~1.3×）；multi-worker → **217 in / 0.29×**（~3.1× Phase-2）。

**命中停止/转向条件（方案 §12）**：

1. ✅ 已完成 profile + **两种独立优化**（宏块+AVX2/VOLK；有序 multi-worker），flowgraph **仍低于实时 15% 以上**（0.29 ≪ 0.85）。  
2. 未触发：CF32 带宽饱和单独证明（source-only 仍 ~1.4 GS/s）。  
3. 未触发：未以降滤波质量换吞吐。  

**推荐下一架构**（有 profile 证据）：

1. **首选**：原生率 scheduled 截窗 + **仅对 PDU 做 65/48**（已知 t0/T 时用 `UwbScheduledExtractor` 在 737.28 M 域截包，包级短 FIR，占空比 ≪ 1 → 等效算力需求下降一个数量级以上）。  
2. **备选**：RFNoC/FPGA 连续重采样（主机侧不再扛 82.9 GMAC/s 连续流）。  
3. **不建议**：继续在固定 65/48 连续 CF32 块上做无证据微调；单核已探到 FIR 主导且 multi-worker 扩展性见顶（~3× / 20 线程）。

正确基线保留：golden max_abs ≤ 4.8e-7，chunk 不变，11/11 CTest，scalar reference 仍在。

**Phase-3 文件变更**：

```
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_core.h   # profile, macroblock, AVX2, thread pool
gr-uwb/include/gnuradio/uwb/uwb_rational_resampler_ccf_65_48.h  # num_workers, set_kernel
gr-uwb/lib/uwb_rational_resampler_ccf_65_48.cc
gr-uwb/lib/qa_uwb_rational_resampler_core.cc   # kernel/mt/profile QA
gr-uwb/lib/qa_uwb_rational_resampler.cc        # multi-worker block QA
gr-uwb/apps/benchmark_resampler_65_48.cc       # profile mode, kernel/workers args
gr-uwb/python/uwb/bindings/python_bindings.cc
gr-uwb/lib/CMakeLists.txt / apps/CMakeLists.txt  # Threads::Threads
docs/performance/测试报告_GNURadio软件升采样_65_48.md  # 本 §5
```

---

## 6. Phase 3.5：最小阶数滤波器复测（相对 70 dB 口径）

> 依据 `分析与下阶段建议_GNURadio软件升采样_65_48.md` §3.1：原滤波器的 70 dB
> 阻带按**绝对**幅度验收，而正确判据是**相对通带** `20log10(|Hstop|/|Hpass|) ≤ −70 dB`。
> 原 quality/realtime 相对阻带实际约 **−129.6/−129.4 dB**，超规格 ~60 dB。

### 6.1 滤波器重设计（`testdata/design_resampler_minorder.py`）

| 档位 | 通带 B | 原 taps / arm | **新 taps / arm** | 相对阻带 | 通带纹波 | 群时延(输出) |
|---|---|---|---:|---:|---:|---:|---:|
| quality | 330 MHz | 5363 / 83 | **2707 / 42** | −70.03 dB | 0.005 dB | 28.19 |
| realtime | 290 MHz | 2635 / 41 | **1319 / 21** | −70.07 dB | 0.005 dB | 13.73 |

- 约 **50%** 计算量降幅；与建议文档估算（2749/43、1319/21）一致。
- Kaiser 最小阶数（scipy `kaiserord` width 归一化到 Nyquist：`transition/(R_V/2)`）。
- taps 文件：`taps_{quality,realtime}_minorder.txt`；golden：`golden_*_minorder/`；
  canonical `taps_{quality,realtime}.txt` 保留未动。

### 6.2 正确性

- 固定 block vs minorder golden：**7/7 逐样本匹配**（max_abs ≤ 1.2e-7，长度精确）。
- §8.5 UWB 回归：quality_minorder / realtime_minorder 均 **FCS pass、status success**，
  detected_start 相对原流偏移 56 / 41 样本（minorder 往返群时延）。

### 6.3 实测吞吐（`--taps` 支持新增；不按 taps 比例推算）

| 档位 | workers | 输入 MS/s | 输出 MS/s | RT ratio |
|---|---:|---:|---:|---:|
| realtime_minorder | 1 | 136.6 | 185.0 | 0.185 |
| realtime_minorder | 8 | 250.7 | 339.5 | 0.340 |
| realtime_minorder | 20 | 260.7 | 353.0 | 0.354 |

### 6.4 结论

- taps 减半：单核 +55%（88→137 MS/s），但 **多核 8→20 worker 几乎不缩放**
  （250.7→260.7 MS/s），多核瓶颈是串行 schedule 走查 / 缓存流量 / worker 同步，
  **不是 FIR 算力**。
- 最优 realtime_minorder 20 worker 仅 **0.354× RT**，远低于建议的 0.85× 门槛。
- **连续主机实时路径正式关闭**。按建议 §3.1 item 5 → 转向 **PDU 架构（§3.2）**。

---

## 7. PDU 级 65/48 重采样（`UwbPduRationalResamplerCcf65_48`）

> 依据 `分析与下阶段建议_GNURadio软件升采样_65_48.md` §3.2：连续 CF32 主机升采样
> 已关闭（最优 0.354× RT）；主架构改为 **737.28 MHz scheduled 截窗 → PDU 级 65/48 →
> 998.4 MHz 解调**。
>
> 日期：2026-08-11  
> 平台：同 §1.1（i7-12700，GR 3.10.1.1，VOLK 2.5.1，gcc）

### 7.1 Block 设计

| 项 | 选择 |
|---|---|
| Block 类型 | `gr::block`，**无 stream 端口** |
| Scheduler | 仅 message handler；无 `forecast` / `general_work` |
| 输入 | message `"packet"`（`UwbScheduledExtractor` 窗口 PDU，CF32） |
| 输出 | message `"packet"`（重采样后 PDU）、可选 `"status"` |
| Core | 复用 `core::RationalResampler65_48Core`（每 PDU 一次 `reset` + `process` + `flush`） |
| 热路径分配 | 预分配 scratch；仅当窗口变长时 `resize`（extractor 几何固定时零扩容） |
| 默认 taps | **quality_minorder**（2707 taps，相对 −70 dB）；吞吐档 realtime_minorder |
| 默认输出率 | 998.4e6；`validate_input_rate` 默认 true（拒非 737.28e6） |
| `emit_policy` | `FullWindow`（默认）或 `CaptureOnly`（裁剪到 capture 区） |

### 7.2 坐标映射（群时延中心）

与流式 block / core tag 律一致：

```text
map(p) = round((p·65 + (T−1)/2) / 48)
```

输出 meta：

| 字段 | 公式 |
|---|---|
| `sample_rate` | `output_sample_rate`（998.4e6） |
| `window_start_sample` | `map(window_start_in)` |
| `pre_guard_samples` | `map(ws+pre) − map(ws)` |
| `capture_samples` | `map(ws+pre+cap) − map(ws+pre)` |
| `post_guard_samples` | `Lout − pre_out − cap_out`（FullWindow） |
| `predicted_start_sample` / `detected_start_sample` | `map(·)`（若输入存在） |
| `sample_count` | 发射长度 |
| 溯源 | `resample_interp=65`, `resample_decim=48`, `resample_filter_delay=(T−1)/2`, `input_sample_rate`, `output_sample_rate` |
| 保留 | `packet_id`, `schedule_index`, `schedule_generation` |

`Lout = ceil(((N−1)·65 + T)/48)`。`CaptureOnly` 裁剪 buffer 切片
`[pre_out, pre_out+cap_out)`，并置 `pre_guard=0`、`sample_count=cap_out`。
输入 guard 过短或裁剪越界时 clamp，并 **一次性** 发 status `"short_guard"`。

### 7.3 QA 结果（`qa_uwb_pdu_rational_resampler.cc`）

CTest：**12/12 PASS**（原 11 + 本块）。

| # | 测试 | 结果 | 关键指标 |
|---|---|---|---|
| 1 | PDU == upfirdn / golden quality_minorder | **PASS** | N=6090→8302，max_abs **1.19e-7**；capture 内部 1.19e-7 |
| 2 | 坐标映射 | **PASS** | ws 50000→67737；pre/cap/post 1354/2708/1540；pred 51000→69091；Lout=5602 |
| 3 | CaptureOnly 裁剪 | **PASS** | n=7207=cap_out；与 full-upfirdn 切片 max_abs **0** |
| 4 | bad_input_rate | **PASS** | dropped=1，status `bad_input_rate`，无崩溃 |
| 5 | short_guard | **PASS** | Lout=142，clamp 无 OOB，status 仅首次，events≥2 |
| 6 | e2e scheduled→PDU resamp→demod | **PASS** | 见下 |
| 7 | 吞吐 sanity（realtime_minorder） | **PASS** | 见下 |

#### e2e FCS（§7.3 #6）

流水线：

1. 998.4 cfile 片段经 realtime taps×(48/65) 做 48/65 抗混叠抽取 → 737.28 流  
2. `UwbScheduledExtractor`（sample_rate=737.28e6，pre/cap/post=8000/240000/4000）  
3. `UwbPduRationalResamplerCcf65_48`（quality_minorder，FullWindow）  
4. `UwbRealtimeDemodulator`（sfd_mode=`ieee`）

| 项 | 值 |
|---|---|
| FCS | **pass** |
| status | **success** |
| detected_start | 300084 |
| mapped predicted | 300056 |
| \|det − pred\| | **28**（≈ quality_minorder 群时延 `map(0)`） |

#### 吞吐 vs slot 率（§7.3 #7）

纯 PDU 重采样（无 demod），窗口 152k 样点 @737.28，**realtime_minorder**（1319 taps），80 PDU：

| 指标 | 值 |
|---|---:|
| pdus/s | **575** |
| 输入 MS/s（仅窗口样点） | **87.4** |
| 输出 MS/s | **118.4** |
| vs 连续最优输入（260.7 MS/s @0.354× RT） | 单核 PDU FIR 约 0.34× 该“满流”数字，但**只处理占空比内样点** |
| headroom vs 200 slot/s | **2.88×** |
| headroom vs 1000 slot/s | **0.58×**（大窗+单核 minorder；减窗/降 taps/多核 core 可抬升） |

相对连续路径的关键优势：连续路径必须扛满 737.28 MS/s 输入（实测最优仅 0.354× RT）；
PDU 路径只对雷达窗做 FIR。在 QM35825 典型 200 slot/s、占空比 ≪ 20% 时，
**575 PDU/s ≈ 2.9× 余量**，远高于连续路径的实时缺口。

正确性默认仍用 **quality_minorder**；吞吐档用 realtime_minorder。

### 7.4 文件清单

```
gr-uwb/include/gnuradio/uwb/uwb_pdu_rational_resampler_ccf_65_48.h
gr-uwb/lib/uwb_pdu_rational_resampler_ccf_65_48.cc
gr-uwb/lib/qa_uwb_pdu_rational_resampler.cc
gr-uwb/grc/uwb_pdu_rational_resampler_ccf_65_48.block.yml
gr-uwb/python/uwb/bindings/python_bindings.cc   # bind_pdu_rational_resampler_ccf_65_48
gr-uwb/lib/CMakeLists.txt / include/.../CMakeLists.txt / grc/CMakeLists.txt
gr-uwb/examples/x410_native_rate_scheduled_pdu_resample.grc  # 已存在，现可绑定本块
docs/performance/测试报告_GNURadio软件升采样_65_48.md       # 本 §7
```
