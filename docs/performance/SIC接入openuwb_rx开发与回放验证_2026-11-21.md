# SIC 接入 openuwb_rx：开发与软件回放验证（2026-11-21）

状态：**代码完成、四预设构建、单元/控制台回归、软件回放 ABBA、
全窗盲搜稀疏定时核与间隔局部探测复测已完成；未上板，未做 MATLAB
交叉验证。** 本轮没有新建 `openuwb_rx_sic`。

- 分支：`feature/openuwb-rx-sic`
- 基线：`main @ 91f0b23`（本轮改动尚未提交）
- 构建：`native-avx2`、`no-uhd` 全量通过；`native-avx512` 仅编译通过
  （本机 i7-12700 无 AVX-512）
- 环境：Intel i7-12700（WSL2，Linux 6.18），GCC 11.4.0，Release，UHD OFF
  用于回放；`native-avx2` 预设为 AVX2/FMA

## 1. 实现

### 1.1 `PacketProcessor` 可选 SIC 级

`include/openuwb/runtime/live_packet_processor.h` 新增：

- `PacketSicOptions { enabled, SicUnifiedConfig, SicUnifiedTemplates }`。
- `PacketProcessingResult` 新增 `sic_executed / sic_ok / sic_committed /
  sic_level / sic_reason / sic_suppression_db / sic_end`。
- 只有 `enabled` 时构造函数才调用 `prepare_sic_unified()`，并预分配
  `SicUnifiedScratch`、`sic_out_`；`--sic off` 不分配这些缓冲。live 路径
  不消费重建干扰 `i_hat`，因此不再分配/清零它。
- `process()` 在 `core::resample_sc16_window()` 之后调用
  `sic::sic_process_unified()`。只有同时满足：
  - `sic_ok == true`，
  - `level ∈ {L1_Preamble, L2_FullPacket}`，
  - `committed == true`，

  才把 `sic_out_` 送入目标 `demodulate_one()`；否则原样使用 `work_`，即
  fail-open。SIC 前后的耗时边界保存在 `resample_end / sic_end / demod_end /
  compute_end`，热路径不分配、不 resize。
- L0/fail-closed 路径不再做整窗内存操作：`SicUnifiedResult` 新增
  `copy_out_on_bypass`（默认 `true` 保持库契约），live 路径设 `false`，
  因为 `out` 只在 committed L1/L2 时被消费；同时 `i_hat = nullptr`。
  25 ms 夹具上这部分 `out` 拷贝 + `i_hat` 清零约 **0.58 ms/候选**。
- ROI 的窗口容量在 worker 初始化时确定；每次调用只把配置结构体按值浅拷贝，
  并把 ROI/sub-ROI 裁剪到本次实际产生的 `made` 个样本，避免
  `roi_hi > n` 的 `invalid_roi`，也不产生堆分配。

### 1.2 CLI

`apps/live_rx_config.{h,cc}` 新增并 fail-closed 校验：

| 参数 | 默认 | 允许值 |
| :--- | :--- | :--- |
| `--sic` | `off` | `off`, `preamble`, `full`, `auto` |
| `--sic-backend` | `ipatov` | `ipatov`, `discrete`, `path` |
| `--sic-code` | `11` | 9..12 |
| `--sic-reps` | `128` | 1..2048；Ipatov 仅 64/128/256 |
| `--sic-sfd` | `dw-8` | `dw-8`（映射 `decawave`）、`4z1/2/3/4`、`ieee` |

非法输入统一在打开 source 前抛错，进程退出码 2；`--sic off` 时既有
`START/PACKET/SUMMARY/FINAL/...` 输出完全不变。

### 1.3 Session 与统计

`apps/live_rx_session.cc`：

- `Workers` 初始化时构造一次 `SicTemplateBundle`（干扰源 SYNC + 按
  `--sic-reps` 平铺的 preamble），以 `shared_ptr` 只读共享给每个 worker；
  模板由 `mod::modulate_prefix_waveform()` 生成，和 TX 调制一致。
- 每个 worker 的 `PacketProcessor` 获得 `PacketSicOptions`；`--sic off` 时
  不创建模板、不准备 scratch。
- `PacketCost` 增加 `sic_us / sic_executed / sic_level`；`demod_us` 的起点
  改为 `sic_end`（off 时 `sic_end == resample_end`）。
- `START` 仅在 `--sic != off` 时追加
  `sic=... sic_backend=... sic_code=... sic_reps=... sic_sfd=...`。

`apps/console_reporter.{h,cc}`：

- PACKET 行仅在 SIC 开启时追加 `sic_us= / sic_lvl=L0|L1|L2 / sic_exec=`。
- SUMMARY 仅在 SIC 开启时追加 `sic_mean / sic_p95`。
- FINAL 前新增独立 `SIC_STATS executed=N l0=.. l1=.. l2=.. mean_us=.. max_us=..`。
- `--sic off` 不输出上述任何字段。

### 1.4 SIC 库的必要扩展

为了支持默认干扰配置（code 11 / 128 SYNC / DW-8）：

- `lib/sic/uwb_discrete_cir_estimator.cc`：Ipatov 几何从 64/256 扩到
  64/128/256；Ipatov 脉冲前缀的 SFD 改为 `cfg.sfd_mode`，不再硬编码 `4z2`。
- `lib/sic/uwb_sic_unified.cc`：Ipatov 盲搜时
  `timing_max_backtrack_symbols >= sync_reps`。否则 stride-14 粗搜落到
  128-SYNC 前导后段时，默认 40 符号回溯无法回到首 SYNC，SFD 会晚 128 个符号
  而必然失败。
- `fullpacket_cancel()` / `fullpacket_cancel_discrete()`：保留直接调用者的
  历史 code10/64/4z2 严格门；统一 SIC 通过 scratch 上的
  `allow_configured_profile`（离散路径由 `prepare_discrete_cir()` 带入，
  全包路径由 `prepare_sic_unified()` 设置）显式允许配置的 code/SYNC/SFD。
  公开函数签名未变。

### 1.5 全窗盲搜定时核

`SicUnifiedConfig` 在 Receiver Optimization 区新增
`rx_timing_sparse / rx_timing_shaped_sparse / cfo_refit_mode`，默认
`false / false / 0`，因此既有 `qa_uwb_sic_unified` 与
`uwb_uhd_sc16_pipeline_runner` 基线不变。`uwb_sic_unified.cc` 把它们一次
赋给 Ipatov/Discrete/Path 共用的 `Qm35825Profile`。

`apps/live_rx_session.cc` 在组装 `PacketSicOptions` 时与目标解调器的 acq
配置同步：

```cpp
sc.rx_timing_sparse        = (o.timing_sparse == "on");
sc.rx_timing_shaped_sparse = (o.timing_shaped == "on") ||
                             (o.timing_shaped == "auto" &&
                              o.timing_sparse == "off");
sc.cfo_refit_mode          = (o.cfo_refit == "auto") ? 1
                           : (o.cfo_refit == "skip") ? 2 : 0;
```

默认 `--timing-sparse on`（`--timing-shaped auto`）因此让 SIC 使用
64-entry 稀疏 ternary 定时核，而不是在 ~609 k 样点全窗上跑 1016-tap
稠密匹配滤波。ROI 仍保持 `[0, made)` 全窗盲搜，未收窄。shaped-sparse
在模板满足 `code*p` 时可进一步精确分解；不满足时自动回退。

### 1.6 宽窗间隔局部探测 + 多符号相干折叠

在稀疏定时核之上，本轮又为**全窗盲搜**增加了间隔局部探测：

- `Qm35825Profile` 新增 `rx_wide_interval_probe /
  rx_wide_probe_step_symbols / rx_wide_probe_fold_symbols /
  rx_wide_probe_fallback`，默认 `false / 42 / 4 / false`，因此
  `LockedBounded`、窄 ROI 与既有调用方保持原行为。
- `SicUnifiedConfig` 提供同名转发字段（默认关闭）；`openuwb_rx` 在
  `--sic != off` 时显式打开；`uwb_sic_unified.cc` 原样写入 Ipatov/Discrete/
  Path 共用的 `Qm35825Profile`。
- `stage_timing` 仅在 `AcquireLegacy` 且实际搜索跨度
  `j_hi - j_lo >= 16*P` 时启用。每个探测起点 `p` 把 `K=4` 个相隔 P 的
  SYNC 相干叠加到 `2P-1` 缓冲，再做 P lag 归一化相关；峰值
  `p + peak.lag` 是真实 SYMBOL 边界，随后走既有回溯、SYNC 跟踪和 SFD。
  K=4 相对单 SYMBOL 增益 +6 dB（K=2 为 +3 dB）。默认取 K=4；50 kHz CFO
  下四符号相位差 ≤0.32 rad，损失很小，100 kHz 下约 −2.4 dB，现有
  24/24 碰撞用例仍全部通过。若现场 CFO 更大，可把
  `rx_wide_probe_fold_symbols` 调回 2。
- 归一化仍使用 `|corr|^2 / (power * Et)`，功率在折叠后的同一缓冲上统计；
  不额外乘 K，否则高 SNR 的完美 K 折会被错误压到 `1/K`。这一点与既有
  `fold_acquire()` 一致。
- 探测步长 M=42 个 SYMBOL（SIC 默认 `sync_reps=128` 时正好是
  `max(8, sync_reps/3)` 上限），并依次收紧到 `max(8, R/3)`、`R-K` 与
  `timing_max_backtrack_symbols`。`M=42 ≤ R-K=124`，仍保证任意前导内
  至少有一个探测点；最坏回溯 42 个 SYMBOL。窗尾约 `(K-1)P` 个起点
  （K=4 时约 3 个 SYMBOL）因余量不足无法折叠，保留原边界。
- 全部探测点未命中且 `rx_wide_probe_fallback=false` 时立即返回
  `TimingFailed`，SIC 快速落 `L0_Bypass`，不再支付 stride-14 粗搜、
  整 SYNC 网格和 fold rescue；`rx_wide_probe_fallback=true` 可恢复旧回退链。

## 2. 测试

### 2.1 新增单元测试

`tests/unit/runtime/qa_uwb_live_packet_processor_sic.cc`（注册在
`tests/CMakeLists.txt`），6 个用例：

1. `disabled_sic_is_packet_equivalent`：同一 clean 目标包，默认
   `PacketProcessor` 与 `enabled=false` 的 SIC 处理器各跑 3 次，逐项比较
   status / PSDU / FCS / PHR bits / timing 起点 / SFD 起点 / CIR 首径 /
   CFO；`sic_executed=false`、`sic_end == resample_end`。
2. `fail_open_without_interferer_keeps_target`：只有目标包（无 code-11
   干扰），SIC 开启后必须落 `L0_Bypass`，解码结果与基线逐项一致。该用例
   同时覆盖间隔探测全部 miss 后的快速失败路径（不跑旧粗搜/fold）。
3. `collision_requires_sic_and_recovers_target`：目标 QM35825（code 9、
   64 SYNC、4z2、16 B PSDU）+ 强 DW1000 干扰（code 11、128 SYNC、DW-8、
   6 倍幅度），目标完整落在干扰 SYNC+SFD 区间。SIC off 失败；
   `preamble` 达 L1、`full`/`auto` 达 L2，三者目标 FCS 全通过且 PSDU 匹配。
   该用例对 `rx_timing_shaped_sparse=false/true` 各跑 3 种 policy，确认
   稀疏定时核不引入检测/恢复回归。
4. `shaped_sparse_collision_recovers_repeated_targets`：同几何、8 个不同
   payload、`rx_timing_shaped_sparse=true` 的 `auto`，8/8 提交且 FCS 通过。
5. `repeated_sic_windows_have_no_state_residue`：同一 SIC 处理器连续处理
   8 个不同 payload 的碰撞窗，每个都提交并 FCS 通过。
6. `wide_probe_collision_offsets_and_cfo_recover`：在 3 个干扰绝对位置
   （4 096 / 98 304 / 196 608）和 2 个 CFO（50/100 kHz）组合下，分别用
   `preamble`（L1）与 `auto`（L2）、`rx_timing_shaped_sparse=false/true`
   跑 24 组；全部 `committed`、等级 ≥L1、目标 FCS 通过且 PSDU 匹配。
   这组用例直接覆盖“窗内前/中/后位置 + CFO”的探测与回溯边界。

`qa_uwb_sic_unified.cc` 中“128 Ipatov 不支持”的断言改为 32，保留“不支持
几何 fail-closed”语义；128 由新增用例覆盖。

### 2.2 控制台回归

`tools/rx/qa_live_dw3000_console.py` 原有检查全部保留，新增：

- `--sic/--sic-backend/--sic-code/--sic-reps/--sic-sfd` 非法值退出码 2，
  且不打印 START；
- 默认 `--sic off` 不出现 `SIC_STATS`、`sic=`、`sic_us=`；
- `--sic auto --workers 1` 回放冒烟：START 参数、PACKET 的
  `sic_us/sic_lvl/sic_exec`、`SIC_STATS`、FINAL 均符合预期。

### 2.3 构建与测试结果

| 预设 | 结果 | 备注 |
| :--- | :--- | :--- |
| `native-avx2` | CTest 63 项：62 通过 + 1 跳过 + 0 失败 | 跳过项为无安装前缀的 install consumer |
| `no-uhd` | CTest 63 项：62 通过 + 1 跳过 + 0 失败 | 同上 |
| `native-avx512` | 全量编译通过 | 本机无 AVX-512，运行时按设计报
`UWB SIMD backend 'avx512' requires ISA ...`，未作为通过记录 |
| `scalar-debug` | 相关 3 项通过 | `demod_core`、`sic_unified`、`live_packet_processor_sic` |

所有命令均使用 `env -u LD_LIBRARY_PATH`。控制台脚本对
`build/native-avx2/apps/openuwb_rx` 通过。`uwb_qa_uwb_fixed_tone_cancel_source`
在一次全量首轮出现历史已知的偶发丢样，单项与随后整套复测均通过；与 SIC
路径无关。

## 3. 软件回放性能

### 3.1 `--sic off` vs `main`

- 夹具：`testdata/uwb_radar/tx_737p28.sc16` 以 1 ms 周期重复 100 次，
  737.28 MS/s SC16，100 ms；SHA-256
  `a49ddb25c7cd25e3925a23264bc56d3ed08de0b16d2dfb1af6aa1f9db7cb0e5a`。
- 命令：`openuwb_rx --replay ... --period-ms 1 --workers 4 --queue 24
  --summary 1000 --print summary`；串行 ABBA 6 配对，先做一次 A/B
  预热并丢弃。
- 结果（branch `--sic off` 相对 main 的中位数，负值表示 branch 更快）：

| 指标 | 中位数 | 6 轮范围 |
| :--- | ---: | :--- |
| `compute_mean` | −18.6% | −31.6% … −15.3% |
| `resample_mean` | −7.0% | −21.1% … −1.7% |
| `demod_mean` | −30.0% | −42.0% … −28.0% |

24 次运行全部 `completed=125/125`、`dropped_queue=0`、`fcs_ok=0`
（该夹具是 preamble 波形，没有可解码 payload）。两棵树的
`uwb_demod_pipeline.cc`、`demod_timing.h`、resampler 头/实现 SHA-256 完全
相同，因此这个方向性差异只能归因于二进布局/频率噪声，**不能解释为 SIC
优化**；可以确认的是 `--sic off` **没有性能退化**。

### 3.2 `--sic auto` 单/多 worker

- 夹具：同一 `tx_737p28.sc16` 以 25 ms 周期重复 20 次（0.5 s），SHA-256
  `1d9e1a42118b64a1c0e5cbaf72a653b43845f497347681bce93a0b903025cbb4`；
  autocorrelation 实际产生 144 个候选。**25 ms 是提交周期，不是窗口长度**：
  每个候选窗为 `pre_guard 36 864 + capture 329 161 + post_guard 8 192 =
  374 217` native samples，经 65/48 重采样后约 **506 773 work samples
  ≈ 499 SYMBOL**（@998.4 MS/s）。
- 3 次重复的中位数，同一夹具、同一命令、同一机器；优化前后对照：

| workers | 版本 | resample_mean / p95 (µs) | sic_mean / p95 (µs) | demod_mean / p95 (µs) | compute_mean (µs) | dropped | executed (L0/L1/L2) |
| ---: | :--- | ---: | ---: | ---: | ---: | ---: | :--- |
| 1 | 优化前（稠密定时核） | 790.3 / 1 015 | 10 624.9 / 12 154 | 619.0 / 961 | 12 069.2 | 83 | 60 (60/0/0) |
| 1 | 稀疏定时核 | 799.4 / 1 098 | **2 996.9 / 3 824** | 869.0 / 1 084 | **4 693.4** | **29** | **114 (114/0/0)** |
| 1 | 间隔探测 + K=2, M=16 | 832.4 / 1 002 | **820.0 / 1 055** | 690.6 / 968 | **2 300.6** | **0** | **143 (143/0/0)** |
| 1 | 间隔探测 + K=4, M=16 | 890.0 / 1 111 | **905.6 / 1 235** | 657.1 / 864 | **2 468.5** | **0** | **143 (143/0/0)** |
| 1 | K=4, M=42（旧 L0 拷贝/清零） | 866.0 / 1 067 | **703.5 / 934** | 653.9 / 847 | **2 210.4** | **0** | **143 (143/0/0)** |
| 1 | K=4, M=42 + L0 免拷贝（当前默认） | 837.6 / 976 | **124.0 / 157** | 650.6 / 819 | **1 612.3** | **0** | **143 (143/0/0)** |
| 4 | 优化前（稠密定时核） | 881.6 / 1 179 | 11 255.9 / 13 949 | 673.7 / 891 | 12 811.2 | 0 | 143 (143/0/0) |
| 4 | 稀疏定时核 | 969.9 / 1 355 | **3 408.3 / 4 520** | 969.8 / 1 353 | **5 253.6** | 0 | 143 (143/0/0) |
| 4 | 间隔探测 + K=2, M=16 | 1 010.3 / 1 374 | **1 036.5 / 1 454** | 767.8 / 1 086 | **2 824.8** | 0 | 143 (143/0/0) |
| 4 | 间隔探测 + K=4, M=16 | 1 067.2 / 1 418 | **1 203.7 / 1 742** | 780.1 / 1 189 | **3 051.0** | 0 | 143 (143/0/0) |
| 4 | K=4, M=42（旧 L0 拷贝/清零） | 1 076.3 / 1 645 | **1 003.1 / 1 628** | 849.1 / 1 439 | **2 892.5** | 0 | 143 (143/0/0) |
| 4 | K=4, M=42 + L0 免拷贝（当前默认） | 953.7 / 1 111 | **140.1 / 180** | 708.8 / 918 | **1 802.6** | 0 | 143 (143/0/0) |

> 表中 K=2/M=16、K=4/M=16 以及 M=42 的“旧 L0”行都包含旧的整窗
> `out` 拷贝 + `i_hat` 清零；只有“L0 免拷贝（当前默认）”行是去掉这两项
> 之后的结果。M=42 与 M=16 的纯探测对比见 3.3。

相对最初优化前中位数：当前默认单 worker `sic_mean` −98.8%、
`sic_p95` −98.7%、`compute_mean` −86.6%、`dropped` 83→0；
4 worker `sic_mean` −98.8%、`sic_p95` −98.7%、`compute_mean` −85.9%，
`dropped` 保持 0。相对上一轮“仅稀疏定时核”：单 worker
2 996.9→124.0 µs（−95.9%）；4 worker 3 408.3→140.1 µs（−95.9%）。

### 3.3 SIC 开销构成（当前默认 K=4/M=42，1 worker）

| 配置 | sic_mean (µs) | sic_p95 (µs) | 说明 |
| :--- | ---: | ---: | :--- |
| 旧 L0 路径（out 拷贝 + i_hat 清零） | 703.5 | 934.1 | 含约 582 µs 整窗内存操作 |
| 仅去掉 i_hat 清零 | 372.4 | 540.2 | 省约 331 µs |
| 再去掉 out 拷贝（当前默认） | 124.0 | 157.4 | 再省约 248 µs |
| K=4/M=16 + L0 免拷贝 | 288.5 | 372.4 | 纯探测对比：约 31 个 probe |

- 当前 124 µs 中，约 **102 µs 是探测点相关**（约 12 个 probe × 约
  8.5 µs），其余约 22 µs 是 SIC 包装、模板准备、窄 seed refine 等固定开销。
- M=16 的 288.5 µs 与 M=42 的 124.0 µs 相差 **164.5 µs**，对应约 19 个
  探测点，单 probe ≈ **8.5 µs**。这与“每个 probe = K 段折叠 + 1016 lag
  稀疏相关”的量级一致。
- 该夹具没有 code-11 干扰，全部 `L0_Bypass`，所以没有 CIR / comm demod /
  reconstruct / subtract 成本。命中路径的 SIC 成本主要由后续 CIR/重构/减法
  决定，不在本表；间隔探测只影响命中前的 timing 搜索。
- 4 worker 的 L0 内存优化同样明显：1 003.1→140.1 µs（−86.0%），说明
  整窗拷贝/清零在多线程下还会叠加带宽竞争。
- 按 `compute_mean` 拆分（1 worker，1 612.3 µs）：resample 837.6
  （52.0%）+ SIC 124.0（7.7%）+ demod 650.6（40.4%），retry≈0。
  SIC 已不再是单候选耗时的主要部分。

M=42 相对 M=16（同 K=4、L0 免拷贝）的纯探测收益：单 worker
288.5→124.0 µs（−57.0%），4 worker 329.3→140.1 µs（−57.5%）。
M=42 是 `sync_reps=128` 下的 `reps/3` 上限；`M=42 ≤ reps-K=124`，
仍保证每个前导至少被一个探测点覆盖，但最坏回溯从 16 个 SYMBOL 增加到
42 个 SYMBOL，弱前导/首段受干扰时比 M=16 更依赖回溯链。

同环境 `--sic off` 对照的 `demod_mean` 中位数为单 worker 871.6 µs、
4 worker 913.4 µs（上一轮记录）；本轮 `--sic auto` 的 `demod_mean`
为 650.6 / 708.8 µs，量级与历史波动一致，说明 `demod` 的变化仍来自运行
环境/CPU 频率，不是 SIC 改动；`resample`/`demod` 源码未改。

说明：

- `sic_us` 是**整窗盲搜**成本：SIC 不知道干扰位置，`roi` 覆盖整个
  work 窗。优化前用 1016-tap 稠密定时核，约 10–12 ms/窗；上一轮改用
  64-entry 稀疏定时核后约 3.0–3.4 ms/窗；间隔探测 + L0 免拷贝后，
  当前默认 K=4/M=42 约 **0.12–0.14 ms/窗**。ROI 未收窄，仍保留窗口
  边缘检测能力。
- 单 worker 现在能处理全部 143 个执行的候选且 `dropped=0`：25 ms 周期下
  候选到达间隔约 3.47 ms，当前默认单候选 `compute_mean≈1.61 ms`，
  仍有约 1.86 ms 余量。4 worker `sic_mean` 略高（0.14 ms）是线程/内存
  竞争，但仍是 0 丢包。
- 夹具中没有 code-11 干扰，因此全部 `L0_Bypass`；这也验证了探测全 miss
  后的快速 fail-open 不会破坏目标解码（目标本身是 preamble-only 波形，
  基线 FCS 也是 0）。
- 真实目标 FCS 的碰撞证据来自 2.1 的单元测试：SIC off 0/8，SIC
  `preamble/full/auto` 8/8（不同 payload 连续 8 包），新增
  `rx_timing_shaped_sparse=true` 的 8/8 用例同样通过；另有 3 个干扰位置
  × 50/100 kHz CFO × L1/L2 × dense/shaped 共 24 组全部提交且目标 FCS
  通过。本轮没有做“可解码目标包”的 live replay 夹具，不能把上面的
  live FCS=0 写成 SIC 失败。

## 4. 未完成 / 证据边界

- **未上板**：所有数据都是文件回放；没有 UHD 实采，没有空口碰撞。
- **未做 MATLAB 交叉验证**：SIC 的 Ipatov 128、DW-8 SFD、盲搜回溯均未与
  MATLAB 参考逐样点对齐。
- `--sic auto` 单 worker 经“稀疏定时核 + 间隔局部探测 + L0 免拷贝”后，
  M=42/K=4 默认 `sic_mean` 从 10.6 ms 降到 **0.124 ms**（旧 L0 拷贝/
  清零路径为 0.70 ms；M=16/K=4 免拷贝为 0.29 ms），25 ms 周期、
  144 候选/0.5 s 下 `dropped=0`；4 worker 也是 0 丢包。
- 间隔探测的边界：搜索窗最末尾约 `(K-1)P` 个起点（K=4 时约 3 个
  SYMBOL）因余量不足无法折叠，仍依赖原边界；M=42 仍满足
  `M ≤ reps-K`（128−4=124），每个前导至少有一个探测点，但最坏回溯
  42 个 SYMBOL，弱前导/首段受干扰时比 M=16 更依赖回溯链。
  `rx_wide_probe_fallback=false` 的快速失败是召回/延迟折中，若现场
  存在极弱前导可打开该开关恢复旧回退链。K=4 的 +6 dB 在低 CFO 下更好，
  但 100 kHz 时相干损失约 −2.4 dB；更高 CFO 应把
  `rx_wide_probe_fold_symbols` 调回 2。
- 是否默认开启 SIC、`L2` 对任意 code/SYNC/SFD 的长期稳定性、以及
  `discrete/path` 后端的 live 碰撞回归，均需后续上板/更多夹具验证。

## 5. 复现

```sh
env -u LD_LIBRARY_PATH cmake --preset native-avx2
env -u LD_LIBRARY_PATH cmake --build --preset native-avx2 -j
env -u LD_LIBRARY_PATH ctest --preset native-avx2 --output-on-failure

env -u LD_LIBRARY_PATH cmake --preset no-uhd
env -u LD_LIBRARY_PATH cmake --build --preset no-uhd -j
env -u LD_LIBRARY_PATH ctest --preset no-uhd --output-on-failure

env -u LD_LIBRARY_PATH cmake --preset native-avx512
env -u LD_LIBRARY_PATH cmake --build --preset native-avx512 -j
# 本机无 AVX-512，不运行 ctest

env -u LD_LIBRARY_PATH python3 tools/rx/qa_live_dw3000_console.py \
  build/native-avx2/apps/openuwb_rx

build/native-avx2/apps/openuwb_rx --replay <fixture> --summary 1000 \
  --print summary --period-ms 1 --workers 4 --queue 24
build/native-avx2/apps/openuwb_rx --replay <fixture> --summary 1000 \
  --print summary --period-ms 25 --workers 1 --queue 24 --sic auto
build/native-avx2/apps/openuwb_rx --replay <fixture> --summary 1000 \
  --print summary --period-ms 25 --workers 4 --queue 24 --sic auto
```

ABBA 原始脚本与 JSON 在 `/tmp/opencode/abba_off2.py`、
`/tmp/opencode/abba_off2.json`、`/tmp/opencode/auto_perf_w{1,4}.json`
（最初优化前基线，已另存 `auto_perf_w{1,4}_before.json`）、
`/tmp/opencode/auto_perf_after.py`、`/tmp/opencode/auto_perf_w{1,4}_after.json`
（稀疏定时核，已另存 `auto_perf_w{1,4}_prev.json`）、
`/tmp/opencode/auto_perf_probe.py`、`/tmp/opencode/auto_perf_w{1,4}_probe.json`
（间隔探测 K=2/M=16，已另存 `auto_perf_w{1,4}_k2.json`）、
`/tmp/opencode/auto_perf_k4.py`、`/tmp/opencode/auto_perf_w{1,4}_k4.json`
（K=4/M=16，已另存 `auto_perf_w{1,4}_k4m16.json`）、
`/tmp/opencode/auto_perf_k4m42.py`、`/tmp/opencode/auto_perf_w{1,4}_k4m42.json`
（K=4/M=42，旧 L0 路径，已另存 `auto_perf_w{1,4}_k4m42_with_ithat.json`）、
`/tmp/opencode/auto_perf_k4m42_noithat.py`（仅去掉 i_hat 清零）、
`/tmp/opencode/auto_perf_k4m42_nocopy.py`（再去掉 out 拷贝）、
`/tmp/opencode/auto_perf_k4m42_final.py`、`/tmp/opencode/auto_perf_w{1,4}_k4m42_final.json`
（当前默认；临时目录，未入库；夹具哈希见上）。
